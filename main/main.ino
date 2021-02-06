#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

#define TUNE_PIN           42
#define AUTOTUNE_PIN       44

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define STEP_ACCURACY      1
#define GEARBOX            15
#define MAX_STEP_SPEED     1500

#define LCD_RS             16
#define LCD_EN             17
#define LCD_PIN_1          23
#define LCD_PIN_2          25
#define LCD_PIN_3          27
#define LCD_PIN_4          29
#define LCD_WIDTH          20
#define LCD_HEIGHT         4

#define MAX_MENU           7
#define SETTING_MENU       4

#define ENCODER_LEFT_PIN   33
#define ENCODER_RIGHT_PIN  31    
#define ENCODER_CLICK_PIN  35
#define BUTTON_DEBOUNCE    10
#define BUZZER             -1 //37

#define SDCARDDETECT       49
#define SDCARDCS           53
#define SDCARDMOSI         51
#define SDCARDMISO         50
#define SDCARDSCK          52

#define LED_PIN            13

#define TEMP_INPUT_PIN_1   3
#define TEMP_INPUT_PIN_2   4
#define TEMP_INPUT_PIN_3   5
#define TEMP_OUTPUT_PIN_1  8
#define TEMP_OUTPUT_PIN_2  9
#define TEMP_OUTPUT_PIN_3  10

#define THRESHHOLD_HIGH    102
#define THRESHHOLD_LOW     98

#define FAN_PIN_PWM        6 //D40
#define FAN_PIN_TAC        2 //D3


struct Menu {
  int id = 0;
  int value = 0;
  char label[16];
};

int flag_left = 0;
int flag_right = 0;
int flag_click = 0;
int flag_temp = 0;
int temp_update = 0;
int buzzer_update = 0;
int flag_fan = 0;

AccelStepper motor = AccelStepper(AccelStepper::DRIVER, E_STEP_PIN, E_DIR_PIN);
int rpm_old = 0;
int rpm_value = 0;

LiquidCrystal lcd(LCD_RS,LCD_EN,LCD_PIN_1,LCD_PIN_2,LCD_PIN_3,LCD_PIN_4);
int menu = 0;
int menu_level = 0;
int menu_old = 0;
int menu_level_old = 0;
Menu rpm;
Menu temperature_nozzle;
Menu rpm_current;
Menu temperature_current_nozzle;
Menu temperature_current_preheat;
Menu temperature_preheat;
Menu fan_speed;
Menu* screen[MAX_MENU];

int encoder_pos = 0;                     
int encoder_turn_status_old = LOW;                 
int encoder_turn_status = LOW;                                           
int encoder_click_status = HIGH;
int encoder_click_status_old = HIGH; 

/*
 * Ku=250
 * Tu=42
 * Ti=22.4
 * Td=14
 * Ki=11.9
 * Kd=140
 * Kp=50
 */
double Kd = 40;         
double Kp = 25;         
double Ki = 0.5;       
double set_point_1, input_1, output_1;
PID temp_1(&input_1, &output_1, &set_point_1, Kp, Ki, Kd, DIRECT);
double set_point_2, input_2, output_2;
PID temp_2(&input_2, &output_2, &set_point_2, Kp, Ki, Kd, DIRECT);
double set_point_3, input_3, output_3;
PID temp_3(&input_3, &output_3, &set_point_3, Kp, Ki, Kd, DIRECT);
int window_size = 5000;
unsigned long window_start_time;

double abs_max = 0;
double abs_min = 100;
unsigned long abs_max_time_1 = 0;
unsigned long abs_min_time_1 = 0;
unsigned long abs_max_time_2 = 0;
unsigned long abs_min_time_2 = 0;
int init_tuning = 0;

bool safety_stop = false;

File database_file;
File log_file;

unsigned long previousRPMMillis;
unsigned long previousMillis;
float RPM_fan;
int state_fan = 0;
unsigned long interval = 3000;
volatile unsigned long pulses=0;
unsigned long lastRPMmillis = 0;

///////////////////////////////////
// FAN CONTROL
///////////////////////////////////
void setupFan() {
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4  = 0;

    // Mode 10: phase correct PWM with ICR4 as Top (= F_CPU/2/25000)
    // OC4C as Non-Inverted PWM output
    ICR4   = (F_CPU/25000)/2;
    OCR4A  = ICR4/2;                    // default: about 50:50
    TCCR4A = _BV(COM4A1) | _BV(WGM41);
    TCCR4B = _BV(WGM43) | _BV(CS40);

    pinMode(FAN_PIN_TAC, INPUT_PULLUP); // Set pin to read the Hall Effect Sensor
    attachInterrupt(digitalPinToInterrupt(FAN_PIN_TAC), countPulse, RISING); // Attach an interrupt to count

    // Set the PWM pin as output.
    pinMode( FAN_PIN_PWM, OUTPUT);
}

void countPulse() {
  pulses++;
  flag_fan = 1;
}

unsigned long calcRPM() {
  unsigned long RPM_fan;
  noInterrupts();
  float elapsedMS = (millis() - lastRPMmillis)/1000.0;
  unsigned long revolutions = pulses/2;
  float revPerMS = revolutions / elapsedMS;
  RPM_fan = revPerMS * 60.0;
  lastRPMmillis = millis();
  pulses=0;
  interrupts();

  return RPM_fan;
}

void writeFanRPM(int value) {
  OCR4A = value*320/10;
}

//////////////////////////////////

///////////////////////////////////
// MOTOR CONTROL
//////////////////////////////////
void setupMotorTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 1;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

int calculateSpeed(float set_rpm) {
  return (360.0/1.8*STEP_ACCURACY)*set_rpm/60.0*GEARBOX;
}

void setupMotorInit() {
  motor.setEnablePin(E_ENABLE_PIN);
  motor.setPinsInverted(false, false, true); //invert logic of enable pin
  motor.disableOutputs();
  motor.setMaxSpeed(MAX_STEP_SPEED);
  motor.setSpeed(calculateSpeed(rpm_value));  
}

ISR(TIMER1_COMPA_vect) {
  if (rpm_old != rpm_value) {
    motor.setSpeed(calculateSpeed(rpm_value));
    rpm_old = rpm_value;
  }
  motor.runSpeed();
}
//////////////////////////////////////////////

//////////////////////////////////////////////
// SD Card Module
//////////////////////////////////////////////
void setupSD() {
  pinMode(TUNE_PIN, INPUT_PULLUP);
  pinMode(SDCARDCS, OUTPUT);
  if(!SD.begin(SDCARDCS)) {
    Serial.println("SD begin error");
  }
  readConfig();
  
  log_file = SD.open("log.txt", FILE_WRITE);
  if (!log_file) {
    Serial.println("Could not open log file");
  }
  log_file.println("Begin logging...");
  
}

void readConfig() {
  database_file.close();
  database_file = SD.open("conf.txt", FILE_READ);
  if (!database_file) {
    Serial.println(F("Failed to create file 1"));
    return;
  }
  StaticJsonDocument<512> outdoc;
  deserializeJson(outdoc, database_file);
  Kp = outdoc["Kp"];
  Ki = outdoc["Ki"];
  Kd = outdoc["Kd"];
  Serial.println(Kp);
  Serial.println(Ki);
  Serial.println(Kd);
  database_file.close();
}

void saveConfig(double Kp, double Ki, double Kd) {
  SD.remove("conf.txt");
  database_file = SD.open("conf.txt", FILE_WRITE);
  if (!database_file) {
    Serial.println(F("Failed to create file 2"));
    return;
  }
  StaticJsonDocument<256> indoc;
  indoc["Kp"] = Kp;
  indoc["Kd"] = Kd;
  indoc["Ki"] = Ki;
  if (serializeJson(indoc, database_file) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  database_file.close();
}

void logSD(int temp_nozzle, int temp_pre, int temp_pre_2) {
  log_file.print("Nozzle temp: ");
  log_file.print(temp_nozzle);
  log_file.print(" / Preheat temp: ");
  log_file.print(temp_pre);
  log_file.print(" / Preheat temp 2: ");
  log_file.println(temp_pre_2);
  log_file.flush();
}
/////////////////////////////////////////////

//////////////////////////////////////////////
// LCD CONTROL
//////////////////////////////////////////////
void setupLCD() {
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  
  rpm.id = 0;
  rpm.value = 0;
  strcpy(rpm.label, "RPM SET:");
  temperature_current_nozzle.id = 1;
  temperature_current_nozzle.value = 0;
  strcpy(temperature_current_nozzle.label, "NOZZLE TEMP:");
  rpm_current.id = 2;
  rpm_current.value = rpm_value;
  strcpy(rpm_current.label, "RPM: ");
  temperature_current_preheat.id = 3;
  temperature_current_preheat.value = 0;
  strcpy(temperature_current_preheat.label, "PREHEAT TEMP:");
  temperature_nozzle.id = 4;
  temperature_nozzle.value = 25;
  strcpy(temperature_nozzle.label, "NOZZLE SET TEMP:");
  temperature_preheat.id = 5;
  temperature_preheat.value = 25;
  strcpy(temperature_preheat.label, "PRE SET TEMP:");
  fan_speed.id = 6;
  fan_speed.value = 0;
  strcpy(fan_speed.label, "FAN SPEED:");

  screen[3] = &rpm;
  screen[4] = &rpm_current;
  screen[5] = &temperature_current_nozzle;
  screen[6] = &temperature_current_preheat;
  screen[0] = &temperature_nozzle;
  screen[1] = &temperature_preheat;
  screen[2] = &fan_speed;
  
  for (int i = 3; i < MAX_MENU; i++) {
    lcd.setCursor(0, i-3);
     if (i == 3)
      lcd.print(">");
     lcd.print(screen[i]->label);
     lcd.setCursor(17, i-3);
     lcd.print(screen[i]->value);
  }
   menu = 0;
   menu_old = 0;
}

void initAutoTunePID() {
  lcd.setCursor(0,0);
  lcd.print("Tuning heat bands...");
  lcd.setCursor(0,1);
  lcd.print("  Setting to 100oC  ");
  lcd.setCursor(0,2);
  lcd.print("Kp=    Kd=    Ki=    ");
}

void right() {
  if (menu_level == 0) {
    if (menu < (SETTING_MENU-1))
      menu++;
  }
  else if (screen[menu]->value < 425){
    screen[menu]->value++;
  }  
  updateScreen();
}

void left() {
  if (menu_level == 0) {
    if (menu > 0)
      menu--;
  }
  else {
    //if (screen[menu]->value > 0)
    screen[menu]->value--;
  } 
  updateScreen();
}

void click() {
  if (menu_level == 0) {
    menu_level++;
  }
  else {
    updateValue();
    menu_level--;
  }
  updateScreen();
}
void rightTune() {
  if (init_tuning == 0 || init_tuning == 3) {
    Kp++;
    temp_1.SetTunings(Kp,0,0);
    temp_2.SetTunings(Kp,0,0);
    lcd.setCursor(3,0);
    lcd.print(Kp);
  }
  if (init_tuning == 1 || init_tuning == 4) {
    Ki+=0.01;
    temp_1.SetTunings(Kp,Ki,0);
    temp_2.SetTunings(Kp,Ki,0);
    lcd.setCursor(13,0);
    lcd.print(Ki);
  }
  if (init_tuning == 2 || init_tuning == 5) {
    Kd++;
    temp_1.SetTunings(Kp,Ki,Kd);
    temp_2.SetTunings(Kp,Ki,Kd);
    lcd.setCursor(3,1);
    lcd.print(Kd);
  }
}

void leftTune() {
  if (init_tuning == 0 || init_tuning == 3) {
    Kp--;
    temp_1.SetTunings(Kp,0,0);
    temp_2.SetTunings(Kp,0,0);
    lcd.setCursor(3,0);
    lcd.print(Kp);
  }
  if (init_tuning == 1 || init_tuning == 4) {
    Ki-=0.01;
    temp_1.SetTunings(Kp,Ki,0);
    temp_2.SetTunings(Kp,Ki,0);
    lcd.setCursor(13,0);
    lcd.print(Ki);
  }
  if (init_tuning == 2 || init_tuning == 5) {
    Kd--;
    temp_1.SetTunings(Kp,Ki,Kd);
    temp_2.SetTunings(Kp,Ki,Kd);
    lcd.setCursor(3,1);
    lcd.print(Kd);
  }
}

void clickTune() {
    init_tuning++;
    if (init_tuning == 0 || init_tuning == 3) {
      lcd.setCursor(3,0);
      lcd.print(Kp);
    }
    if (init_tuning == 1 || init_tuning == 4) {
      lcd.setCursor(13,0);
      lcd.print(Ki);
    }
    if (init_tuning == 2 || init_tuning == 5) {
      lcd.setCursor(3,1);
      lcd.print(Kd);
    }
}

void updateValue() {
  if (menu == 3) {
    if (rpm.value == 0) 
      motor.disableOutputs();
    else
      motor.enableOutputs();
    rpm_value = rpm.value;
    rpm_current.value = rpm_value;
    lcd.setCursor(17, 1);
    lcd.print("   ");
    lcd.setCursor(17, 1);
    lcd.print(screen[menu]->value);
  }
  else if (menu == 0) {
    set_point_1 = temperature_nozzle.value;
  }
  else if (menu == 1) {
    set_point_2 = temperature_preheat.value;
    set_point_3 = temperature_preheat.value;
  }
  else if (menu == 2) {
    state_fan = fan_speed.value;
    writeFanRPM(state_fan);
  }
}

void updateScreen() {
  if (menu_level_old == 0) {
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 0);
    lcd.print(screen[menu]->label);
    lcd.setCursor(17, 0);
    lcd.print(screen[menu]->value);
  }
  else {
    lcd.setCursor(16, 0);
    lcd.print("    ");
    lcd.setCursor(17, 0);
    lcd.print(screen[menu]->value);
  }
  
  if (menu_level != 0) {
    lcd.setCursor(16, 0);
    lcd.print("    ");
    lcd.setCursor(16, 0);
    lcd.print(">");
    lcd.print(screen[menu]->value);
  }
  else {
    lcd.setCursor(0, 0);
    lcd.print(">");
    lcd.print(screen[menu]->label);
    lcd.setCursor(17, 0);
    lcd.print(screen[menu]->value);
  }
  menu_old = menu;
  menu_level_old = menu_level;
}

void updateTemperature() {
  lcd.setCursor(17, 2);
  lcd.print("   ");
  lcd.setCursor(17, 2);
  if (input_1 <= 0) {
    lcd.print(0);
  }
  else if (input_1 < 1000)
    lcd.print((int)input_1);
  else
    lcd.print(999);
  
  lcd.setCursor(13, 3);
  lcd.print("   ");
  lcd.setCursor(13, 3);
  if (input_2 <= 0) {
    lcd.print(0);
  }
  else if (input_2 < 1000)
    lcd.print((int)input_2);
  else
    lcd.print(999);

  lcd.setCursor(16, 3);
  lcd.print("/");
  lcd.setCursor(17, 3);
  if (input_3 <= 0) {
    lcd.print(0);
  }
  else if (input_3 < 1000)
    lcd.print((int)input_3);
  else
    lcd.print(999);
}

void updateTemperatureTune(int id, int temp_1) {
  lcd.setCursor(15, 3);
  lcd.print(id);
  lcd.print(": ");
  lcd.setCursor(17, 3);
  if (temp_1 < 10) {
    lcd.print("  ");
    lcd.print((int)temp_1);
  }
  else if (temp_1 < 100) {
    lcd.print(" ");
    lcd.print((int)temp_1);
  }
  else if (temp_1 < 1000)
    lcd.print((int)temp_1);
  else
    lcd.print(999);
}

void initSafety() {
  lcd.clear();
  lcd.print("  THERMAL MAXIMUM!  ");
  lcd.setCursor(0,1);
  lcd.print("SHUTTING HEATER OFF!");
  lcd.setCursor(0,2);
  lcd.print("UNTIL:    <50");
  lcd.setCursor(0,3);
  lcd.print("UNTIL:    <50");
}

void updateSafety() {
  lcd.setCursor(7,2);
  lcd.print(input_1);
  lcd.setCursor(7,3);
  lcd.print(input_2);
}

void initTunePID() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Kp=");
  lcd.setCursor(3,0);
  lcd.print(Kp);
  lcd.setCursor(10,0);
  lcd.print("Ki=");
  lcd.setCursor(0,1);
  lcd.print("Kd=");
  lcd.setCursor(0,2);
  lcd.print("NOZZLE TEMP: ");
  lcd.setCursor(0,3);
  lcd.print("PREHEAT TEMP: ");
}
//////////////////////////////////////////////

//////////////////////////////////////////////
// ENCODER CONTROL
//////////////////////////////////////////////
void setupEncoder() {
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);

  pinMode(ENCODER_CLICK_PIN, INPUT_PULLUP);

  noInterrupts();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 100;
  TCCR2B |= (1 << WGM12);
  TCCR2B |= (1 << CS12) | (1 << CS10);
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

ISR(TIMER2_COMPA_vect) {
  encoder_turn_status = digitalRead(ENCODER_LEFT_PIN);
  if ((encoder_turn_status_old == LOW) && (encoder_turn_status == HIGH)) {
    if (digitalRead(ENCODER_RIGHT_PIN) == LOW) {
      flag_left = 1;
    } else {
      flag_right = 1;
    }
  }
  encoder_turn_status_old = encoder_turn_status;

  encoder_click_status = digitalRead(ENCODER_CLICK_PIN);
  if ((encoder_click_status == LOW) && (encoder_click_status_old == HIGH)) 
    flag_click = 1;
  encoder_click_status_old = encoder_click_status;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
// TEMPERATURE CONTROL
//////////////////////////////////////////////
void setupTemp() {
  pinMode(TEMP_OUTPUT_PIN_1, OUTPUT);
  pinMode(TEMP_OUTPUT_PIN_2, OUTPUT);
  pinMode(TEMP_OUTPUT_PIN_3, OUTPUT);

  pinMode(TEMP_INPUT_PIN_1, INPUT);
  pinMode(TEMP_INPUT_PIN_2, INPUT);
  pinMode(TEMP_INPUT_PIN_3, INPUT);

  pinMode(TUNE_PIN, INPUT_PULLUP);
  pinMode(AUTOTUNE_PIN, INPUT_PULLUP);

  window_start_time = millis();
  set_point_1 = 25;
  set_point_2 = 25;
  set_point_3 = 25;

  temp_1.SetOutputLimits(0, window_size);
  temp_2.SetOutputLimits(0, window_size);
  temp_3.SetOutputLimits(0, window_size);

  temp_1.SetMode(AUTOMATIC);
  temp_2.SetMode(AUTOMATIC);
  temp_3.SetMode(AUTOMATIC);

  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 14;
  TCCR3B |= (1 << WGM12);
  TCCR3B |= (1 << CS12);
  TIMSK3 |= (1 << OCIE3A);
  interrupts();

  if(digitalRead(AUTOTUNE_PIN) == LOW) {
    autoTune(TEMP_INPUT_PIN_1, TEMP_OUTPUT_PIN_1, THRESHHOLD_LOW, THRESHHOLD_HIGH, 1);
  }
  if(digitalRead(TUNE_PIN) == LOW) {
    tunePID();
  }
  else {
    temp_1.SetTunings(Kp,Ki,Kd);
    temp_2.SetTunings(Kp,Ki,Kd);
    temp_3.SetTunings(Kp,Ki,Kd);
    Serial.println(Kp);
    Serial.println(Ki);
    Serial.println(Kd);
  }
}

ISR(TIMER3_COMPA_vect) {
  flag_temp = 1;
  temp_update++;
  buzzer_update++;
}

bool tunePID() {
  set_point_1 = 100;
  set_point_2 = 50;
  set_point_3 = 50;
  init_tuning = 0;
  initTunePID();
  while(init_tuning < 3) {
    if (flag_left == 1) {
      leftTune();
      flag_left = 0;
    }
    if (flag_right == 1) {
      rightTune();
      flag_right = 0;
    }
    if (flag_click == 1) {
      clickTune();
      flag_click = 0;
    }
  
    if (flag_temp == 1) {
      input_1 = analogRead(TEMP_INPUT_PIN_1);
      input_1 = ((input_1*5.0/1024.0)-1.25)/0.005;
      input_2 = analogRead(TEMP_INPUT_PIN_2);
      input_2 = ((input_2*5.0/1024.0)-1.25)/0.005;
      input_3 = analogRead(TEMP_INPUT_PIN_3);
      input_3 = ((input_3*5.0/1024.0)-1.25)/0.005;
      temp_1.Compute();
      temp_2.Compute();
      temp_3.Compute();
    
      unsigned long now = millis();
      if (now - window_start_time > window_size) {
        window_start_time += window_size;
      }   
  
      ////////////////AUTOMATIC SAFETY/////////////////////////
      if (input_1 > 450 || input_2 > 450 || input_3 > 450 || safety_stop || input_1 < 0 || input_2 < 0 || input_3 < 0) {
        output_1 = 0;
        output_2 = 0;
        if (!safety_stop) {
          digitalWrite(BUZZER, HIGH);
          initSafety();
          safety_stop = true;
        }
      }
      if (input_1 < 50 && input_2 < 50 && input_3 < 50 && safety_stop && input_1 > 0 && input_2 > 0 && input_3 > 0){
        digitalWrite(BUZZER, LOW);
        setupLCD();
        safety_stop = false;
      }
      ///////////////////////////////////////////////////////////
      if (output_1 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_1, HIGH);
      else digitalWrite(TEMP_OUTPUT_PIN_1, LOW);
      if (output_2 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_2, HIGH);
      else digitalWrite(TEMP_OUTPUT_PIN_2, LOW);
      if (output_3 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_3, HIGH);
      else digitalWrite(TEMP_OUTPUT_PIN_3, LOW);
        
      if (temp_update >= 2500 && !safety_stop) {
        updateTemperature();
        Serial.print(input_1);
        Serial.print(" ");
        Serial.print(input_2);
        Serial.print(" ");
        Serial.println(input_3);
        temp_update = 0;
      }
  
      if (buzzer_update >= 2500 && !safety_stop) {
        if (input_1 > set_point_1 + 20 || input_2 > set_point_2 + 20 || input_3 > set_point_3 + 20) {
          digitalWrite(BUZZER, !digitalRead(BUZZER));
        }
        else {
          digitalWrite(BUZZER, LOW);
        }
        buzzer_update = 0;
      }
          
      else if(temp_update >= 1000 && safety_stop) {
        updateSafety();
        temp_update = 0;
      }
      
      flag_temp = 0;
    }
  }
  saveConfig(Kp, Ki, Kd);
  set_point_1 = 25;
  set_point_2 = 25;
  set_point_3 = 25;
}

void autoTune(int input_pin, int output_pin, int thresh_low, int thresh_high, int id) {
  initAutoTunePID();
  while (1) {
    if (flag_temp == 1) {
      if (temp_update >= 2500) {
        updateTemperatureTune(id, input_1);
        temp_update = 0;
      }
      input_1 = analogRead(input_pin);
      input_1 = ((input_1*5.0/1024.0)-1.25)/0.005;
  
      if (input_1 > thresh_high) {
        digitalWrite(output_pin, LOW);
        if (init_tuning == 0)
          init_tuning = 1;
        else if (init_tuning == 2)
          init_tuning = 3;
        else if (init_tuning == 4)
          init_tuning = 5;
      }
      else if (input_1 < thresh_low) {
        digitalWrite(output_pin, HIGH);
        if (init_tuning == 1)
          init_tuning = 2;
        else if (init_tuning == 3)
          init_tuning = 4;
        else if (init_tuning == 5) {
          digitalWrite(output_pin, LOW);
          break;
        }
      }
  
      if (input_1 >= abs_max && init_tuning == 3) {
        abs_max = input_1;
        abs_max_time_1 = millis(); 
      }
      
      if (input_1 < abs_min && init_tuning == 2) {
        abs_min = input_1;
        abs_min_time_1 = millis();
      }
  
      if (input_1 >= abs_max && init_tuning == 5) {
        abs_max = input_1;
        abs_max_time_2 = millis(); 
      }
      
      if (input_1 < abs_min && init_tuning == 4) {
        abs_min = input_1;
        abs_min_time_2 = millis();
      }
      flag_temp = 0;
    }
  }
  double D, A, Pu, Ku;
  D = 120/2;
  A = abs_max - abs_min;
  Pu = abs_max_time_2 - abs_max_time_1;
  Ku = 4*D/(3.14159*A);
  Kp = 50*0.6*Ku;
  Ki = 100*1.2*Ku/Pu;
  Kd = 0.01*0.075*Ku*Pu;
  lcd.setCursor(3,2);
  if (Kp < 10) {
    lcd.print("  "); 
    lcd.print((int)Kp);
  }
  else if (Kp < 100) {
    lcd.print(" ");
    lcd.print((int)Kp);
  }
  else if (Kp < 1000)
    lcd.print((int)Kp);
  else
    lcd.print((int)Kp);
  
  lcd.setCursor(10,2);
  if (Kd < 10) {
    lcd.print("  ");
    lcd.print((int)Kd);
  }
  else if (Kd < 100) {
    lcd.print(" ");
    lcd.print((int)Kd);
  }
  else if (Kd < 1000)
    lcd.print((int)Kd);
  else
    lcd.print((int)Kd);
    
  lcd.setCursor(17,2);
  if (Ki < 10) {
    lcd.print("  ");
    lcd.print((int)Ki);
  }
  else if (Ki < 100) {
    lcd.print(" ");
    lcd.print((int)Ki);
  }
  else if (Ki < 1000)
    lcd.print((int)Ki);
  else
    lcd.print((int)Ki);
  delay(30*1000);
  temp_1.SetTunings(Kp, Ki, Kd);
  temp_2.SetTunings(Kp, Ki, Kd);
  temp_3.SetTunings(Kp, Ki, Kd);
  saveConfig(Kp, Ki, Kd);
}
//////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  setupSD();
  setupMotorInit();
  setupFan();
  setupLCD();
  setupEncoder();
  setupTemp();
  setupLCD();
  setupMotorTimer();
}

void loop() {
  //To fill
  if (flag_left == 1) {
    left();
    flag_left = 0;
  }
  if (flag_right == 1) {
    right();
    flag_right = 0;
  }
  if (flag_click == 1) {
    click();
    flag_click = 0;
  }

  if (flag_temp == 1) {
    input_1 = analogRead(TEMP_INPUT_PIN_1);
    input_1 = 0; //((input_1*5.0/1024.0)-1.25)/0.005;
    input_2 = analogRead(TEMP_INPUT_PIN_2);
    input_2 = 0; //((input_2*5.0/1024.0)-1.25)/0.005;
    input_3 = analogRead(TEMP_INPUT_PIN_3);
    input_3 = 0; //((input_3*5.0/1024.0)-1.25)/0.005;
    temp_1.Compute();
    temp_2.Compute();
    temp_3.Compute();
  
    unsigned long now = millis();
    if (now - window_start_time > window_size) {
      window_start_time += window_size;
    }   

    ////////////////AUTOMATIC SAFETY/////////////////////////
    if (input_1 > 450 || input_2 > 450 || input_3 > 450 || safety_stop || input_1 < 0 || input_2 < 0 || input_3 < 0) {
      output_1 = 0;
      output_2 = 0;
      output_3 = 0;
      if (!safety_stop) {
        digitalWrite(BUZZER, HIGH);
        initSafety();
        safety_stop = true;
      }
    }
    if (input_1 < 50 && input_2 < 50 && input_3 < 50 && safety_stop && input_1 > 0 && input_2 > 0 && input_3 > 0) {
      digitalWrite(BUZZER, LOW);
      setupLCD();
      safety_stop = false;
    }
    ///////////////////////////////////////////////////////////
    if (output_1 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_1, HIGH);
    else digitalWrite(TEMP_OUTPUT_PIN_1, LOW);
    if (output_2 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_2, HIGH);
    else digitalWrite(TEMP_OUTPUT_PIN_2, LOW);
    if (output_3 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_3, HIGH);
    else digitalWrite(TEMP_OUTPUT_PIN_3, LOW);

    if (temp_update >= 2500 && !safety_stop) {
      updateTemperature();
      Serial.print(input_1);
      Serial.print(" ");
      Serial.print(input_2);
      Serial.print(" ");
      Serial.println(input_3);
      logSD(input_1, input_2, input_3);
      temp_update = 0;

      if (input_1 > set_point_1 + 20 || input_2 > set_point_2 + 20 || input_3 > set_point_3 + 20) {
        digitalWrite(BUZZER, LOW); //!digitalRead(BUZZER));
      }
      else {
        digitalWrite(BUZZER, LOW);
      }
      buzzer_update = 0;
    }
        
    else if(temp_update >= 2500 && safety_stop) {
      Serial.print(input_1);
      Serial.print(" ");
      Serial.print(input_2);
      Serial.print(" ");
      Serial.println(input_3);
      updateSafety();
      temp_update = 0;
    }
    
    flag_temp = 0;
  }

  if (millis() - previousMillis > interval) {
    Serial.print("RPM=");
    Serial.print(calcRPM());
    Serial.print(F(" @ PWM="));
    Serial.println(state_fan);
    previousMillis = millis(); 
  }
}
