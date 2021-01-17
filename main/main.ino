#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

#define TUNE_PIN           0

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define STEP_ACCURACY      1
#define GEARBOX            15
#define MAX_STEP_SPEED     1000

#define LCD_RS             16
#define LCD_EN             17
#define LCD_PIN_1          23
#define LCD_PIN_2          25
#define LCD_PIN_3          27
#define LCD_PIN_4          29
#define LCD_WIDTH          20
#define LCD_HEIGHT         4

#define MAX_MENU           4
#define SETTING_MENU       2

#define ENCODER_LEFT_PIN   33
#define ENCODER_RIGHT_PIN  31    
#define ENCODER_CLICK_PIN  35
#define BUTTON_DEBOUNCE    10
#define BUZZER             37

#define SDCARDDETECT       49
#define SDCARDCS           53
#define SDCARDMOSI         49
#define SDCARDMISO         50
#define SDCARDSCK          52

#define LED_PIN            13

#define TEMP_INPUT_PIN_1   3
#define TEMP_INPUT_PIN_2   4
#define TEMP_OUTPUT_PIN_1  8
#define TEMP_OUTPUT_PIN_2  9

#define THRESHHOLD_HIGH    102
#define THRESHHOLD_LOW     98

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

AccelStepper motor = AccelStepper(AccelStepper::DRIVER, E_STEP_PIN, E_DIR_PIN);
int rpm_old = 0;
int rpm_value = 0;

LiquidCrystal lcd(LCD_RS,LCD_EN,LCD_PIN_1,LCD_PIN_2,LCD_PIN_3,LCD_PIN_4);
int menu = 0;
int menu_level = 0;
int menu_old = 0;
int menu_level_old = 0;
Menu rpm;
Menu temperature;
Menu rpm_current;
Menu temperature_current;
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
double Kd = 50;         
double Kp = 25;         
double Ki = 0.05;       
double set_point_1, input_1, output_1;
PID temp_1(&input_1, &output_1, &set_point_1, Kp, Ki, Kd, DIRECT);
double set_point_2, input_2, output_2;
PID temp_2(&input_2, &output_2, &set_point_2, Kp, Ki, Kd, DIRECT);
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
  motor.enableOutputs();
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
  
  SD.begin(SDCARDCS);
  if (digitalRead(TUNE_PIN) == HIGH) {
    readConfig();
  }
  
  log_file = SD.open("log.txt", FILE_WRITE);
  log_file.println("Begin logging...");
  
}

void readConfig() {
  database_file = SD.open("database.txt", FILE_READ);
  if (!database_file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  StaticJsonDocument<512> outdoc;
  deserializeJson(outdoc, database_file);
  Kp = outdoc["Kp"];
  Ki = outdoc["Ki"];
  Kd = outdoc["Kd"];
  database_file.close();
}

void saveConfig(double Kp, double Ki, double Kd) {
  SD.remove("database.txt");
  database_file = SD.open("database.txt", FILE_WRITE);
  if (!database_file) {
    Serial.println(F("Failed to create file"));
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

void logSD(int temp_nozzle, int temp_pre) {
  log_file.print("Nozzle temp: ");
  log_file.print(temp_nozzle);
  log_file.print(" / Preheat temp: ");
  log_file.println(temp_pre);
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
  strcpy(rpm.label, "RPM: ");
  temperature.id = 1;
  temperature.value = 25;
  strcpy(temperature.label, "TEMP: ");
  rpm_current.id = 2;
  rpm_current.value = rpm_value;
  strcpy(rpm_current.label, "RPM CURRENT: ");
  temperature_current.id = 1;
  temperature_current.value = 0;
  strcpy(temperature_current.label, "TEMP CURRENT: ");

  screen[0] = &rpm;
  screen[1] = &temperature;
  screen[2] = &rpm_current;
  screen[3] = &temperature_current;
  
  for (int i = 0; i < MAX_MENU; i++) {
    lcd.setCursor(0, i);
     if (i == 0)
      lcd.print(">");
     lcd.print(screen[i]->label);
     lcd.setCursor(17, i);
     lcd.print(screen[i]->value);
  }

   menu = 0;
   menu_old = 0;
}

void setupLCDTune() {
  setupLCD();
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
  else {// if (screen[menu] < 425){
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
    if (screen[menu]->value > 0)
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
    lcd.setCursor(13,0);
    lcd.print(Kp);
  }
  if (init_tuning == 1 || init_tuning == 4) {
    Ki+=0.01;
    temp_2.SetTunings(Kp,Ki,0);
    lcd.setCursor(13,1);
    lcd.print(Ki);
  }
  if (init_tuning == 2 || init_tuning == 5) {
    Kd++;
    temp_2.SetTunings(Kp,Ki,Kd);
    lcd.setCursor(13,2);
    lcd.print(Kd);
  }
}

void leftTune() {
  if (init_tuning == 0 || init_tuning == 3) {
    Kp--;
    temp_1.SetTunings(Kp,0,0);
    lcd.setCursor(13,0);
    lcd.print(Kp);
  }
  if (init_tuning == 1 || init_tuning == 4) {
    Ki-=0.01;
    temp_2.SetTunings(Kp,Ki,0);
    lcd.setCursor(13,1);
    lcd.print(Ki);
  }
  if (init_tuning == 2 || init_tuning == 5) {
    Kd--;
    temp_2.SetTunings(Kp,Ki,Kd);
    lcd.setCursor(13,2);
    lcd.print(Kd);
  }
}

void clickTune() {
    init_tuning++;
}

void updateValue() {
  if (menu == 0) {
    rpm_value = rpm.value;
    rpm_current.value = rpm_value;
    lcd.setCursor(16, 2);
    lcd.print("    ");
    lcd.setCursor(17, 2);
    lcd.print(screen[2]->value);
  }
  else if (menu == 1) {
    set_point_1 = temperature.value;
    set_point_2 = set_point_1/2;
  }
}

void updateScreen() {
  if (menu_level_old == 0) {
    lcd.setCursor(0, menu_old);
    lcd.print("                ");
    lcd.setCursor(0, menu_old);
    lcd.print(screen[menu_old]->label);
  }
  else {
    lcd.setCursor(16, menu_old);
    lcd.print("    ");
    lcd.setCursor(17, menu_old);
    lcd.print(screen[menu]->value);
  }
  
  if (menu_level != 0) {
    lcd.setCursor(16, menu);
    lcd.print("    ");
    lcd.setCursor(16, menu);
    lcd.print(">");
    lcd.print(screen[menu]->value);
  }
  else {
    lcd.setCursor(0, menu);
    lcd.print(">");
    lcd.print(screen[menu]->label);
  }
  menu_old = menu;
  menu_level_old = menu_level;
}

void updateTemperature() {
  lcd.setCursor(13, 3);
  lcd.print("   /");
  lcd.setCursor(13, 3);
  if (input_1 < 10) {
    lcd.print("  ");
    lcd.print((int)input_1);
  }
  else if (input_1 < 100) {
    lcd.print(" ");
    lcd.print((int)input_1);
  }
  else if (input_1 < 1000)
    lcd.print((int)input_1);
  else
    lcd.print(999);
  
  lcd.setCursor(17, 3);
  lcd.print("   ");
  lcd.setCursor(17, 3);
  if (input_2 < 10) {
    lcd.print("  ");
    lcd.print((int)input_2);
  }
  else if (input_2 < 100) {
    lcd.print(" ");
    lcd.print((int)input_2);
  }
  else if (input_2 < 1000)
    lcd.print((int)input_2);
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

  pinMode(TEMP_INPUT_PIN_1, INPUT);
  pinMode(TEMP_INPUT_PIN_2, INPUT);

  window_start_time = millis();
  set_point_1 = 25;
  set_point_2 = 25;

  temp_1.SetOutputLimits(0, window_size);
  temp_2.SetOutputLimits(0, window_size);

  temp_1.SetMode(AUTOMATIC);
  temp_2.SetMode(AUTOMATIC);

  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 14;
  TCCR3B |= (1 << WGM12);
  TCCR3B |= (1 << CS12);
  TIMSK3 |= (1 << OCIE3A);
  interrupts();

  if(digitalRead(TUNE_PIN) == LOW) {
    /*double D, A, Pu, Ku;
    while(!autoTune(TEMP_INPUT_PIN_1, TEMP_OUTPUT_PIN_1, THRESHHOLD_LOW, THRESHHOLD_HIGH, 1));
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
    delay(30000);
    temp_1.SetTunings(Kp, Ki, Kd);
    lcd.setCursor(0,2);
    lcd.print("Kp=    Kd=    Ki=    ");
  
    abs_max = 100;
    abs_min = 100;
    abs_max_time_1 = 0;
    abs_min_time_1 = 0;
    abs_max_time_2 = 0;
    abs_min_time_2 = 0;
    init_tuning = 0;
  
    while(!autoTune(TEMP_INPUT_PIN_2, TEMP_OUTPUT_PIN_2, THRESHHOLD_LOW, THRESHHOLD_HIGH, 2));
    D = 120/2;
    A = abs_max - abs_min;
    Pu = abs_min_time_2 - abs_min_time_1;
    Ku = 4*D/(3.14159*A);
    Kp = 50*0.6*Ku;
    Ki = 100*1.2*Ku/Pu; //284
    Kd = 0.00025*0.075*Ku*Pu;  //0
      lcd.setCursor(4,2);
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
    
    lcd.setCursor(11,2);
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
      
    lcd.setCursor(18,2);
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
    delay(30000);
    temp_2.SetTunings(Kp, Ki, Kd);*/
    tunePID();
  }
  else {
    temp_1.SetTunings(Kp,Ki,Kd);
    temp_2.SetTunings(Kp,Ki,Kd);
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
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Kp=");
  lcd.setCursor(0,1);
  lcd.print("Ki=");
  lcd.setCursor(0,2);
  lcd.print("Kd=");
  lcd.setCursor(0,3);
  lcd.print("CURRENT TEMPERATURE: ");
  set_point_1 = 100;
  Kp = 0;
  Ki = 0;
  Kd = 0;
  init_tuning = 0;
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
      temp_1.Compute();
    
      unsigned long now = millis();
      if (now - window_start_time > window_size) {
        window_start_time += window_size;
      }   
  
      ////////////////AUTOMATIC SAFETY/////////////////////////
      if (input_1 > 450 || input_2 > 450 || safety_stop) {
        output_1 = 0;
        output_2 = 0;
        if (!safety_stop) {
          digitalWrite(BUZZER, HIGH);
          initSafety();
          safety_stop = true;
        }
      }
      if (input_1 < 50 && input_2 < 50 && safety_stop) {
        digitalWrite(BUZZER, LOW);
        setupLCD();
        safety_stop = false;
      }
      ///////////////////////////////////////////////////////////
      if (output_1 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_1, HIGH);
      else digitalWrite(TEMP_OUTPUT_PIN_1, LOW);
  
      if (temp_update >= 2500 && !safety_stop) {
        updateTemperature();
        Serial.print(input_1);
        Serial.print(" ");
        Serial.println(input_2);
        temp_update = 0;
      }
  
      if (buzzer_update >= 2500 && !safety_stop) {
        if (input_1 > set_point_1 + 20) {
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

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Kp=");
  lcd.setCursor(0,1);
  lcd.print("Ki=");
  lcd.setCursor(0,2);
  lcd.print("Kd=");
  lcd.setCursor(0,3);
  lcd.print("CURRENT TEMPERATURE: ");
  set_point_2 = 100;
  Kp = 0;
  Ki = 0;
  Kd = 0;
  while(init_tuning < 6) {
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
      input_2 = analogRead(TEMP_INPUT_PIN_2);
      input_2 = ((input_1*5.0/1024.0)-1.25)/0.005;
      temp_2.Compute();
    
      unsigned long now = millis();
      if (now - window_start_time > window_size) {
        window_start_time += window_size;
      }   
  
      ////////////////AUTOMATIC SAFETY/////////////////////////
      if (input_1 > 450 || input_2 > 450 || safety_stop) {
        output_1 = 0;
        output_2 = 0;
        if (!safety_stop) {
          digitalWrite(BUZZER, HIGH);
          initSafety();
          safety_stop = true;
        }
      }
      if (input_1 < 50 && input_2 < 50 && safety_stop) {
        digitalWrite(BUZZER, LOW);
        setupLCD();
        safety_stop = false;
      }
      ///////////////////////////////////////////////////////////
      if (output_2 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_2, HIGH);
      else digitalWrite(TEMP_OUTPUT_PIN_2, LOW);
  
      if (temp_update >= 2500 && !safety_stop) {
        updateTemperature();
        Serial.print(input_1);
        Serial.print(" ");
        Serial.println(input_2);
        temp_update = 0;
      }
  
      if (buzzer_update >= 2500 && !safety_stop) {
        if (input_2 > set_point_2 + 20) {
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
}

bool autoTune(int input_pin, int output_pin, int thresh_low, int thresh_high, int id) {
  
  Serial.print("LOOP ");
  Serial.print(flag_temp);
  Serial.print("\n");
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
        return true;
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
  return false;
}
//////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  setupSD();
  setupLCDTune();
  setupEncoder();
  setupTemp();
  setupLCD();
  setupMotorInit();
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
    input_1 = ((input_1*5.0/1024.0)-1.25)/0.005;
    input_2 = analogRead(TEMP_INPUT_PIN_2);
    input_2 = ((input_2*5.0/1024.0)-1.25)/0.005;
    temp_1.Compute();
    temp_2.Compute();
  
    unsigned long now = millis();
    if (now - window_start_time > window_size) {
      window_start_time += window_size;
    }   

    ////////////////AUTOMATIC SAFETY/////////////////////////
    if (input_1 > 450 || input_2 > 450 || safety_stop) {
      output_1 = 0;
      output_2 = 0;
      if (!safety_stop) {
        digitalWrite(BUZZER, HIGH);
        initSafety();
        safety_stop = true;
      }
    }
    if (input_1 < 50 && input_2 < 50 && safety_stop) {
      digitalWrite(BUZZER, LOW);
      setupLCD();
      safety_stop = false;
    }
    ///////////////////////////////////////////////////////////
    if (output_1 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_1, HIGH);
    else digitalWrite(TEMP_OUTPUT_PIN_1, LOW);
    if (output_2 > now - window_start_time) digitalWrite(TEMP_OUTPUT_PIN_2, HIGH);
    else digitalWrite(TEMP_OUTPUT_PIN_2, LOW);

    if (temp_update >= 2500 && !safety_stop) {
      updateTemperature();
      Serial.print(input_1);
      Serial.print(" ");
      Serial.println(input_2);
      logSD(input_1, input_2);
      temp_update = 0;
    }

    if (buzzer_update >= 2500 && !safety_stop) {
      if (input_1 > set_point_1 + 20) {
        digitalWrite(BUZZER, LOW); //digitalWrite(BUZZER, !digitalRead(BUZZER));
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
