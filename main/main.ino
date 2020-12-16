#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <LiquidMenu.h>

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define STEP_ACCURACY      1
#define MAX_STEP_SPEED     10000

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
#define BUZZER             -1

#define LED_PIN            13

struct Menu {
  int id = 0;
  int value = 0;
  char label[16];
};

int flag_left = 0;
int flag_right = 0;
int flag_click = 0;

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
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

int calculateSpeed(float set_rpm) {
  return (360.0/1.8*STEP_ACCURACY)*set_rpm/60.0;
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
// LCD CONTROL
//////////////////////////////////////////////
void setupLCD() {
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  
  rpm.id = 0;
  rpm.value = 0;
  strcpy(rpm.label, "RPM: ");
  temperature.id = 1;
  temperature.value = 0;
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

void right() {
  if (menu_level == 0) {
    if (menu < (SETTING_MENU-1))
      menu++;
  }
  else {
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

void updateValue() {
  if (menu == 0) {
    rpm_value = rpm.value;
    rpm_current.value = rpm_value;
    lcd.setCursor(16, 2);
    lcd.print("    ");
    lcd.setCursor(17, 2);
    lcd.print(screen[2]->value);
  }
  //else if (menu == 1)
    //temperature_value = temp.value;
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

//////////////////////////////////////////////

void setup() {
  setupMotorInit();
  setupMotorTimer();
  setupEncoder();
  setupLCD();
  Serial.begin(9600);
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
}
