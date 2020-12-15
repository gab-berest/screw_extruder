#include "motor_control.h"

using namespace std;

Motor::Motor() {
  
}

Motor::~Motor() {
  
}

int Motor::init(float step_accuracy = 1, int speed = 0, int direction = 0) {
  //Disable interrupts
  noInterrupts();
  //Enable timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 14;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE4A);
  interrupts();
  
  pinMode(E_STEP_PIN, OUTPUT);
  pinMode(E_DIR_PIN, OUTPUT);
  pinMode(E_ENABLE_PIN, OUTPUT);

  digitalWrite(E_ENABLE_PIN, LOW);

  this->setAccuracy(step_accuracy);
  this->setSpeed(speed);
  this->setDirection(direction);
}

int Motor::setDirection(int direction) {
  this->_direction = direction;
  if (this->_direction == 0)
    digitalWrite(E_DIR_PIN, HIGH);
  else
    digitalWrite(E_DIR_PIN, LOW);
  return 0;
  return 0;
}

int Motor::setSpeed(int speed) {
  this->_speed = speed;
  return 0;
}

int Motor::setAccuracy(float accuracy) {
  this->_step_accuracy = accuracy;
  return 0;
}

int Motor::getDirection() {
  return this->_direction;
}

int Motor::getSpeed() {
  return this->_speed;
}

int Motor::getAccuracy() {
  return this->_step_accuracy;
}

int Motor::getStepping() {
  return this->_stepping;
}

int Motor::step() {
  if (!(this->_stepping)) {
    digitalWrite(E_STEP_PIN, HIGH);
    this->_stepping = 1;
  }
  else {
    digitalWrite(E_STEP_PIN, LOW);
    this->_stepping = 0;
  }
  return 0;
}
