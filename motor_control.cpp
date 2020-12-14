#include "motor_contro.h"

using namespace std;

Motor::Motor(float step_accuracy = 1, int speed = 0, int direction = 0) {
  pinMode(E_STEP_PIN, OUTPUT);
  pinMode(E_DIR_PIN, OUTPUT);
  pinMode(E_ENABLE_PIN, OUTPUT);

  digitalWrite(E_ENABLE_PIN, LOW);
  
  this->setAccuracy(step_accuracy);
  this->setSpeed(speed);
  this->setDirection(direction);
}

Motor::~Motor() {
  
}

int Motor::setDirection(int direction) {
  this->_direction = direction;
  if (this->_direction == 0)
    digitalWrite(E_DIR_PIN, HIGH);
  else
    digitalWrite(E_DIR_PIN, LOW);
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

int motor::getAccuracy() {
  return this->_step_accuracy;
}

int Motor::step() {
  if (this->_stepping) {
    digitalWrite(E_STEP_PIN, HIGH);
    this->_stepping = 1;
  }
  else {
    digitalWrite(E_STEP_PIN, LOW);
    this->_stepping = 0;
  }
  return 0;
}
