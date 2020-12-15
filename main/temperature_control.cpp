#include "temperature_control.h"

using namespace std;

Temperature::Temperature(int set_temp) {
  //pinMode(HEATER_0_PIN, OUTPUT);
  //pinMode(FAN_PIN, OUTPUT);
  
  this->setTemperature(set_temp);
  
}
