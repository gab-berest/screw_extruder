#define HEATER_0_PIN  10
#define TEMP_0_PIN    13
#define FAN_PIN       9


#define OUTPUT 1

using namespace std;

class Temperature {
  public:
    Temperature(int set_temp = 0);
    ~Temperature();
    int setTemperature(int temp);
    int getTemperature();
    int readthermocouple();
    int controlTemperature();
  private:
    int _current_temp = 0;
    int _set_temp = 0;
};
