#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

using namespace std;

class Motor {
  public:
    Motor(int step_accuracy = 1, int speed = 0, int direction = 0);
    ~Motor();
    int setDirection(int direction = 0);
    int setAccuracy(int accuracy = 0);
    int setSpeed(int speed = 0);
    int getDirection();
    int getAccuracy();
    int getSpeed();
    int step();
  private:
    int _speed = 0;
    int _direction = 0;
    float _step_accuracy = 0; 
    int _stepping = 0; 
}
