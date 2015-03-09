#define PhData
#include "Arduino.h"

class PhData {
  public:
    PhData(float pHs[]);
    void calibrate();
    void findBestAverage();
    void filterData();
};
