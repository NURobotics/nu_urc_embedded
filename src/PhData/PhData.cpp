#include "Arduino.h"
#include "PhData.h"
#include "Math.h";

float pHLevels [];
float bestPHData[];
PhData::PhData(float pHs[]) {
   pHLevels = pHs;
}

void PhData::calibrate() {
  
}

void PhData::findBestAverage() {
  
  
}

void PhData::filterData() {
  int length = pHLevels.size();
  float deviations [length];
  float avg;
  float standardDev;
  for (int i = 0; i < length); i++) {
    avg += pHs[i]; 
  }
  avg = avg / length;
  for (int i = 0; i < length; i++) {
     deviations[i] = pow(pHs[i] - avg,2);
  }
  for (int i = 0; i < length; i++) {
     standardDev[i] += deviations[i];
  }
  standardDev = pow(standardDev/length,.5f);
  int num = 1;
  int* bestData = new int [num];
  for (int i = 0; i < length; i++) {
     if (abs(pHs[i] - (avg + 2*standardDev)) < 0) {
       bestData[num - 1] = pHs[i];
       num++;
     }
  }
  bestPHData = bestData;
  delete [] bestData;
}

