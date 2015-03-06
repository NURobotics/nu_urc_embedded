#ifndef RoverControl_h
#define RoverControl_h

#include "Arduino.h"

class RoverControl
{
public:
	RoverControl(int lepin, int lppin, int repin, int rppin);
	void setdirection(float dirx, float diry);
	void stop();
private:
	int s;
	int direction;
	int speed;
	float turn;
	int leftepin;
	int leftppin;
	int rightepin;
	int rightppin;
	int cnstrn(int val);
	float t;
};

#endif