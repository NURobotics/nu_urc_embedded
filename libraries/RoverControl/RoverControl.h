#ifndef RoverControl_h
#define RoverControl_h

#include "Arduino.h"

class rovercmd
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
	float arctan(float v);
	float t;
};

#endif