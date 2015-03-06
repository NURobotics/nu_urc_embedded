#include "RoverControl.h"
#include "math.h"

RoverControl::RoverControl(int lepin, int lppin, int repin, int rppin)
{
	leftepin = lepin;
	leftppin = lppin;
	rightepin = repin;
	rightppin = rppin;
	speed = 0;
	direction = 0;
	turn = 0;
	pinMode(leftepin, OUTPUT);
	pinMode(leftppin, OUTPUT);
	pinMode(rightppin, OUTPUT);
	pinMode(rightepin, OUTPUT);
	t = 0;

}
int RoverControl::cnstrn(int val)
{
	if (val >= 255){
		return 255;
	}
	return val;
}
void RoverControl::setdirection(float dirx, float diry)
{
	s = cnstrn(sqrt(diry*diry + dirx*dirx));
	speed = s;
	if (dirx == 0){
		turn = 0;
	}
	else {
		turn = (3.1416/2 - abs(atan(diry/dirx)))/3.1416*2;
		turn = dirx/abs(dirx)*turn;
	}
	if (turn < .5 && turn >= 0){
		s = cnstrn(speed*(.5-turn)*2);
	}
	else if (turn >= .5){
		s = cnstrn(speed*(turn-.5)*2);
	}
	else if (turn < 0 && turn >= -.5) {
		s = cnstrn(-speed*(-.5-turn)*2);
	}
	else {
		s = cnstrn(-speed*(turn+.5)*2);
	}
	if (speed >= 0){
		if (diry >= 0){
			direction = 1;
			if ((turn < .5) && (turn >= 0)){
				analogWrite(leftepin, speed);
				digitalWrite(leftppin, HIGH);
				analogWrite(rightepin, s);
				digitalWrite(rightppin, LOW);
			}
			else if (turn >= .5) {
				analogWrite(leftepin, speed);
				digitalWrite(leftppin, HIGH);
				analogWrite(rightepin, s);
				digitalWrite(rightppin, HIGH);
			}
			else if ((turn < 0) && (turn >= -.5)) {
				analogWrite(leftepin, s);
				digitalWrite(leftppin, HIGH);
				analogWrite(rightepin, speed);
				digitalWrite(rightppin, LOW);
			}
			else if (turn <= -.5) {
				analogWrite(leftepin, s);
				digitalWrite(leftppin, LOW);
				analogWrite(rightepin, speed);
				digitalWrite(rightppin, LOW);
			}

		}
		else {
			if (turn < .5 && turn >= 0){
				direction = 1;
				analogWrite(leftepin, speed);
				digitalWrite(leftppin, LOW);
				analogWrite(rightepin, s);
				digitalWrite(rightppin, HIGH);
			}
			else if (turn >= .5){
				direction = 1;
				analogWrite(leftepin, speed);
				digitalWrite(leftppin, LOW);
				analogWrite(rightepin, s);
				digitalWrite(rightppin, LOW);
			}
			else if (turn < 0 && turn >= -.5) {
				analogWrite(leftepin, s);
				digitalWrite(leftppin, LOW);
				analogWrite(rightepin, speed);
				digitalWrite(rightppin, HIGH);
			}
			else {//if (turn <= -.5) {
				analogWrite(leftepin, s);
				digitalWrite(leftppin, HIGH);
				analogWrite(rightepin, speed);
				digitalWrite(rightppin, HIGH);
			}
		}
	}
	else {
		analogWrite(leftepin, 0);
		analogWrite(rightepin, 0);
	}
}
void RoverControl::stop()
{
	analogWrite(leftepin, 0);
	analogWrite(rightepin, 0);
}
