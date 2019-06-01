#include<Arduino.h>
#ifndef Motor_h
#define Motor_h
class Motor {
private:
	byte enable;
	byte in1;
	byte in2;
public:
	Motor(const byte Enable, const byte In1, const byte In2) {
		this->enable = Enable;
		this->in1 = In1;
		this->in2 = In2;

		pinMode(enable, OUTPUT);
		pinMode(in1, OUTPUT);
		pinMode(in2, OUTPUT);
	}

	void Run(const int Speed) {
		if (Speed >= 0)
		{
			digitalWrite(in1, LOW);
			digitalWrite(in2, HIGH);
			analogWrite(enable, Speed);
		}
		else if(Speed < 0){
			digitalWrite(in1, HIGH);
			digitalWrite(in2, LOW);
			analogWrite(enable, -Speed);
		}
	}

	void Stop(const int Speed) {
		digitalWrite(in1, HIGH);
		digitalWrite(in2, HIGH);
		analogWrite(enable, Speed);
	}
	
	void Standby(){
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
		analogWrite(enable, 0);
	}
	
	
};
#endif

