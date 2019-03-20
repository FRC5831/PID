/*
	PIDController.h
	2018-10-17
	by James Hoi
	email:jameshoi@foxmail.com

Description:
	A example of pidcontroller.

	Robot: Two motor,
		   Two encoder
	Target: Driving Specified distance

Notes:
	SetSpeed is not a real function, 
	you need to change to your motor speed function.
*/

#include"PIDController.h"
#include"Config.h"

PIDController pid_speed[2];
PIDController pid_position[2];
const int rpm_zero[2] = {0,0};

//Your Sensor Value
int current_position[2];
int current_rpm[2];

//Your Target
int target_positon[2];

void Initialize(){
	InitPIDController(pid_speed,Kp_RPM,Ti_RPM,Td_RPM,Tolerance_RPM);
	InitPIDController(pid_position,Kp_Position,Ti_Position,Td_Position,Tolerance_Position);
	SetLimitOutput(pid_position,MaxOutput_PWM,MinOutput_PWM);
}

float cascade(PIDController pid_position,PIDController pid_speed,int current_position,int target_position,int current_rpm){
	float target_rpm = PIDOutput(pid_position,current_position,target_positon)/pid_position.MaxOutput_PWM;
	return PIDOutput(pid_speed,current_rpm,target_rpm);
}

int main(){
	Initialize();
	int size = sizeof(pid_speed)/sizeof(pid_speed[0]); // or sizeof(pid_position)/sizeof(pid_position[0]);
	while(true){
		for(int i=0;i<=size-1;i++){
			if(pid_position.OnTarget)SetSpeed(rpm_zero);
			else SetSpeed(cascade(pid_position[i],pid_speed[i],current_position[i],target_positon[i],current_rpm[i]));
		}
	}
}