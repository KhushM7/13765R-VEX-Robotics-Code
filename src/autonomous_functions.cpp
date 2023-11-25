#include <stdlib.h>
#include "autonomous_functions.h"
#include "pros/rtos.hpp"
#include "variables.h"


void robot_move_to(int motor_speed, std::string sensor, double distance, bool should_slow_down){
	// If a distance is not specified, the robot will continue the move forwards
	double error; //distance away from target
	double PI = 3.141592654;
	int initialMaxMotorSpeed = motor_speed;
	//Using backleft motor encoder position for distance for now
	double prevBLMotorPosition = bottomLeft.get_position();
	int current_speed = 0;

	if (sensor == "BACK"){
		error = distance - back_dist.get();
		if (error < 0){
			motor_speed *= -1;
		}		
	}
	else if (sensor == "FRONT") {
		error = distance - front_dist.get();		
	}
	// else if (sensor == "WHEELS"){
	// 	error = distance;
	// }
	else {
		controller.print(2,0, "Enter the correct sensor parameter");
		controller.rumble(".......");
	}

	// Moves the motors forwards
	if (motor_speed - current_speed > 20){
		current_speed += (motor_speed - current_speed) * 0.1;
	}
	else if (current_speed - motor_speed < -20){
		current_speed += (current_speed - motor_speed) * 0.1;
	}
	left_motors.move_velocity( current_speed);
	right_motors.move_velocity(current_speed);
	
	while (error >=  5 || error <= -5){
		if (sensor == "BACK"){
			error = distance - back_dist.get();
		}
		else if (sensor == "FRONT"){
			error = distance - front_dist.get();
		}
		// else if (sensor == "WHEELS"){
		// 	// error = Pi * diameter of wheel * theta/360
		// 	error -= 100 * PI * ((bottomLeft.get_position()- prevBLMotorPosition)/360);
		// 	controller.print(0, 0, "%F", error);
		// 	prevBLMotorPosition = bottomLeft.get_position();
		// 	if (error < 0){
		// 		error = 0;
		// 	}
		// }
		if (abs(error) < 300 && should_slow_down){
			// Slow down the motor once it gets within the range of 300 mm
			// This is only good when travelling longer distances
			// The motor speed set is a percentage of the initial
			motor_speed = (2.0/3.0) * error;
			if (motor_speed > initialMaxMotorSpeed && initialMaxMotorSpeed > 0){
				motor_speed = initialMaxMotorSpeed;
			}
			else if (motor_speed < initialMaxMotorSpeed && initialMaxMotorSpeed < 0){
				motor_speed = initialMaxMotorSpeed;
			}
		}
		else if (abs(error) > 300 && current_speed < motor_speed){
			if (motor_speed - current_speed > 20){
				current_speed += (motor_speed - current_speed) * 0.1;
			}
			else if (current_speed - motor_speed < -20){
				current_speed += (current_speed - motor_speed) * 0.1;
			}
		}
		left_motors.move_velocity(current_speed);
		right_motors.move_velocity(current_speed);
		pros::delay(20);		
	}

	//Stop the motors
	left_motors.brake();
	right_motors.brake();	
}

void robot_set_velocity(double speed, double milliseconds){
	left_motors.move_velocity(speed);
	right_motors.move_velocity(speed);
	pros::delay(milliseconds);
	left_motors.brake();
	right_motors.brake();
}

//Moves the robot until it is at a specified location, using a PID controller
void robot_moveTo_PID(std::string sensor, double distanceFromObject, bool profiledMotion ){
    //Let's define some variables that will be useful for PID
	const double kP = 1.1;
	const double kI = 0.;
	const double kD = 0.35;

    double error = 0;
    double integral = 0;
    double derivative = 0;
    double prevError;
    double power_to_motors = 0;

	//This is used for slowly "ramping up" the power
	double prev_power_to_motors = 0;

	if (sensor == "BACK"){
		prevError = back_dist.get(); //Necessary to stop program giving undefined errors
	}
	else if (sensor == "FRONT"){
		prevError = front_dist.get();
	}
	else{
		controller.print(0,0, "Incorrect sensor argument - must be BACK or FRONT!");
	}
	
	error = prevError;

	// Keep doing PID loop until the robot is within 5mm of the desired value
	while (abs(error) > 10 || derivative >  3){
		//Proportional - calculating error
		
		if (sensor == "BACK"){
			error = distanceFromObject - back_dist.get(); 
		}
		else if (sensor == "FRONT"){
			error = distanceFromObject - front_dist.get();
		}

		//Integral - only 'activate' it when the error is less than 200mm (20cm)
		//This prevents integral windup
		if (abs(error) < 200){
			integral += error;
		}		

		//Once we have reahed our desired location, we must reset the integral to prevent overshooting
		if (abs(error) < 10){
			integral = 0;
		}

		// Derivative
		derivative = error - prevError;
		prevError = error;

		power_to_motors = (kP * error) + (kI * integral) + (kD * derivative);

		//Max change of power from 0 to 10V in 20 msec
		if (profiledMotion && power_to_motors - prev_power_to_motors > 10){
			power_to_motors = prev_power_to_motors + 10;
		}

		left_motors = power_to_motors;
		right_motors = power_to_motors;
		prev_power_to_motors = power_to_motors;
		pros::delay(20); //Essential for both integral and derivative
    }
	
	stop_robot();
	pros::delay(1000);
	if (sensor == "BACK"){
			error = distanceFromObject - back_dist.get(); 
		}
	else if (sensor == "FRONT"){
			error = distanceFromObject - front_dist.get();
		}
	controller.print(0, 0, "%F", error);
}

// Turns the robot until it has rotated to the specified angle.
// This function uses a PID controller.
void robot_set_heading_PID(double angle)
{
	//Let's define some variables that will be useful for PID
	double kP = 2.15;
	double kI = 0.11;
	double kD = 0.53;

	double error = angle - inertial.get_heading();
	double integral = 0; //This is our integral (needed for I component)
	double derivative = 0;
	double prevError = angle - inertial.get_heading(); // This is needed to calculate derivative
	double power_to_motors = 0;
	
	while (abs(error) > 0.8 || abs(derivative) > 0.5){
		//Rotate using PID where the required angle is therefore greater than the current heading value
		//Proportional - calculating the error
		if (abs(angle - inertial.get_heading()) <= 180)
		{
			error = angle - inertial.get_heading();
		}

		else
		{
			if (inertial.get_heading() > angle)
			{
				error = 360 + angle - inertial.get_heading();
			}
			else
			{
				error =  angle - inertial.get_heading() - 360;
			}
		}

		// Integral			
		integral += error;

		// When we reach our target value, we need to reset our integral so that the robot
		// doesn't overshoot
		if (abs(error) < 0.2){
			integral = 0;
		}

		// To prevent integral windup 
		if (abs(error) > 10){
			integral = 0;
		}

		// Derivative
		derivative = error - prevError; //This is the change of error
		prevError = error;
		
		//Power = proportional + integral + derivative
		power_to_motors = (kP * error) + (kI * integral)+ (kD * derivative);

		if (power_to_motors >= -7 && power_to_motors < -1){
			power_to_motors = -10;
		}
		else if(power_to_motors <= 7 && power_to_motors > 1){
			power_to_motors = 10;
		}	

		left_motors.move(power_to_motors);
		right_motors.move(-power_to_motors); //The right motors must spin the other way
		
		pros::delay(15); // This is essential for both integral and derivative.
	}

	left_motors.brake();
	right_motors.brake();
}

void stop_robot(){
	left_motors.brake();
	right_motors.brake();
}