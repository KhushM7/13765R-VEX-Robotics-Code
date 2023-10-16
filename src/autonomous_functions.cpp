#include <stdlib.h>
#include "autonomous_functions.h"
#include "pros/rtos.hpp"
#include "variables.h"


void robot_move_to(int motor_speed, std::string sensor, double distance, bool should_slow_down){
	// If a distance is not specified, the robot will continue the move forwards
	double error; //distance away from target
	int initialMaxMotorSpeed = motor_speed;

	if (sensor == "BACK"){
		error = distance - back_dist.get();
		controller.print(0,0, "%d", error);
		if (error < 0){
			motor_speed *= -1;
		}		
	}
	else if (sensor == "FRONT") {
		error = distance - front_dist.get();		
	}
	else {
		controller.print(2,0, "Enter the correct sensor parameter");
		controller.rumble(".......");
	}
	controller.print(0,0, "%d", motor_speed);

	// Moves the motors forwards
	left_motors.move_velocity(motor_speed);
	right_motors.move_velocity(motor_speed);
	
	while (distance != 0 && (error >=  5 || error <= -5)){
		if (sensor == "BACK"){
			error = distance - back_dist.get();
		}
		else if (sensor == "FRONT"){
			error = distance - front_dist.get();
		}
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
		left_motors.move_velocity(motor_speed);
		right_motors.move_velocity(motor_speed);
		pros::delay(20);		
	}

	// Only stop the motors if a distance has been set.
	// Otherwise the motors must be stopped manually
	if (distance != 0){
		left_motors.brake();
		right_motors.brake();
	}
	controller.print(0,0, "Done");		
}

//Moves the robot until it is at a specified location, using a PID controller
void robot_moveTo_PID(std::string sensor, double distanceFromObject, double kP, double kI, double kD){
    //Let's define some variables that will be useful for PID
	//(needs tuning)
	// const double kP = kP;
	// const double kI = kI;
	// const double kD = kD;

    double error = 0;
    double integral = 0;
    double derivative = 0;
    double prevError;
    double power_to_motors;
    
	if (sensor == "BACK"){
		prevError = back_dist.get(); //Necessary to stop program giving undefined errors
	}
	else if (sensor == "FRONT"){
		prevError = front_dist.get();
	}
	else{
		controller.print(0,0, "Incorrect sensor argument - must be BACK or FRONT!");
	}

	// Keep doing PID loop until the robot is within 5mm of the desired value
	while (back_dist.get() > distanceFromObject + 5 || back_dist.get() < distanceFromObject - 5){
		//Proportional - calculating error
		if (sensor == "BACK"){
			error = distanceFromObject - back_dist.get(); //Necessary to stop program giving undefined errors
		}
		else if (sensor == "FRONT"){
			error = distanceFromObject - front_dist.get();
		}

		//Integral - only 'activate' it when the error is less than 200mm (20cm)
		//This prevents integral windup
		if (error < 200){
			integral += error;
		}		

		//Once we have reahed our desired location, we must reset the integral to prevent overshooting
		if (error == 0){
			integral = 0;
		}

		// Derivative
		derivative = error - prevError;
		prevError = error;

		power_to_motors = (kP * error) + (kI * integral) + (kD * derivative);

		left_motors = power_to_motors;
		right_motors = power_to_motors;

		pros::delay(10); //Essential for both integral and derivative
    }
}

// Turns the robot until it has rotated to the specified angle.
// This function uses a PID controller.
void robot_set_heading_PID(double angle)
{
	//Let's define some variables that will be useful for PID
	//(needs tuning)
	double kP = 1;
	double kI = 0;
	double kD = 0;

	double error = angle - inertial.get_heading();
	double integral = 0; //This is our integral (needed for I component)
	double derivative = 0;
	double prevError = angle - inertial.get_heading(); // This is needed to calculate derivative
	double power_to_motors = 0;
	
	while (inertial.get_heading() < angle - 1 || inertial.get_heading() > angle + 1){
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
				error = 360 - angle - inertial.get_heading();
			}
			else
			{
				error = angle - (inertial.get_heading() + 360);
			}
		}

		// Integral			
		integral += error;

		// When we reach our target value, we need to reset our integral so that the robot
		// doesn't overshoot
		if (error < 0.2){
			integral = 0;
		}

		// To prevent integral windup (EXPLAIN IN SKETCHBOOK + may need to adjust)
		if (error > 10){
			integral = 0;
		}

		// Derivative
		derivative = error - prevError; //This is the change of error
		prevError = error;
		
		//Power = proportional + integral + derivative
		power_to_motors = (kP * error) + (kI * integral)+ (kD * derivative);

		left_motors.move(power_to_motors);
		right_motors.move(-power_to_motors); //The right motors must spin the other way
		
		pros::delay(15); // This is essential for both integral and derivative.
	}
	left_motors.brake();
	right_motors.brake();
}

void robot_set_heading(double angle){	
	double error;
	double start_time = pros::millis();
	
	do {
		//Decide direction
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
				error =  inertial.get_heading() - angle - 360;
			}
		}
		//Keep rotating till you reach target
		left_motors.move(error);
		right_motors.move(-error);
	} while ((error > 2 || error < -2) && pros::millis() < start_time + 2000);
}

void stop_robot(){
	left_motors.brake();
	right_motors.brake();
}