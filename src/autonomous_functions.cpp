#include <complex>
#include <stdlib.h>
#include "autonomous_functions.h"
#include "pros/rtos.hpp"
#include "variables.h"

const double PI = 3.1415926535897931;

double centidegreesToRadians(double centidegrees){
    return (centidegrees * 0.01 * (PI / 180.0));
}

double radiansToDegrees(double radians){
	return (radians * 180 / PI);
}

void odometry_tracker(){
	//This is the function that will track the robot's motion
	//Define some variables
	const double wheel_radius = 2.0; //Radius of tracking wheel in inches
	const double sL = 0; //Displacement of left tracking wheel from centre
	const double sR = 0; //Displacement of right tracking wheel from centre
	const double sB = 0; //Displacement of back tracking wheel from centre

	//These variables are the previous rotation of each tracking wheel
	double prev_left_pos = left_tracker.get_position();
	double prev_right_pos = right_tracker.get_position();
	double prev_back_pos = back_tracker.get_position();

	double change_in_heading;

	//These variables represent the distance travelled by a wheel per cycle
	double deltaL, deltaR, deltaB;

	//Below are variables used to store the local x,y offset
	double localXOffset, localYOffset;

	//The below local variable is used in case the global variables are in use
	double current_heading = robot_heading.load(); 
	double robotX = robot_x.load();
	double robotY = robot_y.load();

	//Set the refresh rate of the rotation sensors to be as small as possible
	left_tracker.set_data_rate(5);
	right_tracker.set_data_rate(5);
	back_tracker.set_data_rate(5);
	while (true){
		//Step 1: calculate the distance each wheel has travelled
		//Wheel travel = change in wheel position (in radians) * wheel radius
		//s = r * deltaTheta
		deltaL = centidegreesToRadians(left_tracker.get_position() - prev_left_pos) * wheel_radius;
		deltaR = centidegreesToRadians(right_tracker.get_position() - prev_right_pos) * wheel_radius;
		deltaB = centidegreesToRadians(back_tracker.get_position() - prev_back_pos) * wheel_radius;

		//Update the previous variables		
		prev_left_pos = left_tracker.get_position();
		prev_right_pos = right_tracker.get_position();
		prev_back_pos = back_tracker.get_position();		

		//Step 2: calculate the heading (in degrees) of the robot
		change_in_heading = (deltaL - deltaR)/(sL - sR); //in radians
		current_heading += radiansToDegrees(change_in_heading);

		//Step 3: calculate the change in position of the robot
		if (change_in_heading == 0){
			//If the robot has not changed its heading
			localXOffset = deltaB;
			localYOffset = deltaR;
		}
		else{
			localXOffset = 2 * std::sin(change_in_heading/2) * ((deltaB/change_in_heading) + sB);
			localYOffset = 2 * std::sin(change_in_heading/2) * ((deltaR/change_in_heading) + sR);
		}

		//Step 4: Convert the local change in position to the global change in position
		double average_heading = current_heading - (change_in_heading/2);
		//We then rotate the current local offsets by -1 * average_heading
		//Convert the offset vector to polar coordinates
		double polar_r = sqrt(pow(localXOffset, 2) + pow(localYOffset, 2));
		double polar_theta = atan2(localYOffset, localXOffset); //in radians
		//Change the angle (theta)
		polar_theta = polar_theta - average_heading;	
		//Convert back to cartesian coordiantes and update global position
		robotX += polar_r * cos(polar_theta);
		robotY += polar_r * sin(polar_theta);

		//Step 5: Limit range of current heading to [0, 360]
		//This step is not necessary but it allows us to average out this with
		//the reading of the inertial sensor
		if (current_heading >= 360){
			current_heading -= 360;
		}
		else if (current_heading < 0){
			current_heading += 360;
		}

		//Step 6: Update global variables
		robot_x.store(robotX);
		robot_y.store(robotY);
		robot_heading.store(current_heading);

		pros::delay(5);
	}
	
}

void robot_move_to(int motor_speed, std::string sensor, double distance, bool should_slow_down){
	// If a distance is not specified, the robot will continue the move forwards
	double error; //distance away from target
	int initialMaxMotorSpeed = motor_speed;
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
bool robot_set_heading_PID(double angle, bool failSafeIsON)
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

	//Start a little timer: this to catch whether we get stuck or not
	int startTime = pros::millis();
	
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

		//Check if the motors are actually able to turn or not
		//If power_to_motors is more than 7, the robot should rotate
		//We wait for 1.5 seconds to ensure the robot has had time to accelerate
		if (failSafeIsON && pros::millis() - startTime > 800 && abs(power_to_motors) >= 7 && abs(left_motors[0].get_actual_velocity()) < 3){
			//At this point the robot has failed to complete its turn
			stop_robot();
			return false;			
		}
		
		pros::delay(15); // This is essential for both integral and derivative.
	}

	left_motors.brake();
	right_motors.brake();
	return true;
}

void stop_robot(){
	left_motors.brake();
	right_motors.brake();
}