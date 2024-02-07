#include <atomic>
#include <stdlib.h>
#include "autonomous_functions.h"
#include "pros/rtos.hpp"
#include "variables.h"


const double PI = 3.1415926535897931;
std::atomic<double> robot_x = 0;
std::atomic<double> robot_y = 0;	
std::atomic<double> robot_heading = 0;

double centidegreesToRadians(double centidegrees){
    return (centidegrees * 0.01 * (PI / 180.0));
}

double degreesToRadians(double degrees){
	return (degrees * (PI/ 180.0));
}

double radiansToDegrees(double radians){
	return (radians * 180 / PI);
}

void odometry_tracker(){
	//This is the function that will track the robot's motion
	//Define some variables
	//All measurements are in inches
	const double wheel_radius = 1.375; //Radius of tracking wheels in inches
	const double sR = 1.49; //Displacement of right tracking wheel from centre
	const double sB = 0.512; //Displacement of back tracking wheel from centre

	//These variables are the previous rotation of each tracking wheel
	int prev_right_pos = right_tracker.get_position();
	int prev_back_pos = back_tracker.get_position();	

	double change_in_heading = 0;
	

	//These variables represent the distance travelled by a wheel per cycle
	double deltaR, deltaB;

	//Below are variables used to store the local x,y offset
	double localXOffset = 0, localYOffset = 0;

	//The below local variable is used in case the global variables are in use
	double current_heading = robot_heading.load(); 
	double robotX = robot_x.load();
	double robotY = robot_y.load();
	double prev_heading = current_heading;
	int noCycles = 0;

	//Set the refresh rate of the rotation sensors to be as small as possible
	right_tracker.set_data_rate(5);
	back_tracker.set_data_rate(5);
	inertial.set_data_rate(5);
	while (true){
		//Step 1: calculate the distance each wheel has travelled
		//Wheel travel = change in wheel position (in radians) * wheel radius
		//s = r * deltaTheta
		deltaR = centidegreesToRadians(right_tracker.get_position() - prev_right_pos) * wheel_radius;
		deltaB = centidegreesToRadians(back_tracker.get_position() - prev_back_pos) * wheel_radius;

		//Update the previous variables		
		prev_right_pos = right_tracker.get_position();
		prev_back_pos = back_tracker.get_position();		

		//Step 2: calculate the heading and change in heading (in degrees) of the robot
		current_heading = inertial.get_heading();
		
		//If the robot has changed its heading by more than 180 degrees in 5 ms,
		//it has just finished a rotation. (e.g. from 358 degrees to 4 degrees or
		//vice versa. This is a change in heading of 6 degrees)
		if (abs(prev_heading - current_heading) <= 180){
			change_in_heading = current_heading - prev_heading;
		}
		else
		{
			if (current_heading > prev_heading)
			{
				change_in_heading = -(360 + prev_heading - current_heading);
			}
			else
			{
				change_in_heading =  -(prev_heading - current_heading - 360);
			}
		}
		prev_heading = current_heading;

		//Step 3: calculate the change in position of the robot
		if (abs(change_in_heading) <0.1){
			//If the robot has not changed its heading
			localXOffset = deltaB;
			localYOffset = deltaR;
		}
		else{
			localXOffset = 2 * std::sin(degreesToRadians(change_in_heading)/2) * ((deltaB/degreesToRadians(change_in_heading)) + sB);
			localYOffset = 2 * std::sin(degreesToRadians(change_in_heading)/2) * ((deltaR/degreesToRadians(change_in_heading)) + sR);
		}

		//Step 4: Convert the local change in position to the global change in position
		double average_heading = degreesToRadians(current_heading - (change_in_heading/2)); //In radians
		//We then rotate the current local offsets by -1 * average_heading
		//We can use a formula to rotate the vector		
		robotX += std::cos(-average_heading) * localXOffset - std::sin(-average_heading) * localYOffset;
		robotY += std::sin(-average_heading) * localXOffset + std::cos(-average_heading) * localYOffset;

		//Step 6: Update global variables
		robot_x.store(robotX);
		robot_y.store(robotY);
		robot_heading.store(current_heading);

		if (noCycles >= 20){
			controller.print(0, 0, "X: %.2F, Y: %.2F", robot_x.load(), robot_y.load());
			//controller.print(0, 0, "%.2F", current_heading);
		}
		noCycles++;

		pros::delay(10);
	}
	
}

void robotMoveTo(int targetX, int targetY, bool frontFacing){
	//We need to calculate turn velocity and drive velocity separately
	//Then we combine the two at the end

	//We will use a simple proportional loop for now
	//If that is not precise enough then we will use the full PID loop
	
	//Let's define some variables
	double drive_kP = 0;
	double turn_kP = 0;

	double turnError = 0;
	double driveError = std::sqrt(pow(targetX - robot_x.load(), 2) + pow(targetY - robot_y.load(), 2));

	double driveVelocity;
	double turnVelocity;

	//While the error is high or our robot is fast
	while (driveError > 0.5 || abs(left_motors.get_target_velocities()[0]) > 10 || abs(right_motors.get_target_velocities()[0]) > 10){
		//Calculate drive error using Pythagoras' Theorem
		driveError = std::sqrt(pow(targetX - robot_x.load(), 2) + pow(targetY - robot_y.load(), 2));

		//Calculate desired heading using algorithm from before
		double desired_heading = atan2(targetY - robot_y, targetX - robot_x); 
		desired_heading = desired_heading * 180.0 / PI;
		desired_heading = 90 - desired_heading;
		if (desired_heading < 0)
		{
			desired_heading += 360;
		}
		if (!frontFacing)
		{
			desired_heading += 180;
			if (desired_heading >= 360) {
				desired_heading -= 360;
			}
			// Since we are going backwards, the drive error must be negative
			driveError = -driveError;
		}

		//Now calculate actual turn error
		if (abs(desired_heading - inertial.get_heading()) <= 180)
		{
			turnError = desired_heading - inertial.get_heading();
		}

		else
		{
			if (inertial.get_heading() > desired_heading)
			{
				turnError = 360 + desired_heading - inertial.get_heading();
			}
			else
			{
				turnError =  desired_heading - inertial.get_heading() - 360;
			}
		}

		//Calculate both drive and turn velocities now
		driveVelocity = driveError * drive_kP;
		turnVelocity = turnError * turn_kP;

		//Reset turn velocity if the robot is too close to the target
		//For now, this is when the robot is 2 inches away from the target
		if (abs(driveError) < 2){
			turnVelocity = 0;
		}

		//Combine the two together
		left_motors.move(driveVelocity + turnVelocity);
		right_motors.move(driveVelocity - turnVelocity);
	}
}

void robotMoveBy(double dist_in_inches){
	//Travel forward (or back) by a certain distance
	//Define some PID variables
	const double kP = 0.1;
	const double kI = 0;
	const double kD = 0.01;

	//Diameter of tracking wheel
	const double wheel_diameter = 2.75;

	//Calculate the position the tracking wheel will need to end at
	//Position = Current_Position - (distance/ PI*diameter of tracker wheel)*36000
	//Error will be in number of rotations rather than distance
	double target_wheel_pos = ((dist_in_inches/ (PI * wheel_diameter)) * 36000) - right_tracker.get_position();
    double error = target_wheel_pos - right_tracker.get_position();
    double integral = 0;
    double derivative = 0;
    double prevError;
    double power_to_motors = 0;	

	//Keep going until we are only 5mm or roughly a quarter of an inch away from the target
	//If we are moving too fast, we do not exit the loop
	while (abs(error) > 600 || derivative > 500){
		//Calculate the error using the previous formula
		error = target_wheel_pos - right_tracker.get_position();

		// Integral			
		integral += error;

		// When we reach our target value, we need to reset our integral so that the robot
		// doesn't overshoot
		if (abs(error) < 600){
			integral = 0;
		}

		// To prevent integral windup 
		if (abs(error) > 1000){
			integral = 0;
		}

		// Derivative
		derivative = error - prevError; //This is the change of error
		prevError = error;
		
		//Power = proportional + integral + derivative
		power_to_motors = (kP * error) + (kI * integral)+ (kD * derivative);

		left_motors.move(power_to_motors);
		right_motors.move(power_to_motors); 
	}
}

void robotRotateToPoint(int targetX, int targetY, bool frontFacing){
	//First calculate the desired heading
	double desired_heading = atan2(targetY - robot_y, targetX - robot_x); 
    desired_heading = desired_heading * 180.0 / PI;
    desired_heading = 90 - desired_heading;
    if (desired_heading < 0)
    {
        desired_heading += 360;
    }
    if (!frontFacing)
    {
        desired_heading += 180;
        if (desired_heading >= 360) {
            desired_heading -= 360;
        }
    }
	
	//Now rotate to that heading
	robot_set_heading_PID(desired_heading);
}

void robotRotateThenMoveTo(int targetX, int targetY, bool frontFacing){
	robotRotateToPoint(targetX, targetY, frontFacing);
	pros::delay(200); //Allow robot to fully stop
	//Now use PID to move to the point in a straight line
	//If robot doesn't move in a straight line at first, may need to 
	//a pure pursuit like thing.

	//Define some PID variables
	const double kP = 0;
	const double kI = 0;
	const double kD = 0;

	//Uses Pythagoras' theorem
    double error = std::sqrt(pow(targetX - robot_x.load(), 2) + pow(targetY - robot_y.load(), 2));
    double integral = 0;
    double derivative = 0;
    double prevError;
    double power_to_motors = 0;	

	//Keep going until we are only 5mm or roughly a quarter of an inch away from the target
	//If we are moving too fast, we do not exit the loop
	while (abs(error) > 0.25 || derivative > 0.5){
		//Calculate the error using Pythogoras' theorem
		error = std::sqrt(pow(targetX - robot_x.load(), 2) + pow(targetY - robot_y.load(), 2));
		if (!frontFacing){
			//If we are going backwards, error must be negative
			error = -error;
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

		left_motors.move(power_to_motors);
		right_motors.move(power_to_motors); 
	}
}

void robotFollowPoints(double points[]){
	
};
void robot_set_velocity(double speed, double milliseconds){
	left_motors.move_velocity(speed);
	right_motors.move_velocity(speed);
	pros::delay(milliseconds);
	left_motors.brake();
	right_motors.brake();
}

// Turns the robot until it has rotated to the specified angle.
// This function uses a PID controller.
bool robot_set_heading_PID(double angle, bool failSafeIsON)
{
	//Let's define some variables that will be useful for PID
	double kP = 2.53;
	double kI = 0.11;
	double kD = 0.51;

	double current_angle = robot_heading.load();
	double error = angle - current_angle;
	double integral = 0; //This is our integral (needed for I component)
	double derivative = 0;
	double prevError = angle - current_angle; // This is needed to calculate derivative
	double power_to_motors = 0;
	

	//Start a little timer: this to catch whether we get stuck or not
	int startTime = pros::millis();
	
	while (abs(error) > 0.8 || abs(derivative) > 0.5){
		//Rotate using PID where the required angle is therefore greater than the current heading value
		//Proportional - calculating the error
		current_angle = robot_heading.load();

		if (abs(angle - current_angle) <= 180)
		{
			error = angle - current_angle;
		}

		else
		{
			if (current_angle > angle)
			{
				error = 360 + angle - current_angle;
			}
			else
			{
				error =  angle - current_angle - 360;
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
		//We wait for 0.8 seconds to ensure the robot has had time to accelerate
		if (failSafeIsON && pros::millis() - startTime > 800 && abs(power_to_motors) >= 7 
		&& abs(left_motors[0].get_actual_velocity()) < 3){
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