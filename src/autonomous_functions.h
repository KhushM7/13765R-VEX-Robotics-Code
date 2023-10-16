#include <string>
#ifndef AUTON_FUNCTIONS_H
#define AUTON_FUNCTIONS_H

void robot_set_heading_PID(double angle);
void robot_set_heading(double angle);
void stop_robot();
/* Moves the robot in a given direction to a specified distance from the wall. 
 * If no distance is specified, the robot will indefinitely move forwards or backwards unless told otherwise
 * NOTE: parameter sensor must either be BACK or FRONT
 * NOTE: parameter speed must be in rpm
 */ 
void robot_move_to(int speed, std::string sensor, double distance, bool should_slow_down);

void robot_moveTo_PID(std::string sensor, double distanceFromObject, double kP, double kI, double kD);
#endif