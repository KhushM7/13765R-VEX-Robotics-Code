#include <string>
#include <atomic>
#ifndef AUTON_FUNCTIONS_H
#define AUTON_FUNCTIONS_H

bool robot_set_heading_PID(double angle, bool failSafeIsON = false);
void stop_robot();

//Set the velocity of all the drive base motors.
void robot_set_velocity(double speed, double milliseconds);

//Odometry functions and variables
void odometry_tracker();
extern std::atomic<double> robot_x;
extern std::atomic<double> robot_y;
extern std::atomic<double> robot_heading;

#endif