#include <string>
#include <atomic>
#ifndef AUTON_FUNCTIONS_H
#define AUTON_FUNCTIONS_H

bool robot_set_heading_PID(double angle, bool failSafeIsON = false);
void stop_robot();

//Set the velocity of all the drive base motors. If milliseconds = 0, 
//robot moves indefinitely.
void robot_set_velocity(double speed, double milliseconds);

//MOve the robot by a relative distance from its current position
void robotMoveBy(double dist_in_inches);

//Odometry functions and variables
void odometry_tracker();
extern std::atomic<double> robot_x;
extern std::atomic<double> robot_y;
extern std::atomic<double> robot_heading;

//Get the robot to face a point and
//specify whether you want the back of the robot to face the front 
//or the front of the robot
bool robotRotateToPoint(double targetX, double targetY, bool frontFacing, bool failSafeIsON = false);

//Get the robot to first face a point and then move to that point 
//in a straight line
void robotRotateThenMoveTo(double targetX, double targetY, bool frontFacing, int PIDConstants = 0);

//Follow a smooth path of points
//This will likely use pure pursuit
void robotFollowPoints(double points[]);

//Move to a point whilst changing the heading at the same time
//Specify if you want the robot to face its front or not
void robotMoveTo(double targetX, double targetY, bool frontFacing, int PIDConstants = 0, int initialDirection = 0);

#endif