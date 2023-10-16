#include "variables.h"
#include "main.h"

//Initialise devices
pros::Controller controller(CONTROLLER_MASTER);	
pros::Motor topLeft(13, MOTOR_GEARSET_18, false);
pros::Motor topRight(18, MOTOR_GEARSET_18, true);
pros::Motor bottomLeft(11, MOTOR_GEARSET_18, true);
pros::Motor bottomRight(17, MOTOR_GEARSET_18, false);

pros::Motor_Group left_motors({topLeft, bottomLeft});
pros::Motor_Group right_motors({topRight, bottomRight});

pros::Motor intake(12, MOTOR_GEAR_BLUE, false);
pros::Motor catapult(2, MOTOR_GEAR_RED, false);
pros::Motor lift1(7, MOTOR_GEAR_RED, false);
pros::Motor lift2(7, MOTOR_GEAR_RED, false);
pros::Motor_Group lift({lift1, lift2});
pros::ADIDigitalOut wings('H');
pros::ADIDigitalOut claw('A');

//Initialise sensors
pros::Imu inertial(10);
pros::Distance front_dist(9);
pros::Distance back_dist(3);
pros::Rotation cataRotation(14);