#include "variables.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <atomic>

//Initialise devices
pros::Controller controller(CONTROLLER_MASTER);	

pros::Motor topLeft(3, MOTOR_GEAR_BLUE, true);
pros::Motor topRight(4, MOTOR_GEAR_BLUE, false);

pros::Motor catapultLeft(0, MOTOR_GEAR_GREEN, true);
pros::Motor catapultRight(0, pros::E_MOTOR_GEAR_GREEN, false);
pros::Motor_Group catapult_motors({catapultLeft, catapultRight});

pros::Motor bottomLeft(2, MOTOR_GEAR_BLUE, true);
pros::Motor bottomRight(11, MOTOR_GEAR_BLUE, false);

pros::Motor_Group left_motors({topLeft, bottomLeft});
pros::Motor_Group right_motors({topRight, bottomRight});

pros::ADIDigitalOut PTOpiston('E');
std::atomic_bool is_PTO_on_base = false;

pros::Motor intake(0, MOTOR_GEAR_BLUE, false);

pros::Motor flywheel(0, pros::E_MOTOR_GEAR_BLUE);
pros::ADIDigitalOut wings('H');

//Initialise sensors
pros::Imu inertial(19);
pros::Rotation right_tracker(1);
pros::Rotation back_tracker(13, true);
