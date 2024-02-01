#include "variables.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <atomic>

//Initialise devices
pros::Controller controller(CONTROLLER_MASTER);	

pros::Motor topLeft(9, MOTOR_GEAR_BLUE, true);
pros::Motor topRight(13, MOTOR_GEAR_BLUE, false);

pros::Motor catapultLeft(10, MOTOR_GEAR_GREEN, true);
pros::Motor catapultRight(2, pros::E_MOTOR_GEAR_GREEN, false);
pros::Motor_Group catapult_motors({catapultLeft, catapultRight});

pros::Motor bottomLeft(11, MOTOR_GEAR_BLUE, true);
pros::Motor bottomRight(1, MOTOR_GEARSET_18, false);

pros::Motor_Group left_motors({topLeft, bottomLeft});
pros::Motor_Group right_motors({topRight, bottomRight});

pros::ADIDigitalOut PTOpiston('H');
std::atomic_bool is_PTO_on_base = true;

pros::Motor intake(0, MOTOR_GEAR_BLUE, false);

pros::Motor flywheel(0, pros::E_MOTOR_GEAR_BLUE);
pros::ADIDigitalOut wings('F');

//Initialise sensors
pros::Imu inertial(0);
pros::Rotation left_tracker(0);
pros::Rotation right_tracker(0);
pros::Rotation back_tracker(0);
