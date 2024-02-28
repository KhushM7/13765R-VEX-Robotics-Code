#include "main.h"
#include "autonomous_functions.h"
#include "display/lv_misc/lv_color.h"
#include "pros/colors.h"
#include "pros/llemu.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "variables.h"
#include <cmath>
#include <string>
#include <vector>
#include <atomic>

//PTO variable
std::atomic_bool isSwitchingPTO = false;

//Autonomous functions
void auton_defensive_SAFE_AWP(){
	//First determine the starting position + heading
	inertial.set_heading(135);
	robot_heading = inertial.get_heading();
	robot_x = 24; //Need to measure
	robot_y = 12; //Need to measure

	//Start the odometry task
	pros::Task odom_task(odometry_tracker);

	//Go back to get space
	robot_set_velocity(-100, 400);
	//Open wings
	wings.set_value(1);
	//Go forward and remove triball
	left_motors.move(80);
	right_motors.move(128);
	pros::delay(600);
	wings.set_value(0);

	//Now go in front of the goal
	robotMoveTo(14, 36, false, 1);	
	//Rotate to face goal and ram triballs in twice
	robot_set_heading_PID(0);
	//Outake triball whilst scoring it
	intake.move_velocity(-600);
	robot_set_velocity(100, 1000);
	
	//Move back and ram the triball again
	robot_set_velocity(-100, 400);
	robot_set_velocity(100, 600);

	//Now go to EV bar and get AWP.
	//Get some space
	robot_set_velocity(-100, 300);
	robotRotateThenMoveTo(36, 12, true, 1);
	//Reverse intake to hopefully get triballs to our offensive side
	intake.move_velocity(-600);
	robotRotateThenMoveTo(60, 12, true, 1);
	intake.brake();
}

//This route focusses on taking the one of the centre triballs away as 
// fast as possible, by using our intake to remove the triballs
// from the centre.
void auton_defensive_CENTRAL_SNAG(){
	//First determine the starting position + heading
	inertial.set_heading(0);
	robot_heading = inertial.get_heading();
	robot_x = 36; //Need to measure
	robot_y = 12; //Need to measure

	//Start the odometry task
	pros::Task odom_task(odometry_tracker);

	//Go forward and then move to the centre triball
	robot_set_velocity(100, 400);
	//Now rotate whilst moving towards first centre triball
	intake.move_voltage(12000);
	robotMoveTo(48,59, true);
	//Now turn away
	robotRotateToPoint(34, 34, true);

	//Go to matchload bar
	robotMoveTo(34, 34, true);
	intake.move(-128);
	robotRotateThenMoveTo(14, 24, true);

	//Remove corner triball
	robot_set_heading_PID(135);
	wings.set_value(1);
	robot_set_velocity(100, 500);
	wings.set_value(0);

	//Now go and touch EV bar for AWP.
	//Reverse intake to try and push triballs over to other side
	intake.move_voltage(-12000);
	robotMoveTo(63, 10, true, 3); 
}

void auton_offensive(){
	//First determine the starting position + heading
	inertial.set_heading(0); 
	robot_heading = inertial.get_heading();
	robot_x = 108; //Need to measure
	robot_y = 16; //Need to measure

	//Start the odometry task
	pros::Task odom_task(odometry_tracker);

	//Go and intake centre triball.
	//Directly move to the target
	intake.move(-127);
	robot_set_velocity(100, 400);
	intake.move(127);
	robotMoveTo(76, 65, true, 3);
	stop_robot();
	
	//Now score both centre triballs
	robot_set_heading_PID(90, false);
	
	intake.move(-127);
	wings.set_value(1);
	robot_set_velocity(100, 1000);
	intake.brake();

	//Go back and score non-neutral zone triballs
	robot_set_velocity(-100, 100); //Get some space
	//Walk to the triball nearest to us
	intake.move(127);
	robotRotateThenMoveTo(96, 44, true);
	robotRotateThenMoveTo(81,44, true);
	//Now go score stuff
	robot_set_heading_PID(90);
	intake.move(-127);
	wings.set_value(1);
	robot_set_velocity(100, 1000);
	robot_set_velocity(-100, 300);
	robot_set_velocity(100, 500);

}

void auton_skills(){
	//Start a timer so that we can know how long 
	//there is left in the auton
	uint32_t startTime  = pros::millis();

	//First determine the starting position + heading
	inertial.set_heading(0);
	robot_heading = inertial.get_heading();
	robot_x = 36; 
	robot_y = 16; 

	//Start the odometry task now
	pros::Task odom_task(odometry_tracker);

	//Now begin actual routine.
	//Now go score red triballs
	wings.set_value(1);
	pros::delay(200);
	//Start flywheel
	flywheel.move(100);
	//Now rotate and go to the matchload bar
	robotRotateThenMoveTo(8, 32, true, 4);	

	//Rotate to the other goal and start matchloading
	robot_set_heading_PID(175);
	//Wait till there are 25 seconds left
	while (pros::millis() - startTime > 59000){
		pros::delay(20);
	}

	//Stop the matchloading phase
	flywheel.brake();
	wings.set_value(0); //Now we can close the wings

	//Score the red triballs
	robot_set_velocity(-100, 100); //Get some space
	robot_set_heading_PID(0);
	//Ram into the triballs now
	robot_set_velocity(100, 700);
	
	//Now move to the other side of the field
	//Get some space
	robot_set_velocity(-100, 100);
	//Move to other side now
	//Reverse the intake to ensure we are not possessing triballs
	intake.move(-127); 
	robotRotateThenMoveTo(36, 11, true);
	robotRotateThenMoveTo(108, 11, true, 1);

	//Score triballs from side of goal
	robotRotateThenMoveTo(132, 36, true);
	robot_set_heading_PID(0);
	//Ram triballs twice
	robot_set_velocity(100, 600);
	robot_set_velocity(-100, 100);
	robot_set_velocity(100, 300);

	//Now go from the long side of the goal
	//Move away from goal
	robot_set_velocity(100, 250);
	
	//Go to the short bar - long bar corner
	robotRotateThenMoveTo(84, 36, false);

	//Now head towards centre of the field
	robotRotateThenMoveTo(84, 72, true);

	//Rotate to the goal
	robot_set_heading_PID(90);
	//Open wings and score	
	wings.set_value(1);
	robot_set_velocity(100, 2000);
	wings.set_value(0);
	
	//Now score the triball from an angle
	//Get some space
	robot_set_velocity(-100, 300);
	robotRotateToPoint(80, 84, false);
	//Go back, open wings, and ram into triballs from an angle
	robot_set_velocity(-100, 800);
	wings.set_value(1);
	robot_set_velocity(100, 1000);	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	while (!pros::lcd::is_initialized()){
		pros::delay(20);
	}
	pros::lcd::set_background_color(LV_COLOR_BLACK);
	pros::lcd::set_text_color(LV_COLOR_WHITE);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

//AUTONOMOUS SELECTOR stuff
typedef enum{
	OFFENSIVE,
	DEFENSIVE,
	SKILLS
} autonStates;
autonStates auton_state = SKILLS; //Default. 
//Also may not be able to select auton during skills
bool hasConfimed = false;

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	while (!pros::competition::is_autonomous()){
		if(pros::lcd::read_buttons() & LCD_BTN_LEFT){
			//Set the auton to be offensive
			auton_state = OFFENSIVE;
			pros::lcd::set_text(0, "OFFENSIVE");
		}
		if (pros::lcd::read_buttons() & LCD_BTN_CENTER){
			//Set the auton to be defensive
			auton_state = DEFENSIVE;
			pros::lcd::set_text(0, "DEFENSIVE");			
		}
		if (pros::lcd::read_buttons() & LCD_BTN_RIGHT){
			//Set the auton to be for skills
			auton_state = SKILLS;
			pros::lcd::set_text(0, "SKILLS   ");	
		}
		pros::delay(5);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//Wait until the inertial is done calibratng
	while (inertial.is_calibrating()){
		pros::delay(20);
	}
	//Now start the autonomous
	auton_skills();
}

//Gets the hottest motor, printing a two character code that represents the motor
// E.g. BR for back right or I  for intake
//After the motor code, it prints the temperature of that motor
void display_hottest_motor(std::vector<pros::Motor> all_motors){
	int highest_motor_temp = all_motors[0].get_temperature();
	std::string hottest_motor = "";
	for (int i = 1; i < all_motors.size(); i++){			
		if (all_motors[i].get_temperature() > highest_motor_temp){
			highest_motor_temp = all_motors[i].get_temperature();
			//Using order of motors when list was created
			if (i == 0){
				hottest_motor = "BR";				
			}
			else if (i == 1){
				hottest_motor = "BL";
			}
			else if (i == 2){
				hottest_motor = "TL";
			}
			else if (i == 3){
				hottest_motor = "TR";
			}
			else if (i == 4){
				hottest_motor = "I ";
			}
			else if (i == 5){
				hottest_motor = "C1";
			}
			else{
				hottest_motor = "C2";
			}
		}
	}
	controller.print(0,0,"%s %.3F", hottest_motor, highest_motor_temp);
}

// Display all motor temperatures on brain screen
void display_all_motor_temps(std::vector<pros::Motor> all_motors) {
    // Display base motor temperatures
    for (int i = 0; i < all_motors.size(); i++) {
        // Check if the motor temperature is above 40 degrees Celsius
        if (all_motors[i].get_temperature() > 40) {
            pros::lcd::set_text_color(LV_COLOR_RED); // Set text color to red
        } else {
            pros::lcd::set_text_color(LV_COLOR_WHITE); // Set text color to white for other motors
        }

        // Define motor labels based on original motor naming convention
        std::string motorLabel = "";
        if (i == 0) {
            motorLabel = "BcLeft:  ";
        } else if (i == 1) {
            motorLabel = "BcRight: ";
        } else if (i == 2) {
            motorLabel = "TpLeft:  ";
        } else if (i == 3) {
            motorLabel = "TpRight: ";
        } else if (i == 4) {
            motorLabel = "Intake:  ";
        } else if (i == 5) {
            motorLabel = "Cata-1:  ";
        } else if (i == 6){
			motorLabel = "Cata-2:  ";
		} else if (i == 7){
			motorLabel = "FlyWheel:";
		}
		
		

        // Display the motor temperature with appropriate color
        pros::lcd::print(i, "%s%.3F", motorLabel.c_str(), all_motors[i].get_temperature());
    }

    // Reset text color to white for any additional text you want to display
    pros::lcd::set_text_color(LV_COLOR_WHITE);
}

void display_PTO_state(){
	if (!is_PTO_on_base.load()){
		controller.print(0, 0, "4-motor drive");
	}
	else{
		controller.print(0, 0, "6-motor drive");
	}
}

void switch_PTO_state(){
	while (true){
		if (isSwitchingPTO.load()){
			//Set the base motors to rotate forward
			left_motors.move_velocity(200);
			right_motors.move_velocity(200);

			//Wait for about 0.75s
			pros::delay(750);

			//Extend pistons in the correct way
			if (!is_PTO_on_base.load()){
				//Switch to 6 motor drive
				PTOpiston.set_value(1);
			}
			else{
				//Switch to 4 motor drive
				PTOpiston.set_value(0);
			}

			//Now restore driver control
			isSwitchingPTO = false;	
		}
		pros::delay(40);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	std::vector<pros::Motor> all_motors = 
	{bottomLeft,
	 bottomRight,
	 topLeft,
	 topRight,
	 intake,
	 catapultLeft,
	 catapultRight,
	 flywheel
	 };

	controller.clear_line(0);

	//Variables for double-binding
	bool isClawDown = false;
	bool isIntakeOff = true;
	int intakeState = 0; //0 = off; 1 = forward; -1 = reverse
	bool hangIsMoving = false;
	bool catapultIsMoving = false;
	bool flywheelIsMoving = false;

	//Wing variables
	int32_t wingTimer = 0;	
	bool solidWingsAreOpen = false;
	bool flappy_wingsAreOpen = false;

	//Start display task which will display both motor temperatures
	// and the current PTO state
	pros::Task display_task([=](){
		while(true){
			display_all_motor_temps(all_motors);
			display_PTO_state();
			pros::delay(500);
		}
	});

	pros::Task PTO_switching_task(switch_PTO_state);

	while(true){
		//Drive base code
		if (!isSwitchingPTO.load()){			
			if (controller.get_analog(ANALOG_LEFT_Y) > 8 || controller.get_analog(ANALOG_LEFT_Y) < -8){
				left_motors.move(controller.get_analog(ANALOG_LEFT_Y));

				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultLeft.move(controller.get_analog(ANALOG_LEFT_Y));
				}			
			}
			
			else
			{
				left_motors.brake();

				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultLeft.move(controller.get_analog(ANALOG_LEFT_Y));
				}	
			}
		
			//These if statements mean the robot will stop even if the controller position is stuck at say 1
			// (prevents robot moving if joystick gets stuck)
			if (controller.get_analog(ANALOG_RIGHT_Y) > 8 || controller.get_analog(ANALOG_RIGHT_Y) < -8){
				right_motors.move(controller.get_analog(ANALOG_RIGHT_Y));
				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultRight.move(controller.get_analog(ANALOG_RIGHT_Y));
				}
			}
			else{
				right_motors.brake();
				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultRight.brake();
				}
			}
		}


		//Toggle intake when Button R1 is pressed (make intake go forward)
		if (controller.get_digital_new_press(DIGITAL_R1)){
			if (intakeState != 1){
				//Forward intake
				intake.move(127);
				intakeState = 1;
			}
			else{
				//Reverse intake
				intake.move(-127);
				intakeState = -1;
			}
		}

		
		
		//Turn off intake when Button R2 is pressed (make intake reverse)
		if (controller.get_digital_new_press(DIGITAL_R2)){
			//Toggle intake between stop and reverse
			if (intakeState == 0){
				intake.move(-127);
				intakeState = -1;
			}
			else{
				intake.brake();
				intakeState = 0;
			}
		}

		//Wings
		if (controller.get_digital_new_press(DIGITAL_L1)){
			//Start a timer here
			wingTimer = pros::millis();
		}

		if (controller.get_digital(DIGITAL_L1)){
			//Check if the timer has been started
			if (wingTimer != 0){
				//Check how much time has elapsed
				if (pros::millis() - wingTimer > 750){
					//If the wings have been held down for a long time
					//Flappy wings
					flappy_wings.set_value(1);
					flappy_wingsAreOpen = true;					
				}				
			}
		}
		else{
			//If we have just released the wings button
			if (wingTimer != 0){
				if (flappy_wingsAreOpen){
					//Go to solid wings
					solidWingsAreOpen = true;
					flappy_wingsAreOpen = false;
					wings.set_value(1);
				}
				//The only way the flappy wings are closed is if 
				// the button was quickly pressed.
				else{
					//Normal toggle code
					if (solidWingsAreOpen){
						wings.set_value(0);
						solidWingsAreOpen = false;
					}
					else{
						wings.set_value(1);
						solidWingsAreOpen = true;
					}
				}
				wingTimer = 0; //Reset wing timer.
			}
		}
		

		//Flywheel
		if (controller.get_digital_new_press(DIGITAL_LEFT)){
			if (flywheelIsMoving){
				//Stop flywheel
				flywheel.brake();
				flywheelIsMoving = false;
			}
			else{
				//Make flywheel spin at full
				flywheel.move_voltage(12000);
				flywheelIsMoving = true;
			}
		}

		if (controller.get_digital_new_press(DIGITAL_UP)){
			if (flywheelIsMoving){
				//Stop flywheel
				flywheel.brake();
				flywheelIsMoving = false;
			}
			else{
				//Make flywheel spin at reverse
				flywheel.move_velocity(-600);
				flywheelIsMoving = true;
			}
		}  

		//Lift code
		if (controller.get_digital_new_press(DIGITAL_A)){
			if (is_PTO_on_base.load()){
				PTOpiston.set_value(1);
				is_PTO_on_base = false;
			}
			
			else{
				//Run lift code
				//For now it is just a vibration on controller
				if (hangIsMoving){
					catapult_motors.brake();
					hangIsMoving = false;
				}
				else{
					catapult_motors.move_voltage(12000);
					hangIsMoving = true;
				}
			}
		}

		if (controller.get_digital_new_press(DIGITAL_X)){
			if (is_PTO_on_base.load()){
				PTOpiston.set_value(1);
				is_PTO_on_base = false;
			}
			
			else{
				//Run lift code
				//For now it is just a vibration on controller
				if (hangIsMoving){
					catapult_motors.brake();
					hangIsMoving = false;
				}
				else{
					catapult_motors.move_voltage(-12000);
					hangIsMoving = true;
				}
			}
		}
		
		//Switch to 6 motor drive
		if (controller.get_analog(ANALOG_LEFT_X) <= -110
		&& controller.get_analog(ANALOG_RIGHT_X) <= -110)
		{
			if (!is_PTO_on_base){
				PTOpiston.set_value(0);
				is_PTO_on_base = true;
			}
			
		}			
		

		pros::delay(10); //Refresh rate of a motor
	}
}
