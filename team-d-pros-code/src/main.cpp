#include "main.h"

#pragma region PORTS

#define ROLLER_MOTOR_PORT 6
#define RIGHT_DRIVE_1_PORT 2
#define RIGHT_DRIVE_2_PORT 3
#define LEFT_DRIVE_1_PORT 4
#define LEFT_DRIVE_2_PORT 5
#define FINGER_PORT 20
#define CONVEYOR_PORT 1
#define FLYWHEEL_PORT 12
#define INERTIAL_PORT 13
#define VISION_PORT 19

#pragma endregion PORTS

#pragma region VARIABLES
int autonomousMode = 5; // 1: THREE SQUARE, 2: TWO SQUARE
bool twoStickMode = true;
double maxSpeed = 0.9;

float minimumStick = 5;			 // Minimum output from controller sticks
bool refreshScreenEveryX = true; // If true, it will refresh every "nextScreenRef" x 20 milliseconds.
								 // If false, it will refresh every battery % change (Checked every "nextScreenRef")
double turningCap = 1;			 // 0 to 1

// Temporary Variables for motors
double fwdBackSpd = 0;
double turnSpd = 0;
double leftSpd = 0;
double rightSpd = 0;

// Math Function
int abs(int val)
{ // Convert integers to their absolute value
	return val < 0 ? -val : val;
}
double absDouble(double val){ // Convert integers to their absolute value
  return val < 0 ? -val : val;
}

bool conveyorOn = false; // Default to conveyor OFF, boolean that controls whether the conveyor is running
bool flywheelOn = false; // Default to flywheel OFF, boolean that controls whether the flywheeel is running
bool conveyorReversed = false;

int fingerMode = 0; // Finger modes:
/**
0 - Finger is not in motion
1 - Finger is moving to push the flywheel
2 - Finger is moving back to it's original position
*/

// Screen related
int screenRefCount = 300; // Counter that counts up to the nextScreenRef amount
int nextScreenRef = 200;  // The amount of 20 msec intervals to the next refresh

int lastBatteryAmt = 0; // Holds the value of the battery without needing to check the battery each time
int screenInformationMode = 3;
/*
0 - Battery (JUICE) percentage
1 - Movement Motor (TRLR-L, TRLR-R) TEMPS
2 - Conveyor Motor (MIX) TEMPS + Flywheel Motor (TST) TEMPS
3 - Pusher Motor (FINGER) TEMPS + Roller Motor (ROLL) temps

*/

// Button related
bool conveyorButtonPressed = false;
bool fingerButtonPressed = false;
bool flywheelButtonPressed = false;

bool flywheelAdjustUpPressed = false;
bool flywheelAdjustDownPressed = false;

bool conveyorUnReverseButtonPressed = false;
bool conveyorReverseButtonPressed = false;

int flywheelAdjustedSpeed = 60;

#pragma endregion VARIABLES

#pragma region SETUP_PORTS
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_drive1(LEFT_DRIVE_1_PORT);
pros::Motor left_drive2(LEFT_DRIVE_2_PORT);
pros::Motor right_drive1(RIGHT_DRIVE_1_PORT);
pros::Motor right_drive2(RIGHT_DRIVE_2_PORT);
pros::Motor_Group left_drive({left_drive1, left_drive2});
pros::Motor_Group right_drive({right_drive1, right_drive2});

pros::Motor finger_motor(FINGER_PORT);
pros::Motor conveyor_motor(CONVEYOR_PORT);
pros::Motor flywheel_motor(FLYWHEEL_PORT);
pros::Motor roller_motor(ROLLER_MOTOR_PORT);
pros::Imu inertial_motor(INERTIAL_PORT);

pros::Vision vision(VISION_PORT, pros::E_VISION_ZERO_CENTER);
#pragma endregion SETUP_PORTS

#pragma region SCREEN_FUNCTIONS

/** SCREEN FUNCTIONS **/
void refreshScreen(bool updateRow1, bool updateRow2, bool updateRow3)
{

	// Clear screen
	// Controller1.Screen.clearScreen();

	if (updateRow1)
	{
		// Set cursor to row 1, column 1
		master.clear_line(0);

		// Print whether or not the conveyor is on or off
		if (conveyorReversed)
		{
			master.print(0, 0, "R-MIX");
			master.print(0, 5, conveyorOn ? "ING | " : "ED | ");

			master.print(0, 11, "TOAST");
			master.print(0, 16, flywheelOn ? "ING" : "ED");
		}
		else
		{
			master.print(0, 0, "MIX");
			master.print(0, 3, conveyorOn ? "ING | " : "ED | ");

			master.print(0, 9, "TOAST");
			master.print(0, 14, flywheelOn ? "ING" : "ED");
		}
	}

	if (updateRow2)
	{
		// Set cursor to row 2, column 1
		master.clear_line(1);

		// Print brain battery amount
		if (screenInformationMode == 0)
		{
			master.print(1, 0, "JUICE: %d", lastBatteryAmt);
		}
		else if (screenInformationMode == 1)
		{
			master.print(1, 0, "TRLR-L: %.0f", left_drive1.get_temperature());
			master.print(1, 10, "TRLR-R: %.0f", right_drive1.get_temperature());
		}
		else if (screenInformationMode == 2)
		{
			master.print(1, 0, "MIXER: %.0f", conveyor_motor.get_temperature());
			master.print(1, 10, "TOAST: %.0f", flywheel_motor.get_temperature());
		}
		else if (screenInformationMode == 3)
		{
			master.print(1, 0, "FINGER: %.0f", finger_motor.get_temperature());
			master.print(1, 10, "ROLL: %.0f", roller_motor.get_temperature());
		}
	}

	if (updateRow3)
	{
		// Set cursor to row 3, column 1
		master.clear_line(2);

		master.print(2, 0, "FINGER:");
		master.print(2, 7, fingerMode == 0 ? "LAZY" : "BUSY");
	}
}

void setupScreen()
{
	// Clear screen in-case something happens to be on it somehow
	master.clear();

	// Set-up battery capacity
	lastBatteryAmt = int(pros::battery::get_capacity);

	// Refresh screen
	refreshScreen(true, true, true);
}

#pragma endregion SCREEN_FUNCTIONS

#pragma region AUTONOMOUS_FUNCTIONS


void MovePID(double feet, int speed){ // Input in feet, speed in percent 0-100
  //input is feet, converts to inches, gets radians with arc length, converts to degrees
  float targetAngle = ((feet * 12) / 2.125) * (180/3.14159);
  right_drive.tare_position();
  left_drive.tare_position();

  float Kp = 0;
  float Ki = 0;
  float Kd = 0;

  float error = 0;
  float lastError = 0;
  float integral = 0;
  float derivative = 0;

  float errors[480];
  int counter = 0;

	pros::screen::erase();
	pros::screen::set_pen(COLOR_RED);
	pros::screen::draw_line(0, 120, 479, 120);
	pros::screen::set_pen(COLOR_WHITE);

  float targetSpeed = 0;

  // For some reason I thought PID was always for distance, turns out it apparently was only used for correcting motors
  std::vector<double> left_positions = left_drive.get_positions();
  std::vector<double> right_positions = right_drive.get_positions();
  while(left_positions[0] < targetAngle){
	left_positions = left_drive.get_positions();
	right_positions = right_drive.get_positions();
    targetSpeed = targetAngle - left_positions[0];
    if(targetSpeed > speed){
      targetSpeed = speed;
    }
    if(targetSpeed < -speed){
      targetSpeed = -speed;
    }
    if(targetSpeed < 5 && targetSpeed > -5){
      if(targetSpeed < 0){
        targetSpeed = -5;
      }
      else{
        targetSpeed = 5;
      }
    }

    error = left_positions[0] - right_positions[0];

    integral = integral + error;
    derivative = error - lastError;

	left_drive.move_velocity(targetSpeed);
	right_drive.move_velocity(targetSpeed + (error * Kp) + (integral * Ki) + (derivative * Kd));
	
    lastError = error + 0;

    errors[counter] = error;
	pros::screen::set_pen(COLOR_WHITE);
	pros::screen::draw_pixel(counter, 120+(error*5));
	pros::screen::set_pen(COLOR_BLUE);
	pros::screen::draw_pixel(counter, 120+(integral));
	pros::screen::set_pen(COLOR_GREEN);
	pros::screen::draw_pixel(counter, 120+(derivative*5));
	
    counter ++;
    if(counter >= 480){
      counter = 479;
    }

    counter ++;
    pros::delay(20);
  }

  left_drive.brake();
  right_drive.brake();
}

void TurnA(int angle){ // Accepts any angle from 0 to 359.99, based on clockwise from starting position
  double turnToAngle = angle;
  if(turnToAngle >= 360){
    turnToAngle -= 360;
  }
  else if(turnToAngle < 0){
    turnToAngle += 360;
  }
  

  double turnSpeed = 0;
  double maxSpeed = 50;

  double InertialPlus = 0;
  double difference = turnToAngle - inertial_motor.get_heading();

  
  int counter = 0;
  while(counter < 20){
    difference = turnToAngle - (inertial_motor.get_heading() + InertialPlus);
    if(absDouble(difference - 360) < absDouble(difference)){
      InertialPlus += 360;
      difference = turnToAngle - (inertial_motor.get_heading() + InertialPlus);
    }
    if(absDouble(difference + 360) < absDouble(difference)){
      InertialPlus -= 360;
      difference = turnToAngle - (inertial_motor.get_heading() + InertialPlus);
    }

    turnSpeed = absDouble(difference) < 2 ? (difference < 0 ? -2 : 2) : difference;
    turnSpeed = turnSpeed > maxSpeed ? maxSpeed : (turnSpeed < -maxSpeed ? -maxSpeed : turnSpeed);
    
	right_drive.move_velocity(-turnSpeed);
	left_drive.move_velocity(turnSpeed);

    if(absDouble(difference) < 0.5){
      counter ++;
    }
    else{
      counter = 0;
    }

    pros::delay(20);
  }

  left_drive.brake();
  right_drive.brake();
}

void TurnI(int angle){
  double turnToAngle = 180 + angle;
  if(turnToAngle >= 360){
    turnToAngle -= 360;
  }
  else if(turnToAngle < 0){
    turnToAngle += 360;
  }
  inertial_motor.set_heading(180);

  double turnSpeed = 0;
  double maxSpeed = 40;
  double difference = turnToAngle - inertial_motor.get_heading();

  int counter = 0;
  while(counter < 20){
    difference = turnToAngle - inertial_motor.get_heading();
    turnSpeed = absDouble(difference) < 2 ? (difference < 0 ? -2 : 2) : difference;
    turnSpeed = turnSpeed > maxSpeed ? maxSpeed : (turnSpeed < -maxSpeed ? -maxSpeed : turnSpeed);
    
	right_drive.move_velocity(-turnSpeed);
	left_drive.move_velocity(turnSpeed);

    if(absDouble(difference) < 0.5){
      counter ++;
    }
    else{
      counter = 0;
    }

    pros::delay(20);
  }

  left_drive.brake();
  right_drive.brake();
}

void Move(int feet, int speed)
{ // Input in feet, speed in percent 0-100
	// input is feet, converts to inches, gets radians with arc length, converts to degrees
	int angle = int(((feet * 12) / 2.125) * (180 / 3.14159));

	left_drive.tare_position();
	right_drive.tare_position();

	right_drive.move_absolute(angle, speed * 2);
	left_drive.move_absolute(angle, speed * 2);

	int deadangle = 1;
	while (left_drive.get_positions()[0] < (angle - deadangle) || left_drive.get_positions()[0] > (angle + deadangle) || right_drive.get_positions()[0] < (angle - deadangle) || right_drive.get_positions()[0] > (angle + deadangle))
	{
		pros::delay(5);
	}
}

void Turn(int angle, int speed)
{ // Positive angle spins clockwise?
	int angleAdjust = int(-angle * 3.6);

	left_drive.tare_position();
	right_drive.tare_position();

	right_drive.move_relative(angleAdjust, speed * 2);
	left_drive.move_relative(-angleAdjust, speed * 2);

	int deadangle = 1;
	while (left_drive.get_positions()[0] < (-angleAdjust - deadangle) || left_drive.get_positions()[0] > (-angleAdjust + deadangle) || right_drive.get_positions()[0] < (angleAdjust - deadangle) || right_drive.get_positions()[0] > (angleAdjust + deadangle))
	{
		pros::delay(5);
	}
}

void AdjustRoller(float angleAmount)
{ // Spins roller at 50 speed for X seconds
	right_drive.move_velocity(-20);
	left_drive.move_velocity(-20);
	pros::delay(300);
	roller_motor.tare_position();
	roller_motor.move_absolute(angleAmount, -200);
	int deadangle = 1;
	while (roller_motor.get_position() < angleAmount - deadangle || roller_motor.get_position() > angleAmount + deadangle)
	{
		pros::delay(5);
	}
	left_drive.move_velocity(0);
	right_drive.move_velocity(0);
}

void AdjustFlywheel(int speed)
{ // Flywheel motor will spin clockwise with a positive speed
	flywheel_motor.move_velocity(speed * 2);
}

void AdjustConveyor(int speed)
{ // Conveyor motor will spin clockwise with a positive speed
	conveyor_motor.move_velocity(speed * 2);
}

void FingerActivate()
{ // FINGER ACTIVATE
	finger_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	finger_motor.tare_position();
	// FingerMotor.setPosition(0, turns);

	finger_motor.move_velocity(200);
	while (finger_motor.get_position() < 15)
	{
		pros::delay(20);
	}
	finger_motor.move_velocity(-200);
	while (finger_motor.get_position() >= 0)
	{
		pros::delay(20);
	}
	finger_motor.move_velocity(0);
	//pros::delay(300);
}

#pragma endregion AUTONOMOUS_FUNCTIONS

#pragma region RC_FUNCTIONS

void updateConveyor()
{ // To toggle the conveyor

	// Update conveyor status in-function to prevent wasted computations
	if (conveyorOn)
	{
		// Set conveyor motor to full power (Because why wouldn't you)
		conveyor_motor.move_velocity(-200);
		if (conveyorReversed)
		{
			conveyor_motor.move_velocity(200);
		}
	}
	else
	{
		// Stop conveyor motor
		conveyor_motor.move_velocity(0);
	}

	// Update screen to refresh whether or not "Conveyor: " says "ON" or "OFF"
	refreshScreen(true, false, false);
}

void toggleConveyor()
{ // To toggle the conveyor
	// Update the bool controlling whether or not the conveyor is on
	conveyorOn = !conveyorOn;

	updateConveyor();
}

void toggleFlywheel(int thespeed)
{ // To toggle the flywheel
	// Update the bool controlling whether or not the conveyor is on
	flywheelOn = !flywheelOn;

	// Update conveyor status in-function to prevent wasted computations
	if (flywheelOn)
	{
		// Set conveyor motor to full power (Because why wouldn't you)
		flywheel_motor.move_velocity(200);
	}
	else
	{
		// Stop conveyor motor
		flywheel_motor.move_velocity(0);
	}

	// Update screen to refresh whether or not "Conveyor: " says "ON" or "OFF"
	refreshScreen(true, false, false);
}

void updateFinger()
{
	finger_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	finger_motor.tare_position();
	if (fingerMode == 1)
	{ // FINGER OUT!
		finger_motor.move_velocity(100);
		if (finger_motor.get_position() >= 10)
		{
			fingerMode = 2;
		}
	}
	else if (fingerMode == 2)
	{ // FINGER RETURN!
		finger_motor.move_velocity(-100);
		if (finger_motor.get_position() <= 0)
		{
			finger_motor.move_velocity(0);
			fingerMode = 0;
			//refreshScreen(false, false, true);
		}
	}
}

void updateFlywheelSpeed(int speedUpdate)
{
	flywheelAdjustedSpeed += speedUpdate;
	if (flywheelAdjustedSpeed > 100)
	{
		flywheelAdjustedSpeed = 100;
	}
	if (flywheelAdjustedSpeed < 5)
	{
		flywheelAdjustedSpeed = 5;
	}
	if (flywheelOn)
	{
		// Set conveyor motor to full power (Because why wouldn't you)
		flywheel_motor.move_velocity(flywheelAdjustedSpeed*2);
	}
	else
	{
		// Stop conveyor motor
		flywheel_motor.move_velocity(0);
	}

	master.clear();
	master.print(0, 0, "TOAST SPEED: %d", flywheelAdjustedSpeed);

	screenRefCount = 0;
	nextScreenRef = 100;
}

#pragma endregion RC_FUNCTIONS

#pragma region OTHER_FUNCTIONS
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_drive1(LEFT_DRIVE_1_PORT, pros::E_MOTOR_GEAR_GREEN, false);
	pros::Motor left_drive2(LEFT_DRIVE_2_PORT, pros::E_MOTOR_GEAR_GREEN, false);
	pros::Motor right_drive1(RIGHT_DRIVE_1_PORT, pros::E_MOTOR_GEAR_GREEN, true);
	pros::Motor right_drive2(RIGHT_DRIVE_2_PORT, pros::E_MOTOR_GEAR_GREEN, true);
	pros::Motor_Group left_drive({left_drive1, left_drive2});
	pros::Motor_Group right_drive({right_drive1, right_drive2});

	pros::Motor finger_motor(FINGER_PORT, pros::E_MOTOR_GEAR_GREEN, false);
	pros::Motor conveyor_motor(CONVEYOR_PORT, pros::E_MOTOR_GEAR_GREEN, false);
	pros::Motor flywheel_motor(FLYWHEEL_PORT, pros::E_MOTOR_GEAR_GREEN, false);
	pros::Motor roller_motor(ROLLER_MOTOR_PORT, pros::E_MOTOR_GEAR_GREEN, true);
	pros::Imu inertial_motor(INERTIAL_PORT);
	pros::Vision vision(VISION_PORT, pros::E_VISION_ZERO_CENTER);
	// pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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

#pragma endregion OTHER_FUNCTIONS

#pragma region AUTONOMOUS

void StartAutonomous(int mode)
{	
	// All autonomous actions should happen here
	// LeftDriveMotors.setTimeout(2, seconds);
	// RightDriveMotors.setTimeout(2, seconds);

	finger_motor.tare_position();

	if (mode == 1)
	{ // Two-Square Start Plan (TWO DISKS SHOT)
		AdjustFlywheel(71);

		// Two square start
		Move(0.75, 40);
		Turn(90, 40);
		Move(2.1, 40);
		Turn(-85, 40);

		pros::delay(1000);

		FingerActivate();
		FingerActivate();

		// Roll roller
		Move(-0.45, 40);
		AdjustRoller(90);

		AdjustFlywheel(0);
		AdjustConveyor(0);
	}
	if (mode == 2)
	{ // Two-Square Boosted Plan (FIVE DISKS SHOT)
		AdjustFlywheel(71);

		// Two square start
		Move(0.75, 40);
		Turn(90, 40);
		Move(2.1, 40);
		Turn(-85, 40);

		pros::delay(1000);

		FingerActivate();
		FingerActivate();

		// Roll roller
		Move(-0.45, 40);
		AdjustRoller(90);

		// Move towards other discs and pick them up
		Move(0.15, 40);
		Turn(90, 40);
		Move(-0.25, 40);
		Turn(45, 40);
		AdjustConveyor(100);
		Move(-5.65, 50);

		// Turn towards goal and shoot again
		AdjustFlywheel(60);
		Turn(-90, 40);
		FingerActivate();
		FingerActivate();
		FingerActivate();

		AdjustFlywheel(0);
		AdjustConveyor(0);
	}
	if (mode == 3)
	{ // Three-Square Start Plan (TWO DISKS SHOT)
		Move(-0.1, 40);
		AdjustRoller(90);
		AdjustFlywheel(69);
		pros::delay(4000);
		FingerActivate();
		pros::delay(1000);
		FingerActivate();
		AdjustFlywheel(0);
	}
	if (mode == 4)
	{ // Skills Plan (TEN DISKS SHOT)
		// Move backwards into roller slowly and spin roller
		Move(-0.1, 40);
		AdjustRoller(180);
		AdjustFlywheel(69);

		// Pick up disc that is on autonomous line
		Move(1.5, 40);
		Turn(135, 40);
		AdjustConveyor(100);
		Move(-1.5, 40);
		Turn(-45, 40);

		// Move backwards into 2nd roller slowly and spin roller
		AdjustConveyor(0);
		Move(-1.5, 40);
		AdjustRoller(180);

		// Move forwards slightly and shoot three disks into the blue basket
		Move(0.5, 40);
		FingerActivate();
		FingerActivate();
		FingerActivate();

		// Progress towards picking up next three discs
		Turn(90, 40);
		Move(-1, 40);
		Turn(45, 40);
		AdjustConveyor(100);
		AdjustFlywheel(40);

		// Pick up three disks
		Move(-5.65, 40);

		// Turn towards red basket and shoot three disks into there
		Turn(90, 40);
		AdjustConveyor(0);
		FingerActivate();
		FingerActivate();
		FingerActivate();

		// Move backwards into next set of three discs
		AdjustConveyor(100);
		Move(-2.82, 40);
		Turn(-90, 40);
		Move(-5.65, 40);

		// Move towards next roller
		Turn(-45, 40);
		Move(-0.75, 40);
		Turn(90, 40);

		// Move backwards into 3rd roller slowly and spin roller
		AdjustConveyor(0);
		AdjustFlywheel(69);
		Move(-1.5, 40);
		AdjustRoller(180);

		// Move forwards slightly and shoot three disks into the red basket
		Move(1, 40);
		FingerActivate();
		FingerActivate();
		FingerActivate();

		// Move towards final disc and shoot it
		AdjustConveyor(100);
		Turn(-135, 40);
		Move(-1.5, 40);
		Turn(135, 40);
		FingerActivate();
		AdjustFlywheel(0);
		AdjustConveyor(0);
		Turn(-90, 40);

		// Move backwards into final roller slowly and spin roller
		Move(-1.5, 40);
		AdjustRoller(180);
	}
}

void autonomous()
{
	master.clear();
	master.print(0, 0, "COOKING STARTED");

	StartAutonomous(autonomousMode);

	master.clear();
	master.print(0, 0, "COOKING FINISHED");
}

#pragma endregion AUTONOMOUS

#pragma region RC
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
void opcontrol()
{
	// User control code here, inside the loop
	setupScreen();
	while (1)
	{
		int turnAxis = twoStickMode ? master.get_analog(ANALOG_RIGHT_X) : master.get_analog(ANALOG_LEFT_X);
		// Left/Right Motors
		if (abs(master.get_analog(ANALOG_LEFT_Y)) + abs(turnAxis) >= minimumStick)
		{
			// Only move if controller sticks are more than the deadband (In case of controller drift)
			// Use dual stick mode, easier to go straight or control turns if necessary
			fwdBackSpd = master.get_analog(ANALOG_LEFT_Y);
			turnSpd = -turnAxis * 0.85;

			leftSpd = fwdBackSpd - turnSpd;
			rightSpd = fwdBackSpd + turnSpd;

			leftSpd *= maxSpeed;
			rightSpd *= maxSpeed;

			// Set velocities of motors
			left_drive.move_velocity(leftSpd * 2);
			right_drive.move_velocity(rightSpd * 2);
		}
		else
		{
			// Stop motors
			left_drive.move_velocity(0);
			right_drive.move_velocity(0);
		}

		// Conveyor - Run the "toggleConveyor" function every time the A button is pressed
		if (!conveyorButtonPressed && master.get_digital(DIGITAL_A))
		{
			toggleConveyor();
			conveyorButtonPressed = true;
		}
		if (conveyorButtonPressed && !master.get_digital(DIGITAL_A))
		{
			conveyorButtonPressed = false;
		}

		// Conveyor - Update whether conveyor is reversed or not
		if (!conveyorReverseButtonPressed && master.get_digital(DIGITAL_LEFT))
		{
			conveyorReversed = true;
			updateConveyor();
			conveyorReverseButtonPressed = true;
		}
		if (!conveyorUnReverseButtonPressed && master.get_digital(DIGITAL_RIGHT))
		{
			conveyorReversed = false;
			updateConveyor();
			conveyorUnReverseButtonPressed = true;
		}

		if (conveyorReverseButtonPressed && !master.get_digital(DIGITAL_LEFT))
		{
			conveyorReverseButtonPressed = false;
		}
		if (conveyorUnReverseButtonPressed && !master.get_digital(DIGITAL_RIGHT))
		{
			conveyorUnReverseButtonPressed = false;
		}

		// Flywheel - Run the "toggleFlywheel" function every time the A button is pressed
		if (!flywheelButtonPressed && master.get_digital(DIGITAL_X))
		{
			toggleFlywheel(flywheelAdjustedSpeed);
			flywheelButtonPressed = true;
		}
		if (flywheelButtonPressed && !master.get_digital(DIGITAL_X))
		{
			flywheelButtonPressed = false;
		}

		// Flywheel - increase speed when the UP button is pressed
		if (!flywheelAdjustUpPressed && master.get_digital(DIGITAL_UP))
		{
			updateFlywheelSpeed(5);
			flywheelAdjustUpPressed = true;
		}
		if (flywheelAdjustUpPressed && !master.get_digital(DIGITAL_UP))
		{
			flywheelAdjustUpPressed = false;
		}

		// Flywheel - decrease speed when the DOWN button is pressed
		if (!flywheelAdjustDownPressed && master.get_digital(DIGITAL_DOWN))
		{
			updateFlywheelSpeed(-5);
			flywheelAdjustDownPressed = true;
		}
		if (flywheelAdjustDownPressed && !master.get_digital(DIGITAL_DOWN))
		{
			flywheelAdjustDownPressed = false;
		}

		// Roller - roll based on L1 and R1
		if (master.get_digital(DIGITAL_L1))
		{
			roller_motor.move_velocity(-200);
		}
		else if (master.get_digital(DIGITAL_R1))
		{
			roller_motor.move_velocity(200);
		}
		else
		{
			roller_motor.move_velocity(0);
		}

		// Finger - Update "fingerMode" to 1 to start finger sequence
		if (master.get_digital(DIGITAL_Y))
		{
			if (fingerMode == 0 && flywheel_motor.get_actual_velocity() > (flywheelAdjustedSpeed * 2) - 2 && flywheel_motor.get_actual_velocity() < (flywheelAdjustedSpeed * 2) + 2)
			{ // Only start finger sequence when finger sequence is not running
				fingerMode = 1;
				refreshScreen(false, false, true);
			}
			fingerButtonPressed = true;
		}
		if (fingerButtonPressed && !master.get_digital(DIGITAL_Y))
		{
			fingerButtonPressed = false;
      		refreshScreen(false, false, true);
		}
		

		pros::screen::erase();
		pros::screen::print(pros::E_TEXT_MEDIUM, 2, "VELOCITY: ", flywheel_motor.get_actual_velocity());
    	updateFinger();

		// Expansion
		if (master.get_digital(DIGITAL_B))
		{
			// Pneumatic.set(true);
		}

		// Screen Refresh every x msec (Not every check unless you want to REALLY lag the brain/controller)
		screenRefCount += 1;
		if (screenRefCount >= nextScreenRef)
		{						// Have we reached the next screen refresh?
			screenRefCount = 0; // Reset counter

			if (nextScreenRef == 100)
			{
				nextScreenRef = 200;
				refreshScreen(true, true, true);				// Run refresh screen function
				lastBatteryAmt = pros::battery::get_capacity(); // Update "lastBatteryAmt" to remember what the battery % was last
			}
			else
			{
				screenInformationMode++;
				if (screenInformationMode > 3)
				{
					screenInformationMode = 0;
				}
				if (refreshScreenEveryX)
				{													// If the mode is to refresh every x amount of times
					refreshScreen(false, true, false);				// Run refresh screen function
					lastBatteryAmt = pros::battery::get_capacity(); // Update "lastBatteryAmt" to remember what the battery % was last
				}
				else
				{ // If the mode is to refresh if the battery percentage changes
					if (lastBatteryAmt != pros::battery::get_capacity())
					{													// Check if battery % changed from last refresh
						refreshScreen(false, true, false);				// Refresh screen
						lastBatteryAmt = pros::battery::get_capacity(); // Update "lastBatteryAmt" to remember what the battery % was last
					}
				}
			}
		}
		//

		// Wait
		pros::delay(20);
	}
}

#pragma endregion RC

// :)