#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-11, -12, -13},     // Left Chassis Ports (negative port will reverse it!)
    {18, 19, 20},  // Right Chassis Ports (negative port will reverse it!)

    14,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  // Set wrist motor to hold so it does not coast and jitter when PID output is small
  Wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  liftPID.exit_condition_set(80, 3, 250, 7, 500, 500);
  armPID.exit_condition_set(80, 3, 250, 7, 500, 500);

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}




void wristTask(){
  pros::delay(2000);
  // Initialize PID target to current wrist position to avoid sudden movement
  armPID.target_set(rARM.get_position());
  while(true){
      double current = rARM.get_position();
      double target = armPID.target_get();
      double err = target - current;
      double pid_out = armPID.compute(current);
      if (std::abs(err) < 200.0) {
        // Hold the wrist near the target with a small torque instead of setting zero.
        pid_out = err > 0 ? 12 : (err < 0 ? -12 : 0);
      }
      // Clamp PID output to motor move range (-127 to 127)
      if (pid_out > 127) pid_out = 127;
      if (pid_out < -127) pid_out = -127;
      setARM((int)pid_out);

      pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Wrist_Task(wristTask);




void liftTask(){
  pros::delay(2000);
  // Initialize PID target to current lift position to avoid sudden movement
  liftPID.target_set(LiftR.get_position());
  while(true){
      double pid_out = liftPID.compute(LiftR.get_position());
      double target = liftPID.target_get();
      double current = LiftR.get_position();
      // For endpoint targets, force the PID output to match the needed direction
      if (std::abs(target - 0.0) < 1.0 || std::abs(target - 24120.0) < 50.0) {
        double err = target - current; // positive => need to go up, negative => need to go down
        if (std::abs(err) < 5.0) {
          pid_out = 0; // close enough
        } else {
          // Ensure pid_out sign matches desired direction
          pid_out = std::fabs(pid_out) * (err > 0 ? 1.0 : -1.0);
        }
      }
      // Clamp PID output to motor move range (-127 to 127)
      if (pid_out > 127) pid_out = 127;
      if (pid_out < -127) pid_out = -127;
      setLift((int)pid_out);

      pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Lift_Task(liftTask);




void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  LiftR.tare_position();  // Reset lift sensor to 0

  rARM.tare_position(); // Reset wrist sensor to 0

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}


void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);


void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    static bool auton_test_triggered = false;

    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Manual autonomous test trigger. This fires only once when B + Down are pressed,
    // then resets when the buttons are released so it cannot freeze the robot loop.
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      if (!auton_test_triggered) {
        auton_test_triggered = true;
        pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
        autonomous();
        chassis.drive_brake_set(preference);
      }
    } else {
      auton_test_triggered = false;
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}


int countController = 0;

void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);




  while (true) {
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
        // Sensor max is ~0.67 rotations. pros::Rotation returns centi-degrees,
        // so 0.67 rotations -> 0.67 * 360 deg * 100 = ~24120 centidegrees

      if (master.get_digital(DIGITAL_UP)) {
        liftPID.target_set(-0.5);
      
      } else if (master.get_digital(DIGITAL_RIGHT)) {
        liftPID.target_set(1600);
      
      } else if (master.get_digital(DIGITAL_DOWN)) {
        liftPID.target_set(2600);

      } else if (master.get_digital(DIGITAL_LEFT)) {
        liftPID.target_set(3600);

      } else if (master.get_digital(DIGITAL_X)) {
        liftPID.target_set(5600);

      } else if (master.get_digital(DIGITAL_A)) {
        liftPID.target_set(7000);

      } else if (master.get_digital(DIGITAL_B)) {
        liftPID.target_set(8000);
      }


      if (master.get_digital(DIGITAL_R1)) {
        armPID.target_set(1950);  // 1.75 rotations = 1.75 * 360 deg * 100 = 63000 centidegrees

      } else if (master.get_digital(DIGITAL_R2)) {
        armPID.target_set(2575);  // 2.35 rotations = 2.35 * 360 deg * 100 = 84600 centidegrees

      } else if (master.get_digital(DIGITAL_L1)) {
        armPID.target_set(0);  // 0 rotations = 0 * 360 deg * 100 = 0 centidegrees

      } 


      if(!(countController % 25)){
        master.print(0,0, "%lf", LiftR.get_position());
      }
      countController++;

        Claw.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));


    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }

}