#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 127; // increase for stronger turns
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_forward_set(25.0, 0.0, 140.0);  // Forward drive constants — higher D for cleaner stop
  chassis.pid_drive_constants_backward_set(38.0, 0.0, 65.0);  // Backward — lower D to prevent tipping on deceleration
  chassis.pid_heading_constants_set(3.0, 0.0, 3.0);           // Low — jerk is mechanical not heading PID
  chassis.pid_turn_constants_set(4.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 300_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there

  // Reduce left-side current limit to counteract rightward jerk at end of drives.
  // Start at 3200 mA (vs 4000 on right). Lower = more correction, raise if robot starts curving left.
  for (auto& m : chassis.left_motors)
    m.set_current_limit(3200);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .


void right_4_3() {
  // Your autonomous code for the blue right starting position use 4/3 blocks put against the wall then push robot against it and parking zone
  intake.move(127);
  wing.set(true);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 75);
  chassis.pid_wait_quick_chain();
  match_mech.set(true);
  pros::delay(120);
  chassis.pid_drive_set(12_in, 100, true);  
  chassis.pid_wait();
  pros::delay(850);
  chassis.pid_drive_set(-1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, 50, true);
  chassis.pid_wait();
  intake.move(0);
  chassis.pid_drive_set(-36_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-17_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
  match_mech.set(false);
  intake.move(-127);
  intake.move(127);
  top_intake.move(127);
  pros::delay(2100);
  chassis.pid_drive_set(8.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  top_intake.move(0);
  chassis.pid_turn_set(10_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-8.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  wing.set(false);
  chassis.pid_turn_set(83_deg, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-28.0_in, 50, true);
  chassis.pid_wait();     
  wing.set(true);
  top_intake.move(0);
  chassis.pid_drive_set(3_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_set(137_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(120);
  wing.set(false);
  // chassis.pid_drive_set(10, DRIVE_SPEED, true);   
  // chassis.pid_wait_quick_chain();  
  // match_mech.set(true);
  // chassis.pid_turn_set(-130_deg, TURN_SPEED);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-2, DRIVE_SPEED, true);      
  // chassis.pid_wait_quick_chain();
  // match_mech.set(false);
  // pros::delay(100);
  // chassis.pid_drive_set(2, DRIVE_SPEED, true);      
  // chassis.pid_wait_quick_chain();
  // intake.move(-127);
  // pros::delay(1000);
}

void left_4_3() {
  // Mirror of right_4_3 for the left starting position — all turns negated
  intake.move(127);
  wing.set(true);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 75);
  chassis.pid_wait_quick_chain();
  match_mech.set(true);
  pros::delay(120);
  chassis.pid_drive_set(12_in, 100, true);
  chassis.pid_wait();
  pros::delay(850);
  chassis.pid_drive_set(-1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, 50, true);
  chassis.pid_wait();
  intake.move(0);
  chassis.pid_drive_set(-36_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-17_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait();
  match_mech.set(false);
  intake.move(-127);
  intake.move(127);
  top_intake.move(127);
  pros::delay(2100);
  chassis.pid_drive_set(8.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  top_intake.move(0);
  chassis.pid_turn_set(-10_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-8.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  wing.set(false);
  chassis.pid_turn_set(-83_deg, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-28.0_in, 50, true);
  chassis.pid_wait();
  wing.set(true);
  top_intake.move(0);
  chassis.pid_drive_set(3_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-137_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(120);
  wing.set(false);
}

// Minimal definitions for autons referenced from main.cpp
void test_Auto() {
  // Simple test autonomous: run the swing example
  swing_example();
}

void right_skills() {
  // Placeholder: use right_4_3 routine for now
  right_4_3();
}

void Left_Solo_OP() {
  // Placeholder left-side autonomous
  combining_movements();
}

void left_7_ball() {
  // Placeholder left 7-ball autonomous
  combining_movements();
}
// ═══════════════════════════════════════════════════════════════════
//  WALL CORRECTION — USAGE GUIDE
//
//  Replace:
//    chassis.pid_drive_set(X_in, speed, slew);
//    chassis.pid_wait();
//  With:
//    verified_drive(X, speed, slew);          // full wait + correction
//
//  Replace:
//    chassis.pid_drive_set(X_in, speed, slew);
//    chassis.pid_wait_quick_chain();
//  With:
//    verified_drive(X, speed, slew, true);    // chain — no correction
//
//  ── Example: right_4_3 with corrections ────────────────────────
//
//  void right_4_3_precise() {
//    intake.move(127);
//    wing.set(true);
//
//    // Forward wall is in view — dist_fwd verifies the move
//    verified_drive(28.5, DRIVE_SPEED, true, &dist_fwd);
//
//    chassis.pid_turn_set(90_deg, 75);
//    chassis.pid_wait_quick_chain();
//    match_mech.set(true);
//    pros::delay(120);
//
//    // Side wall now faces dist_side after the 90° turn
//    verified_drive(11.0, 100, true, &dist_side);
//    pros::delay(1000);
//    intake.move(0);
//
//    // Reversing — back wall is in view
//    verified_drive(-36.0, 50, true, &dist_back);
//
//    // No wall in sensor path here — nullptr skips verification
//    verified_drive(6.8, DRIVE_SPEED, true, nullptr);
//
//    // ... rest of routine ...
//  }
//
//  ── Tuning tip ──────────────────────────────────────────────────
//  Connect to the terminal.  [WC] lines print the before/after
//  readings and the exact correction applied every single drive.
//  This shows you exactly how much your PID was drifting per move.
// ═══════════════════════════════════════════════════════════════════

// chassis.pid_drive_set(19_in, DRIVE_SPEED,true);
// chassis.pid_wait_quick_chain();
// chassis.pid_turn_set(360_deg, 75);
// chassis.pid_wait_quick_chain();
// chassis.pid_drive_set(-7.9_in, DRIVE_SPEED,true);
// chassis.pid_wait_quick_chain();
// chassis.pid_turn_set(-136_deg, 75);
// chassis.pid_wait_quick_chain();;
// chassis.pid_drive_set(15_in, 50,true);
// chassis.pid_wait_quick_chain();
// match_mech.set(true);
// chassis.pid_drive_set(3_in, 50,true);
// chassis.pid_wait();
// pros::delay(100);
// pros::delay(1000);
void test_wall_correct() {
    verified_drive(24.0, DRIVE_SPEED, true);
}

// ─────────────────────────────────────────────────────────────────
//  Full test auton — run this to verify every system before a match
//  Watch the terminal ([TEST] lines) to see heading after each move.
//  Robot should end up back where it started facing 0°.
// ─────────────────────────────────────────────────────────────────
void full_test() {
    // ── 1. Mechanisms ────────────────────────────────────────────
    printf("[TEST] 1/5 — Mechanisms check\n");
    intake.move(127);
    top_intake.move(127);
    pros::delay(750);
    intake.move(0);
    top_intake.move(0);

    wing.set(true);
    pros::delay(400);
    wing.set(false);

    match_mech.set(true);
    pros::delay(400);
    match_mech.set(false);

    printf("[TEST] Mechanisms done — MEASURE: everything retracted? (2 sec)\n");
    pros::delay(2000);

    // ── 2. Drive forward 24" ─────────────────────────────────────
    printf("[TEST] 2/5 — Drive forward 24\"\n");
    chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
    printf("[TEST] Heading: %.2f deg — MEASURE: distance from start (3 sec)\n",
           chassis.odom_theta_get());
    pros::delay(3000);

    // ── 3. Turn right 90° ────────────────────────────────────────
    printf("[TEST] 3/5 — Turn to 90 deg\n");
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
    printf("[TEST] Heading: %.2f deg — MEASURE: is robot square? (3 sec)\n",
           chassis.odom_theta_get());
    pros::delay(3000);

    // ── 4. Turn back to 0° ───────────────────────────────────────
    printf("[TEST] 4/5 — Turn back to 0 deg\n");
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait();
    printf("[TEST] Heading: %.2f deg — MEASURE: facing original direction? (3 sec)\n",
           chassis.odom_theta_get());
    pros::delay(3000);

    // ── 5. Drive backward 24" ────────────────────────────────────
    printf("[TEST] 5/5 — Drive backward 24\"\n");
    chassis.pid_drive_set(-24_in, 70, true);
    chassis.pid_wait();
    printf("[TEST] Final heading: %.2f deg — MEASURE: back at start? (done)\n",
           chassis.odom_theta_get());
}

{ 
  
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void right_4_3_slow();

{
  intake.move(127);
  wing.set(true);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 75);
  chassis.pid_wait_quick_chain();
  match_mech.set(true);
  pros::delay(120);
  chassis.pid_drive_set(12_in, 100, true);  
  chassis.pid_wait();
  pros::delay(850);
  chassis.pid_drive_set(-1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in, 50, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, 50, true);
  chassis.pid_wait();
  intake.move(0);
  chassis.pid_drive_set(-36_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-17_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
  match_mech.set(false);
  intake.move(-127);
  intake.move(127);
  top_intake.move(127);
  pros::delay(2100);
  chassis.pid_drive_set(8.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  top_intake.move(0);
  chassis.pid_turn_set(10_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-8.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  wing.set(false);
  chassis.pid_turn_set(83_deg, 75);
  chassis.pid_wait();
  pros::delay(4000);   
  chassis.pid_drive_set(-28.0_in, 50, true);
  chassis.pid_wait();  
  wing.set(true);
  top_intake.move(0);
  chassis.pid_drive_set(3_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_set(137_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(120);
  wing.set(false);

}