#pragma once
#include "main.h"

// ═══════════════════════════════════════════════════════════════════
//  WALL CORRECTION SYSTEM — VEX Pushback
//
//  Sensors:
//    dist_fwd        — front distance sensor  (forward drive correction)
//    dist_side_left  — left  distance sensor  (available for auton use)
//    optical_fwd     — front optical          (orange gate for dist_fwd)
//    optical_left    — left  optical          (orange gate for dist_side_left)
//    vision_back     — rear Vision AI         (goal detection in autons,
//                                              NOT used for wall correction)
//
//  How verified_drive works:
//    1. Optical gate: if either optical detects orange (match loader balls,
//       hue 10–35°) → skip correction. All other colors pass through.
//    2. Snapshot dist_fwd before move (forward drives only — no rear sensor).
//    3. Run pid_drive_set + pid_wait normally.
//    4. Compare actual displacement to requested.
//    5. If error > 0.5" and < 3.0" → one corrective PID drive.
//       0.5" tolerance prevents back-and-forth oscillation.
// ═══════════════════════════════════════════════════════════════════


// ─────────────────────────────────────────────────────────────────
//  STEP 1: Fill in your smart port numbers
// ─────────────────────────────────────────────────────────────────
#define DIST_FWD_PORT          19   // distance sensor — FRONT
#define DIST_SIDE_LEFT_PORT    11   // distance sensor — LEFT SIDE
#define VISION_BACK_PORT       14   // Vision AI       — BACK (goal detection)
#define OPTICAL_FWD_PORT       20   // optical sensor  — FRONT
#define OPTICAL_LEFT_PORT      12   // optical sensor  — LEFT SIDE

// ─────────────────────────────────────────────────────────────────
//  STEP 2: Distance sensor validation thresholds
// ─────────────────────────────────────────────────────────────────
#define DIST_MIN_CONFIDENCE   35     // walls: ~40–63
#define DIST_MIN_OBJ_SIZE     120    // walls: ~200–400
#define DIST_MAX_RANGE_IN     110.0  // ignore readings beyond this

// ─────────────────────────────────────────────────────────────────
//  STEP 3: Correction bounds
// ─────────────────────────────────────────────────────────────────
// Errors below this are ignored — prevents oscillation chasing perfect.
#define MIN_CORRECTION_IN     0.5

// Errors above this mean the sensor is probably obstructed — skip.
#define MAX_CORRECTION_IN     3.0

// Speed (0-127) used for the post-drive heading straighten.
#define WALL_CORRECT_TURN_SPEED  105

// ─────────────────────────────────────────────────────────────────
//  STEP 4: Optical sensor — ORANGE block detection only
//
//  The optical sensors only block corrections when they see ORANGE
//  (match loader balls visible through the clear match loader).
//  All other colors (white walls, yellow stump, gray, nothing) let
//  the correction through — works on both field color variants.
//
//  Orange hue on VEX optical: ~10–35°, high saturation.
//  Tune by printing values in opcontrol while facing an orange piece:
//    printf("hue=%.1f sat=%.2f\n",
//           optical_left.get_hue(),
//           optical_left.get_saturation());
// ─────────────────────────────────────────────────────────────────
#define WALL_OPT_ORANGE_HUE_MIN  10.0   // orange starts around 10°
#define WALL_OPT_ORANGE_HUE_MAX  35.0   // orange ends around 35°
#define WALL_OPT_ORANGE_SAT_MIN   0.50  // must be well-saturated to count

// Minimum proximity reading to consider the optical sensor "seeing" something.
// Values range 0-255; tune if you get false positives/negatives.
#define WALL_OPT_DETECT_PROX       20

// ─────────────────────────────────────────────────────────────────
//  Sensor objects  (defined in wall_correct.cpp)
// ─────────────────────────────────────────────────────────────────
extern pros::Distance dist_fwd;
extern pros::Distance dist_side_left;
extern pros::Vision   vision_back;      // use in autons for goal detection
extern pros::Optical  optical_fwd;
extern pros::Optical  optical_left;


// ─────────────────────────────────────────────────────────────────
//  API
// ─────────────────────────────────────────────────────────────────

// Call once in initialize().
void wall_correct_init();

// Returns true if the distance sensor is reading a large flat object.
bool is_valid_wall_dist(pros::Distance& dist);

// Returns true if this optical sensor detects orange (match loader balls).
bool optical_sees_orange(pros::Optical& opt);

// Optical gate — returns false only if orange is detected.
// White walls, yellow stump, gray, or nothing → returns true (proceed).
// Orange detected on either sensor → returns false (skip correction).
bool optical_gate_ok();

// ── Core function ─────────────────────────────────────────────────
//
//  Drop-in replacement for:
//    chassis.pid_drive_set(X_in, speed, slew);
//    chassis.pid_wait();
//
//  Replace with:
//    verified_drive(X, speed, slew);
//
//  Forward drives  (+): verifies with dist_fwd, corrects if needed.
//  Backward drives (−): no rear sensor — runs PID only, no correction.
//  Tolerance: ±0.5" so one small error doesn't trigger another drive.
//
//  chain = false  → full pid_wait  + heading correction + distance correction
//  chain = true   → pid_wait_quick_chain (fast, no correction — for mid-auton)
//
//  EXAMPLES:
//    verified_drive(28.5,  DRIVE_SPEED, true);         // forward  28.5" with correction
//    verified_drive(-18.0, DRIVE_SPEED, true);         // backward 18" (PID only)
//    verified_drive(12.0,  DRIVE_SPEED, true, true);   // chain move — no correction
void verified_drive(double target_in,
                    int    speed,
                    bool   slew  = false,
                    bool   chain = false);

// ── Heading correction ────────────────────────────────────────────
//  Requires two side distance sensors (left + right) to detect skew.
//  Currently only dist_side_left is installed — this function is
//  commented out until a right side sensor is added.
//
// void wall_correct_heading(pros::Distance& dist_left,
//                            pros::Distance& dist_right,
//                            double          separation_in,
//                            double          expected_deg);
