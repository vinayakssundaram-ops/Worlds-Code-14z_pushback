#include "main.h"
#include "wall_correct.hpp"
#include <cmath>

// ─────────────────────────────────────────────────────────────────
//  Sensor definitions
// ─────────────────────────────────────────────────────────────────
pros::Distance dist_fwd        (DIST_FWD_PORT);
pros::Distance dist_side_left  (DIST_SIDE_LEFT_PORT);
pros::Vision   vision_back     (VISION_BACK_PORT);
pros::Optical  optical_fwd     (OPTICAL_FWD_PORT);
pros::Optical  optical_left    (OPTICAL_LEFT_PORT);

// ─────────────────────────────────────────────────────────────────
//  Init
// ─────────────────────────────────────────────────────────────────
void wall_correct_init() {
    optical_fwd.set_led_pwm(100);
    optical_left.set_led_pwm(100);
    pros::delay(150);
    printf("[WC] initialized — fwd opt=%s  left opt=%s\n",
           optical_fwd.is_installed()  ? "ok" : "missing",
           optical_left.is_installed() ? "ok" : "missing");
}

// ─────────────────────────────────────────────────────────────────
//  Distance sensor validation
// ─────────────────────────────────────────────────────────────────
bool is_valid_wall_dist(pros::Distance& dist) {
    if (!dist.is_installed()) return false;

    int32_t d    = dist.get();
    int32_t conf = dist.get_confidence();
    int32_t size = dist.get_object_size();

    if (d <= 0 || d == PROS_ERR)              return false;
    if ((double)d / 25.4 > DIST_MAX_RANGE_IN) return false;
    if (conf < DIST_MIN_CONFIDENCE)            return false;
    if (size < DIST_MIN_OBJ_SIZE)             return false;
    return true;
}

// ─────────────────────────────────────────────────────────────────
//  Optical: orange detection
//  Orange match loader balls: hue ~10–35°, high saturation.
//  Works on both field color variants — only orange is blocked,
//  not white walls, yellow stump, or any other color.
// ─────────────────────────────────────────────────────────────────
bool optical_sees_orange(pros::Optical& opt) {
    if (!opt.is_installed()) return false;
    int32_t prox = opt.get_proximity();
    if (prox == PROS_ERR) return false;
    if (prox < WALL_OPT_DETECT_PROX) return false;

    double hue = opt.get_hue();
    double sat = opt.get_saturation();

    return (hue >= WALL_OPT_ORANGE_HUE_MIN &&
            hue <= WALL_OPT_ORANGE_HUE_MAX &&
            sat >= WALL_OPT_ORANGE_SAT_MIN);
}

// ─────────────────────────────────────────────────────────────────
//  Optical gate
//  Returns false only if orange is detected on either sensor.
//  Everything else (white, yellow, gray, nothing) → returns true.
// ─────────────────────────────────────────────────────────────────
bool optical_gate_ok() {
    pros::Optical* opts[2]  = {&optical_fwd, &optical_left};
    const char*    names[2] = {"fwd", "left"};

    for (int i = 0; i < 2; i++) {
        if (!opts[i]->is_installed()) continue;
        int32_t prox = opts[i]->get_proximity();
        if (prox == PROS_ERR) continue;
        if (prox < WALL_OPT_DETECT_PROX) continue;

        if (optical_sees_orange(*opts[i])) {
            printf("[WC] optical[%s]: orange detected (hue=%.1f sat=%.2f) — corrections blocked\n",
                   names[i], opts[i]->get_hue(), opts[i]->get_saturation());
            return false;
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────
//  verified_drive
//
//  Forward drives (+target):
//    Snapshots dist_fwd before → PID drives → checks actual
//    displacement → corrects if error is 0.5"–3.0".
//    0.5" floor prevents oscillation chasing a perfect value.
//
//  Backward drives (−target):
//    No rear sensor — just runs PID, no correction.
//
//  Optical gate runs first: yellow stump or orange blocks in view
//  block all corrections for that move.
// ─────────────────────────────────────────────────────────────────
void verified_drive(double target_in, int speed, bool slew, bool chain) {

    // ── 1. Chain mode: fast PID, no correction ────────────────────
    if (chain) {
        chassis.pid_drive_set(target_in * 1_in, speed, slew);
        chassis.pid_wait_quick_chain();
        return;
    }

    // ── 2. Backward drive: no rear sensor, PID only ───────────────
    if (target_in < 0) {
        chassis.pid_drive_set(target_in * 1_in, speed, slew);
        chassis.pid_wait();
        return;
    }

    // ── 3. Optical gate + snapshot ────────────────────────────────
    bool   opt_ok     = optical_gate_ok();
    double snap       = -1.0;
    double hdg_before = chassis.odom_theta_get();

    if (opt_ok && is_valid_wall_dist(dist_fwd)) {
        snap = dist_fwd.get() / 25.4;
        printf("[WC] snap=%.3f in  hdg=%.2f deg\n", snap, hdg_before);
    }

    // ── 4. PID drive ──────────────────────────────────────────────
    chassis.pid_drive_set(target_in * 1_in, speed, slew);
    chassis.pid_wait();

    // ── 5. Heading correction ─────────────────────────────────────
    // Always runs for forward drives — decoupled from distance snapshot.
    // When the robot curves during a drive the forward sensor points
    // at an angle, making (snap - end) undercount displacement and
    // triggering a spurious forward correction.  Fix the heading
    // first; if heading was corrected, skip the distance correction
    // because the curved path already makes the position unreliable.
    double hdg_after       = chassis.odom_theta_get();
    double drift           = hdg_after - hdg_before;
    while (drift >  180.0) drift -= 360.0;
    while (drift < -180.0) drift += 360.0;

    bool heading_corrected = false;

    if (std::fabs(drift) > 1.0 && std::fabs(drift) <= 15.0) {
        printf("[WC] hdg drift %.2f deg — straightening to %.2f\n",
               drift, hdg_before);
        chassis.pid_turn_set(hdg_before * 1_deg, WALL_CORRECT_TURN_SPEED);
        chassis.pid_wait();
        heading_corrected = true;

        hdg_after = chassis.odom_theta_get();
        drift     = hdg_after - hdg_before;
        while (drift >  180.0) drift -= 360.0;
        while (drift < -180.0) drift += 360.0;
    }

    // ── 6. Drift guard ────────────────────────────────────────────
    if (std::fabs(drift) > 3.0) {
        printf("[WC] heading still off %.2f deg — skipping distance correction\n",
               drift);
        return;
    }

    // If heading was corrected, the drive was curved and our snapshot
    // position is no longer reliable for distance correction — stop here.
    if (heading_corrected) {
        printf("[WC] heading was corrected — skipping distance correction\n");
        return;
    }

    // No valid snapshot — heading was still corrected above, but skip distance.
    if (snap < 0) return;

    // ── 7. Read dist_fwd after move ───────────────────────────────
    if (!is_valid_wall_dist(dist_fwd)) {
        printf("[WC] lost wall after move — skipping\n");
        return;
    }

    double end          = dist_fwd.get() / 25.4;
    double actual_moved = snap - end;   // differential; sensor offset cancels
    double error        = target_in - actual_moved + 4.0;  // Account for 4" sensor offset from robot front

    printf("[WC] target=%.3f  actual=%.3f  error=%.4f in\n",
           target_in, actual_moved, error);

    // ── 8. Correct if outside tolerance ──────────────────────────
    if (std::fabs(error) < MIN_CORRECTION_IN) {
        printf("[WC] within %.1f\" tolerance — no correction\n", MIN_CORRECTION_IN);
        return;
    }
    if (std::fabs(error) > MAX_CORRECTION_IN) {
        printf("[WC] error %.3f\" too large — sensor likely obstructed\n", error);
        return;
    }

    printf("[WC] correcting %.4f in\n", error);
    chassis.pid_drive_set(error * 1_in, speed);
    chassis.pid_wait();
}

// ─────────────────────────────────────────────────────────────────
//  Heading correction  (requires two side sensors — left + right)
//  Currently only dist_side_left is installed; commented out until
//  a right side sensor is added.
// ─────────────────────────────────────────────────────────────────
// void wall_correct_heading(pros::Distance& dist_left,
//                            pros::Distance& dist_right,
//                            double          separation_in,
//                            double          expected_deg) {
//     if (!is_valid_wall_dist(dist_left) || !is_valid_wall_dist(dist_right)) {
//         printf("[WC] heading: invalid reading(s) — skipped\n");
//         return;
//     }
//
//     double da = dist_left.get()  / 25.4;
//     double db = dist_right.get() / 25.4;
//
//     double skew_deg      = std::atan2(da - db, separation_in) * (180.0 / M_PI);
//     double corrected_hdg = expected_deg - skew_deg;
//
//     double curr_hdg = chassis.odom_theta_get();
//     double delta    = corrected_hdg - curr_hdg;
//
//     if (std::fabs(delta) > 8.0) {
//         printf("[WC] heading correction too large (%.2f deg) — skipped\n", delta);
//         return;
//     }
//
//     printf("[WC] heading  %.3f  →  %.3f deg  (Δ=%.3f)\n",
//            curr_hdg, corrected_hdg, delta);
//     chassis.odom_xyt_set(chassis.odom_x_get() * 1_in,
//                          chassis.odom_y_get() * 1_in,
//                          corrected_hdg        * 1_deg);
// }
