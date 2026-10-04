/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2026 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(RESONANCE_TEST)

#include "resonance_generator.h"

#if ENABLED(FT_MOTION)
  #include "../../module/ft_motion.h"
#endif

#include "../../gcode/gcode.h" // for home_all_axes

resonance_test_params_t ResonanceGenerator::rt_params;     // Resonance test parameters
float ResonanceGenerator::timeline;
float ResonanceGenerator::sample_time;
float ResonanceGenerator::sweep_center;
bool ResonanceGenerator::center_valid = false;
#if ENABLED(FT_MOTION)
  uint32_t ResonanceGenerator::ftm_frame = 0;
  uint32_t ResonanceGenerator::ftm_phase = 0;
#endif

bool ResonanceGenerator::active = false;                   // Resonance test active
bool ResonanceGenerator::done = false;                     // Resonance test done
int32_t ResonanceGenerator::freq_to_phase_fp;
int32_t ResonanceGenerator::max_freq_fp;
int32_t ResonanceGenerator::phase_fp;
int32_t ResonanceGenerator::current_freq_fp;
int32_t ResonanceGenerator::amplitude_precalc_fp; 


#if HAS_STANDARD_MOTION
  block_t ResonanceGenerator::block;
#endif

ResonanceGenerator rtg;

ResonanceGenerator::ResonanceGenerator() {}

void ResonanceGenerator::start() {
  gcode.home_all_axes(); // Always home axes first
  motion.blocking_move_xy(X_CENTER, Y_CENTER, Z_CLEARANCE_FOR_HOMING);

  rt_params.start_pos = motion.position;
  active = true;
  done = false;
  center_valid = false;
  TERN_(FT_MOTION, ftm_frame = ftm_phase = 0);

  // Clamp Z-axis acceleration for safety
  if (rt_params.axis == Z_AXIS)
    NOMORE(rt_params.accel_per_hz, 15.0f);

  // Calculate time constant for sine sweep
  const float rt_time = rt_params.octave_duration * (logf(RATIO) / logf(2.0f));

  sample_time = rt_time;

  #if HAS_STANDARD_MOTION
    if (TERN1(FT_MOTION, !ftMotion.cfg.active)) {
      block.reset();
      block.initial_rate = (rt_params.axis == Z_AXIS) ? 2000 : 8000;
    }
  #endif

  // Precompute fixed-point sine sweep parameters
  amplitude_precalc_fp = F2FP((rt_params.amplitude_correction * rt_params.accel_per_hz * 0.25f) / sq(M_PI));
  current_freq_fp = F2FP(rt_params.min_freq);
  freq_to_phase_fp = F2FP(2.0f * M_PI * rt_time);
  max_freq_fp = F2FP(rt_params.max_freq);
  phase_fp = 0;
}

void ResonanceGenerator::abort() {
  reset();
  #if HAS_STANDARD_MOTION
    if (!TERN0(FT_MOTION, ftMotion.cfg.active))
      return;
  #endif
  TERN_(FT_MOTION, ftMotion.reset());
}

void ResonanceGenerator::reset() {
  rt_params = resonance_test_params_t();
  #if HAS_STANDARD_MOTION
    if (!TERN0(FT_MOTION, ftMotion.cfg.active))
      block.reset();
  #endif
  active = false;
  done = false;
}

float ResonanceGenerator::calc_next_pos() {
  // Phase accumulation (drop 4 LSBs of freq so the product stays inside int32)
  phase_fp += ((current_freq_fp >> 4) * freq_to_phase_fp) >> (FP_BITS - 4);
  if (phase_fp >= M_TAU_FP) phase_fp -= M_TAU_FP;
  else if (phase_fp < 0) phase_fp += M_TAU_FP;

  return calc_pos(phase_fp, FP2F(current_freq_fp));
}

float ResonanceGenerator::calc_pos(const int32_t phase_fp, const float freq_hz) {
  // -π <= r_fp <= π
  const int32_t r_fp = (phase_fp > M_PI_FP) ? phase_fp - M_TAU_FP : phase_fp;

  // r² in Q16:  Q12 × Q12 = Q24, >> 8
  const int32_t rh_fp = r_fp >> 4;
  const int32_t r2_fp = (rh_fp * rh_fp) >> 8;

  // 1.0 - 0.101321184·r²:  Q12 × Q16 = Q28, >> 12
  const int32_t poly_fp = (int32_t)FP_ONE - (((r2_fp >> 4) * C0101321184_FP) >> 12);

  // r·poly:  Q15 × Q15 = Q30, >> 14 → Q16 (peaks at ~1.21·2³⁰, fits in int32)
  const int32_t rp_fp = ((r_fp >> 1) * (poly_fp >> 1)) >> 14;

  // Amplitude ∝ 1/f done in float: one div, one mul, no int64
  return (FP2F(amplitude_precalc_fp) / freq_hz) * FP2F(rp_fp);
}

#if ENABLED(FT_MOTION)

  void ResonanceGenerator::fill_stepper_plan_buffer() {
    // FT Motion trajectory coordinates are relative to its last reset(), not absolute machine
    // positions (rt_params.start_pos), so sweep around the last queued point. Using start_pos
    // here would command a jump of the whole machine position in a single frame.
    xyze_float_t traj_coords = ftMotion.get_last_target_traj();

    #if HAS_FTM_CORE_CARTESIAN
      // On CoreXY last_target_traj is in motor A/B. Sweep in head X/Y: invert A = X+Y, B = CORESIGN(X-Y).
      {
        const float a = traj_coords.x, b = traj_coords.y;
        traj_coords.x = (a + CORESIGN(b)) * 0.5f;
        traj_coords.y = (a - CORESIGN(b)) * 0.5f;
      }
    #endif

    // Sweep center, kept from the first frame to avoid cumulative errors
    if (!center_valid) {
      sweep_center = traj_coords[rt_params.axis];
      center_valid = true;
    }

    // One frame is FTM_TS long. The frequency doubles every octave_duration seconds
    // of real time, so derive it from the frame count (no accumulated error).
    const float octaves_per_frame = (FTM_TS) / rt_params.octave_duration;

    while (!ftMotion.stepping.is_full()) {
      // Calculate current frequency with exponential sweep
      const float freq = rt_params.min_freq * exp2f(float(ftm_frame) * octaves_per_frame);
      if (freq > rt_params.max_freq) {
        done = true;
        return;
      }
      ++ftm_frame;

      // Integrate frequency into phase, in whole cycles of 2^32 so it wraps for free
      ftm_phase += uint32_t(freq * ((FTM_TS) * 4294967296.0f));

      // Cycles (2^32) -> radians (Q16, 0..2π)
      const int32_t phase_fp = int32_t((uint64_t(ftm_phase) * M_TAU_FP) >> 32);

      // Resonate the axis being tested
      traj_coords[rt_params.axis] = sweep_center + calc_pos(phase_fp, freq);

      // Head X/Y -> motor A/B (as FTMotion::calc_traj_point does); the hold and stepping plan work on motor coordinates
      xyze_float_t motor_coords = traj_coords;
      #if HAS_FTM_CORE_CARTESIAN
        motor_coords.x = traj_coords.x + traj_coords.y;
        motor_coords.y = CORESIGN(traj_coords.x - traj_coords.y);
      #endif

      TERN_(HAS_FTM_DIR_CHANGE_HOLD, motor_coords = ftMotion.ftm_hold_frames(motor_coords));

      // Store in buffer
      ftMotion.stepping_enqueue(motor_coords);
    }
  }

#endif // FT_MOTION

#if HAS_STANDARD_MOTION

  block_t* ResonanceGenerator::generate_resonance_block() {
    // Static variables to retain state between calls and avoid cumulative errors
    static float prev_pos = 0.0f, step_accumulator = 0.0f;

    const float step_mm = planner.settings.axis_steps_per_mm[rt_params.axis];
    const uint8_t axis_bit = 1 << rt_params.axis;

    // Calculate current frequency with exponential sweep
    current_freq_fp += current_freq_fp >> FP_BITS;
    if (current_freq_fp > max_freq_fp) {
      done = true;
      return nullptr;
    }

    // Calculate position and accumulate steps
    const float current_pos = calc_next_pos();
    step_accumulator += (current_pos - prev_pos) * step_mm;
    prev_pos = current_pos;

    // Extract steps
    const int32_t delta_steps = (int32_t)floor(step_accumulator);
    step_accumulator -= delta_steps;
    const uint32_t abs_steps = abs(delta_steps);

    block.initial_rate = (uint32_t)(_MAX(abs_steps, 1U) / sample_time);

    // Update block
    block.steps[rt_params.axis] = abs_steps;
    block.step_event_count = abs_steps;
    if (delta_steps < 0)
      block.direction_bits |= axis_bit;
    else
      block.direction_bits &= ~axis_bit;

    return &block;
  }

#endif // HAS_STANDARD_MOTION

#endif // RESONANCE_TEST
