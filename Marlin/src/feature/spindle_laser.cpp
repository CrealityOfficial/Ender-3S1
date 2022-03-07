/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * feature/spindle_laser.cpp
 */

#include "../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "spindle_laser.h"

#if ENABLED(SPINDLE_SERVO)
  #include "../module/servo.h"
#endif

SpindleLaser cutter;
uint8_t SpindleLaser::power = 0;

#if ENABLED(LASER_FEATURE)
  cutter_test_pulse_t SpindleLaser::testPulse = 50;                   // (ms) Test fire pulse default duration
  uint8_t SpindleLaser::last_block_power;                             // Track power changes for dynamic power
  feedRate_t SpindleLaser::feedrate_mm_m = 1500,
             SpindleLaser::last_feedrate_mm_m; // = 0                 // (mm/min) Track feedrate changes for dynamic power
#endif

bool SpindleLaser::isReady = false;                                   // Ready to apply power setting from the UI to OCR
CutterMode SpindleLaser::cutter_mode = CUTTER_MODE_STANDARD;

cutter_power_t SpindleLaser::menuPower = 0,                           // Power set via LCD menu in PWM, PERCENT, or RPM
               SpindleLaser::unitPower = 0;                           // LCD status power in PWM, PERCENT, or RPM

#if ENABLED(MARLIN_DEV_MODE)
  cutter_frequency_t SpindleLaser::frequency;                         // PWM frequency setting; range: 2K - 50K
#endif
#define SPINDLE_LASER_PWM_OFF TERN(SPINDLE_LASER_PWM_INVERT, 255, 0)

//
// Init the cutter to a safe OFF state
//
void SpindleLaser::init() {
  #if ENABLED(SPINDLE_SERVO)
    MOVE_SERVO(SPINDLE_SERVO_NR, SPINDLE_SERVO_MIN);
  #else 
    OUT_WRITE(SPINDLE_LASER_ENA_PIN, !SPINDLE_LASER_ACTIVE_STATE);    // Init spindle to off
  #endif
  #if ENABLED(SPINDLE_CHANGE_DIR)
    OUT_WRITE(SPINDLE_DIR_PIN, SPINDLE_INVERT_DIR ? 255 : 0);         // Init rotation to clockwise (M3)
  #endif
  #if ENABLED(SPINDLE_LASER_PWM)
    SET_PWM(SPINDLE_LASER_PWM_PIN);
    analogWrite(pin_t(SPINDLE_LASER_PWM_PIN), SPINDLE_LASER_PWM_OFF); // Set to lowest speed
  #endif
  #if ENABLED(HAL_CAN_SET_PWM_FREQ) && defined(SPINDLE_LASER_FREQUENCY)
    set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), SPINDLE_LASER_FREQUENCY);
    TERN_(MARLIN_DEV_MODE, frequency = SPINDLE_LASER_FREQUENCY);
  #endif
  #if ENABLED(AIR_EVACUATION)
    OUT_WRITE(AIR_EVACUATION_PIN, !AIR_EVACUATION_ACTIVE);            // Init Vacuum/Blower OFF
  #endif
  #if ENABLED(AIR_ASSIST)
    OUT_WRITE(AIR_ASSIST_PIN, !AIR_ASSIST_ACTIVE);                    // Init Air Assist OFF
  #endif
}

#if ENABLED(SPINDLE_LASER_PWM)
  /**
   * Set the cutter PWM directly to the given ocr value
   */
  void SpindleLaser::_set_ocr(const uint8_t ocr) {
    #if NEEDS_HARDWARE_PWM && SPINDLE_LASER_FREQUENCY
      if(laser_device.is_laser_device()) {
        laser_device.laser_power_start(ocr);
      }else{
        set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), TERN(MARLIN_DEV_MODE, frequency, SPINDLE_LASER_FREQUENCY));
        set_pwm_duty(pin_t(SPINDLE_LASER_PWM_PIN), ocr ^ SPINDLE_LASER_PWM_OFF);
      }
    #else 
      analogWrite(pin_t(SPINDLE_LASER_PWM_PIN), ocr ^ SPINDLE_LASER_PWM_OFF);
    #endif
  }

  void SpindleLaser::set_ocr(const uint8_t ocr) {
  // WRITE(SPINDLE_LASER_ENA_PIN,  SPINDLE_LASER_ACTIVE_STATE); // Cutter ON // 107011 -20210928
    _set_ocr(ocr);
  }

  void SpindleLaser::ocr_off() {
  //  WRITE(SPINDLE_LASER_ENA_PIN, !SPINDLE_LASER_ACTIVE_STATE); // Cutter OFF //107011-20210928
    _set_ocr(0);
  }
#endif

//
// Set cutter ON/OFF state (and PWM) to the given cutter power value
//
void SpindleLaser::apply_power(const uint8_t opwr) {
  static uint8_t last_power_applied =0;
  if (opwr == last_power_applied) return;
  last_power_applied = opwr;
  power = opwr;
  #if ENABLED(SPINDLE_LASER_PWM)
    if (cutter.unitPower == 0 && CUTTER_UNIT_IS(RPM)) {
      ocr_off();
      isReady = false;
    }
    else if (ENABLED(CUTTER_POWER_RELATIVE) || enabled()) {
      set_ocr(power);
      isReady = true;
    }
    else {
      ocr_off();
      isReady = false;
    }
  #elif ENABLED(SPINDLE_SERVO)
    MOVE_SERVO(SPINDLE_SERVO_NR, power);
  #else
    //107011 -20210928
    //WRITE(SPINDLE_LASER_ENA_PIN, enabled() ? SPINDLE_LASER_ACTIVE_STATE : !SPINDLE_LASER_ACTIVE_STATE);
    isReady = true;
  #endif
}

#if ENABLED(SPINDLE_CHANGE_DIR)
  //
  // Set the spindle direction and apply immediately
  // Stop on direction change if SPINDLE_STOP_ON_DIR_CHANGE is enabled
  //
  void SpindleLaser::set_reverse(const bool reverse) {
    const bool dir_state = (reverse == SPINDLE_INVERT_DIR); // Forward (M3) HIGH when not inverted
    if (TERN0(SPINDLE_STOP_ON_DIR_CHANGE, enabled()) && READ(SPINDLE_DIR_PIN) != dir_state) disable();
    WRITE(SPINDLE_DIR_PIN, dir_state);
  }
#endif

#if ENABLED(AIR_EVACUATION)

  // Enable / disable Cutter Vacuum or Laser Blower motor
  void SpindleLaser::air_evac_enable()  { WRITE(AIR_EVACUATION_PIN,  AIR_EVACUATION_ACTIVE); } // Turn ON

  void SpindleLaser::air_evac_disable() { WRITE(AIR_EVACUATION_PIN, !AIR_EVACUATION_ACTIVE); } // Turn OFF

  void SpindleLaser::air_evac_toggle()  { TOGGLE(AIR_EVACUATION_PIN); } // Toggle state

#endif // AIR_EVACUATION

#if ENABLED(AIR_ASSIST)

  // Enable / disable air assist
  void SpindleLaser::air_assist_enable()  { WRITE(AIR_ASSIST_PIN,  AIR_ASSIST_PIN); } // Turn ON

  void SpindleLaser::air_assist_disable() { WRITE(AIR_ASSIST_PIN, !AIR_ASSIST_PIN); } // Turn OFF

  void SpindleLaser::air_assist_toggle()  { TOGGLE(AIR_ASSIST_PIN); } // Toggle state

#endif // AIR_ASSIST

  // 107011
 class spindle_laser_soft_pwm laser_device;



#endif // HAS_CUTTER
