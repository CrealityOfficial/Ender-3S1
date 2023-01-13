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

#include "../../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "../gcode.h"
#include "../../feature/spindle_laser.h"
#include "../../module/stepper.h"

/**
 * Laser:
 *  M3 - Laser ON/Power (Ramped power)
 *  M4 - Laser ON/Power (Continuous power)
 *
 * Spindle:
 *  M3 - Spindle ON (Clockwise)
 *  M4 - Spindle ON (Counter-clockwise)
 *
 * Parameters:
 *  S<power> - Set power. S0 will turn the spindle/laser off, except in relative mode.
 *  O<ocr>   - Set power and OCR (oscillator count register)
 *
 *  If no PWM pin is defined then M3/M4 just turns it on.
 *
 *  At least 12.8KHz (50Hz * 256) is needed for Spindle PWM.
 *  Hardware PWM is required on AVR. ISRs are too slow.
 *
 * NOTE: WGM for timers 3, 4, and 5 must be either Mode 1 or Mode 5.
 *       No other settings give a PWM signal that goes from 0 to 5 volts.
 *
 *       The system automatically sets WGM to Mode 1, so no special
 *       initialization is needed.
 *
 *       WGM bits for timer 2 are automatically set by the system to
 *       Mode 1. This produces an acceptable 0 to 5 volt signal.
 *       No special initialization is needed.
 *
 * NOTE: A minimum PWM frequency of 50 Hz is needed. All prescaler
 *       factors for timers 2, 3, 4, and 5 are acceptable.
 *
 *  SPINDLE_LASER_ENA_PIN needs an external pullup or it may power on
 *  the spindle/laser during power-up or when connecting to the host
 *  (usually goes through a reset which sets all I/O pins to tri-state)
 *
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */
void GcodeSuite::M3_M4(const bool is_M4) {
// SERIAL_ECHOPAIR("M3_M4\n");
  planner.synchronize();   // Wait for previous movement commands (G0/G1/G2/G3) to complete before changing power
  if (parser.seen('I')) {
    cutter.cutter_mode = is_M4 ? CUTTER_MODE_DYNAMIC : CUTTER_MODE_CONTINUOUS;
    cutter.set_enabled(true);
  }

  auto get_s_power = [] {
    if (parser.seen('S')) {
      cutter.unitPower = parser.value_ushort();
      // PWM implied and ranges from S0 to S180 for a positional servo. Typical use would be a pen up/down function.
      #if ENABLED(SPINDLE_SERVO)
        cutter.power = cutter.unitPower;
      #else
        if (cutter.cutter_mode == CUTTER_MODE_STANDARD){ // PWM not implied, power converted to OCR from unit definition and min/max or on/off if not PWM.
          if(laser_device.is_laser_device())
          {
              // 107011 激光模式下将S0-1000比例转换为S0-255
              cutter.unitPower = laser_device.power16_to_8(cutter.unitPower);
              //cutter.power= cutter.unitPower;
          }else{ 
            cutter.power = TERN(SPINDLE_LASER_PWM, cutter.power_to_range(cutter_power_t(cutter.unitPower)), cutter.unitPower > 0 ? 255 : 0);
          }
        }
      #endif
      //cutter.menuPower = cutter.unitPower;
    }
    else if (cutter.cutter_mode == CUTTER_MODE_STANDARD){
      cutter.unitPower = cutter.cpwr_to_upwr(SPEED_POWER_STARTUP);
    }
    return cutter.unitPower;// cutter.unitPower;
  };

  if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS || cutter.cutter_mode == CUTTER_MODE_DYNAMIC) {  // Laser power in inline mode
    TERN_(LASER_FEATURE, cutter.inline_power(cutter.upower_to_ocr(get_s_power())));
  }
  else {
    #if ENABLED(SPINDLE_LASER_PWM)
      //cutter.set_power(cutter.upower_to_ocr(get_s_power())); // 107011-20210925 修复M3 S 命令无效的bug
      cutter.apply_power(get_s_power());//107011 -20211008 cutter.set_ocr(cutter.upower_to_ocr(get_s_power())); // 107011-20210925 修复M3 S 命令无效的bug
    #elif ENABLED(SPINDLE_SERVO)
      cutter.set_power(get_s_power());
    #else
      cutter.set_enabled(true);
    #endif
    //cutter.set_reverse(is_M4); /107011-20210925 //修复M3 S 命令无效的bug
  }
}

/**
 * M5 - Cutter OFF (when moves are complete)
 */
void GcodeSuite::M5() {
  
  planner.synchronize();
  if (parser.seen('I')) {
    cutter.set_enabled(false);                  // Clear inline mode flags
    cutter.cutter_mode = CUTTER_MODE_STANDARD;  // Switch from inline to standard mode, has no effect on current power output!
  }
  cutter.apply_power(0);                        // M5 kills power in either mode but if it's in inline it will be still be the active mode

}

#endif // HAS_CUTTER
