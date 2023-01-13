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

#if ENABLED(SDSUPPORT)

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../module/printcounter.h"
#include "../../lcd/marlinui.h"

#if ENABLED(PARK_HEAD_ON_PAUSE)
  #include "../../feature/pause.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../feature/host_actions.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY) 
  #include "../../feature/powerloss.h"
#elif ENABLED(CREALITY_POWER_LOSS)
  #include "../../feature/PRE01_Power_loss/PRE01_Power_loss.h"
#endif

#if ENABLED(DGUS_LCD_UI_MKS)
  #include "../../lcd/extui/dgus/DGUSDisplayDef.h"
#endif

#include "../../MarlinCore.h" // for startOrResumeJob

#if HAS_CUTTER
  #include "../../feature/spindle_laser.h"
#endif

/**
 * M24: Start or Resume SD Print
 */
void GcodeSuite::M24() {
//SERIAL_ECHOPAIR("M24\n");
  #if ENABLED(DGUS_LCD_UI_MKS)
    if ((print_job_timer.isPaused() || print_job_timer.isRunning()) && !parser.seen("ST"))
      MKS_resume_print_move();
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    if (parser.seenval('S')) card.setIndex(parser.value_long());
    if (parser.seenval('T')) print_job_timer.resume(parser.value_long());
  #elif ENABLED(CREALITY_POWER_LOSS)
    if (parser.seenval('S')) card.setIndex(parser.value_long());
    if (parser.seenval('T')) print_job_timer.resume(parser.value_long());
  #endif

  #if ENABLED(PARK_HEAD_ON_PAUSE)
    if (did_pause_print) {
      resume_print(); // will call print_job_timer.start()
      return;
    }
  #endif

  #if HAS_CUTTER
    if(laser_device.is_laser_device()) // 107011-20210925 激光模式
    {
      laser_device.remove_card_before_is_printing = true; //记录拔卡之前的打印状态,用于拔卡恢复时判断 20211020
      //SERIAL_ECHOPAIR("M24 laser_device.power=", laser_device.power);
      cutter.apply_power(laser_device.power); // 恢复激光功率
    }
  #endif
  if (card.isFileOpen()) {
    card.startOrResumeFilePrinting();            // SD card will now be read for commands
    startOrResumeJob();               // Start (or resume) the print job timer
    TERN_(POWER_LOSS_RECOVERY, recovery.prepare());
    TERN_(CREALITY_POWER_LOSS, pre01_power_loss.prepare());
    // SERIAL_ECHOLNPAIR("\r\ninfo.sd_filename: ", pre01_power_loss.info.sd_filename);
  }

  #if ENABLED(HOST_ACTION_COMMANDS)
    #ifdef ACTION_ON_RESUME
      host_action_resume();
    #endif
    TERN_(HOST_PROMPT_SUPPORT, host_prompt_open(PROMPT_INFO, PSTR("Resuming SD"), DISMISS_STR));
  #endif

  ui.reset_status();
}

/**
 * M25: Pause SD Print
 *
 * With PARK_HEAD_ON_PAUSE:
 *   Invoke M125 to store the current position and move to the park
 *   position. M24 will move the head back before resuming the print.
 */
void GcodeSuite::M25() {
// SERIAL_ECHOPAIR("M25\n");
  #if ENABLED(PARK_HEAD_ON_PAUSE)

    M125();

  #else

    // Set initial pause flag to prevent more commands from landing in the queue while we try to pause
    #if ENABLED(SDSUPPORT)
      if (IS_SD_PRINTING()) card.pauseSDPrint();
    #endif

    #if ENABLED(POWER_LOSS_RECOVERY) && DISABLED(DGUS_LCD_UI_MKS)
      if (recovery.enabled) recovery.save(true);
    #elif ENABLED(CREALITY_POWER_LOSS) && DISABLED(DGUS_LCD_UI_MKS)
      if (pre01_power_loss.enabled) pre01_power_loss.save(true);
    #endif

    print_job_timer.pause();

    // //107011 -20210926
    // #if HAS_CUTTER
    //   if(laser_device.is_laser_device()){
    //     //保存暂停时的功率
    //     laser_device.power = cutter.power;
    //     SERIAL_ECHOPAIR("M24 laser_device.power=", laser_device.power);
    //   }
    // #endif

    TERN_(DGUS_LCD_UI_MKS, MKS_pause_print_move());
    
    TERN_(DWIN_CREALITY_LCD, ui.reset_status());
    //IF_DISABLED(DWIN_CREALITY_LCD, ui.reset_status());
    //TERN(DWIN_CREALITY_LCD,,ui.reset_status());

    #if ENABLED(HOST_ACTION_COMMANDS)
      TERN_(HOST_PROMPT_SUPPORT, host_prompt_open(PROMPT_PAUSE_RESUME, PSTR("Pause SD"), PSTR("Resume")));
      #ifdef ACTION_ON_PAUSE
        host_action_pause();
      #endif
    #endif

  #endif
}

#endif // SDSUPPORT
