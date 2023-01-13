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
#include "../../inc/MarlinConfigPre.h"

#if ENABLED(CREALITY_POWER_LOSS)

#include "PRE01_Power_loss.h"
#include "../../core/macros.h"
#include "../../sd/cardreader.h"
#include "../../lcd/marlinui.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/printcounter.h"
#include "../../module/temperature.h"
#include "../../core/serial.h"
#include "../../libs/BL24CXX.h"

#if ENABLED(FWRETRACT)
  #include "fwretract.h"
#endif

#if ENABLED(RTS_AVAILABLE)
  #include "../../lcd/dwin/lcd_rts.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_POWER_LOSS_RECOVERY)
#include "../../core/debug_out.h"


/***************************************************
 *                全局变量声明区域
 * 
 * 
 * 
 * 
**************************************************/
//ADC采样相关的参数
#define SPLIT_VOLTAGE_RATIO         (11.0)   //采样电阻的比值
#define POWER_CHECK_REFERENCE_VOL   (3.30)   //采样参考电压3.3v
#define POWER_DOWN_THRESHOLD_UP     (23.0)   //断电的上限电压
#define POWER_DOWN_THRESHOLD_DWON   (3.3)    //断电的下限电压
#define ADC_SAMPLE_RATIO            (4096.0) //采样分辨率，10位ADC采样
#define CHECK_TIMES                 (1)     //CHECK_TIMES 次以后V才认为是断电  否则认为是抖动不做处理

/************************ADC采集的参数预设*****************************/
// adc 24V电压采样的结果
Power_ADC_Info_t PRE01PowerLoss::POWER_ADC_VALUE;  //ADC采样后的数值


/************************断电续打恢复需要的参数预设*****************************/
//断电续打后的挤出恢复长度
#ifndef POWER_LOSS_PURGE_LEN       
  #define POWER_LOSS_PURGE_LEN 0
#endif

//断电时无备用电源不缩回
#if DISABLED(BACKUP_POWER_SUPPLY)
  #undef POWER_LOSS_RETRACT_LEN   // No retract at outage without backup power
#endif

//断电续打后回抽长度
#ifndef POWER_LOSS_RETRACT_LEN
  #define POWER_LOSS_RETRACT_LEN 0
#endif

/***************************断电续打先关的变量定义********************************/

bool PRE01PowerLoss::enabled; // Initialized by settings.load()

SdFile PRE01PowerLoss::file;
creality_job_recovery_info_t PRE01PowerLoss::info;
const char PRE01PowerLoss::filename[5] = "/PLR";
uint8_t PRE01PowerLoss::queue_index_r;
uint32_t PRE01PowerLoss::cmd_sdpos, // = 0
         PRE01PowerLoss::sdpos[BUFSIZE];

#if ENABLED(DWIN_CREALITY_LCD)
  bool PRE01PowerLoss::dwin_flag; // = false
#endif

/**********************************************************
 *                  类对象定义区域
 * 
 * 
 * 
 * ******************************************************/
// STM32ADC current_vlotage_adc(ADC1);  //24v电压采集类ADC类对象定义
PRE01PowerLoss pre01_power_loss;


/*****************************************************
 * Copyright:www.creality.com
 * Author:jankin
 * Date:2021-09-30
 * Description:超级电容断电续打初始化
******************************************************/

void PRE01PowerLoss::init()
{
  ADC_GPIO_init();
  OUT_WRITE(BACKPOWER_CTRL_PIN, LOW);    /*底电平打开电容充电*/
}

/*****************************************************
 * Copyright:www.creality.com
 * Author:jankin
 * Date:2021-09-30
 * Description:获取当前采样到数据 并转换成当前的24V电压值
******************************************************/
void PRE01PowerLoss::check_current_voltage(){
  float raw = POWER_ADC_VALUE.raw;
  float voltage = (raw > 0) ? (((double_t)raw / ADC_SAMPLE_RATIO) * POWER_CHECK_REFERENCE_VOL * SPLIT_VOLTAGE_RATIO) : 2; //10-bit ADC acquisition 1 times
  static uint8_t count = 0;
  static bool power_loss_flag = false; 
  SERIAL_ECHOLNPAIR("\r\nraw: ", raw);
  SERIAL_ECHOLNPAIR("\r\nvoltage: ", voltage);
  if( (voltage >= POWER_DOWN_THRESHOLD_DWON) && (voltage <= POWER_DOWN_THRESHOLD_UP)) //低于阈值电压，触发断电打印数据保存
  {
    ++count;

    if(count == CHECK_TIMES && !power_loss_flag)
    {
       OUT_WRITE(BACKPOWER_CTRL_PIN, HIGH);
       delay(50); 
       power_loss_flag = true;
       save(true);
       delay(100);
       HAL_reboot(); // Reboot the board
    }
  }
  else
  {
    count = 0;
  }
}

/**
 * Clear the recovery info
 */
void PRE01PowerLoss::data_init() { memset(&info, 0, sizeof(info)); }


/**
 * Enable or disable then call changed()
 */
void PRE01PowerLoss::enable(const bool onoff) {
  enabled = onoff;
  changed();
}

/**
 * The enabled state was changed:
 *  - Enabled: Purge the job recovery file
 *  - Disabled: Write the job recovery file
 */
void PRE01PowerLoss::changed() {
  if (!enabled)
    purge();
  else if (IS_SD_PRINTING())
    save(true);
}

/**
 * Check for Print Job Recovery during setup()
 *
 * If a saved state exists send 'M1000 S' to initiate job recovery.
 */
void PRE01PowerLoss::check() {
  // BL24CXX::read(PLR_ADDR,(uint8_t*)&info,sizeof(info));
  // if (!card.isMounted()) card.mount();
  if (card.isMounted()) {
    // load();
    #ifdef EEPROM_PLR
      BL24CXX::read(PLR_ADDR,(uint8_t*)&info,sizeof(info));
    #endif

    if (!valid()) return cancel();
    // queue.inject_P(PSTR("M1000S"));
    info.print_job_elapsed = print_job_timer.duration() + info.print_job_elapsed;
  }
}

/**
 * Delete the recovery file and clear the recovery data
 */
void PRE01PowerLoss::purge() {
  data_init();   
  write();     //清空EEPROM里面的断电续打数据
  // card.removeJobRecoveryFile();
}

/**
 * Load the recovery data, if it exists
 */
void PRE01PowerLoss::load() {
  // if (exists()) {
  //   open(true);
  //   (void)file.read(&info, sizeof(info));
  //   close();
  // }
  debug(PSTR("Load"));
}

/**
 * Set info fields that won't change
 */
void PRE01PowerLoss::prepare() {
  card.getAbsFilenameInCWD(info.sd_filename);  // SD filename
  cmd_sdpos = 0;
}

/**
 * Save the current machine state to the power-loss recovery file
 */
void PRE01PowerLoss::save(const bool force/*=false*/, const float zraise/*=POWER_LOSS_ZRAISE*/, const bool raised/*=false*/) {

  // We don't check IS_SD_PRINTING here so a save may occur during a pause

  #if SAVE_INFO_INTERVAL_MS > 0
    static millis_t next_save_ms; // = 0
    millis_t ms = millis();
  #endif

  #ifndef POWER_LOSS_MIN_Z_CHANGE
    #define POWER_LOSS_MIN_Z_CHANGE 0.05  // Vase-mode-friendly out of the box
  #endif

  // Did Z change since the last call?
  if (force
    #if DISABLED(SAVE_EACH_CMD_MODE)      // Always save state when enabled
      #if SAVE_INFO_INTERVAL_MS > 0       // Save if interval is elapsed
        || ELAPSED(ms, next_save_ms)
      #endif
      // Save if Z is above the last-saved position by some minimum height
      || current_position.z > info.current_position.z + POWER_LOSS_MIN_Z_CHANGE
    #endif
  ) {

    #if SAVE_INFO_INTERVAL_MS > 0
      next_save_ms = ms + SAVE_INFO_INTERVAL_MS;
    #endif

    // Set Head and Foot to matching non-zero values
    if (!++info.valid_head) ++info.valid_head; // non-zero in sequence
    //if (!IS_SD_PRINTING()) info.valid_head = 0;
    info.valid_foot = info.valid_head;

    // Machine state
    info.current_position = current_position;
    info.feedrate = uint16_t(MMS_TO_MMM(feedrate_mm_s));
    info.zraise = zraise;
    info.flag.raised = raised;                      // Was Z raised before power-off?

    TERN_(GCODE_REPEAT_MARKERS, info.stored_repeat = repeat);
    TERN_(HAS_HOME_OFFSET, info.home_offset = home_offset);
    TERN_(HAS_POSITION_SHIFT, info.position_shift = position_shift);
    TERN_(HAS_MULTI_EXTRUDER, info.active_extruder = active_extruder);

    #if DISABLED(NO_VOLUMETRICS)
      info.flag.volumetric_enabled = parser.volumetric_enabled;
      #if HAS_MULTI_EXTRUDER
        for (int8_t e = 0; e < EXTRUDERS; e++) info.filament_size[e] = planner.filament_size[e];
      #else
        if (parser.volumetric_enabled) info.filament_size[0] = planner.filament_size[active_extruder];
      #endif
    #endif

    #if HAS_EXTRUDERS
      HOTEND_LOOP() info.target_temperature[e] = thermalManager.degTargetHotend(e);
    #endif

    TERN_(HAS_HEATED_BED, info.target_temperature_bed = thermalManager.degTargetBed());

    #if HAS_FAN
      COPY(info.fan_speed, thermalManager.fan_speed);
    #endif

    #if HAS_LEVELING
      info.flag.leveling = planner.leveling_active;
      info.fade = TERN0(ENABLE_LEVELING_FADE_HEIGHT, planner.z_fade_height);
    #endif

    TERN_(GRADIENT_MIX, memcpy(&info.gradient, &mixer.gradient, sizeof(info.gradient)));

    #if ENABLED(FWRETRACT)
      COPY(info.retract, fwretract.current_retract);
      info.retract_hop = fwretract.current_hop;
    #endif

    // Elapsed print job time
    info.print_job_elapsed = print_job_timer.duration();

    // Relative axis modes
    info.axis_relative = gcode.axis_relative;

    // Misc. Marlin flags
    info.flag.dryrun = !!(marlin_debug_flags & MARLIN_DEBUG_DRYRUN);
    info.flag.allow_cold_extrusion = TERN0(PREVENT_COLD_EXTRUSION, thermalManager.allow_cold_extrude);

    #if ENABLED(RTS_AVAILABLE)
      //recovery flag 
      // info.recovery_flag = true;
      info.recovery_flag = PoweroffContinue;
    #endif

    write();
  }
}

/**
 * Save the recovery info the recovery file
 */
void PRE01PowerLoss::write() {

  debug(PSTR("Write"));
  BL24CXX::write(PLR_ADDR, (uint8_t*)&pre01_power_loss.info, sizeof(pre01_power_loss.info));
  // open(false);
  // file.seekSet(0);
  // const int16_t ret = file.write(&info, sizeof(info));
  // if (ret == -1) DEBUG_ECHOLNPGM("Power-loss file write failed.");
  // if (!file.close()) DEBUG_ECHOLNPGM("Power-loss file close failed.");
}


/**
 * Resume the saved print job
 */
void PRE01PowerLoss::resume() {

  char cmd[MAX_CMD_SIZE+16], str_1[16], str_2[16];

  if(info.sdpos>=96)
  {
    info.sdpos-=96; //rock_20210819;
  }

  const uint32_t resume_sdpos = info.sdpos; // Get here before the stepper ISR overwrites it

  // Apply the dry-run flag if enabled
  if (info.flag.dryrun) marlin_debug_flags |= MARLIN_DEBUG_DRYRUN;

  // Restore cold extrusion permission
  TERN_(PREVENT_COLD_EXTRUSION, thermalManager.allow_cold_extrude = info.flag.allow_cold_extrusion);

  #if HAS_LEVELING
    // Make sure leveling is off before any G92 and G28
    gcode.process_subcommands_now_P(PSTR("M420 S0 Z0"));
  #endif

  #if HAS_HEATED_BED
    const celsius_t bt = info.target_temperature_bed;
    if (bt) {
      // Restore the bed temperature
      sprintf_P(cmd, PSTR("M190S%i"), bt);
      gcode.process_subcommands_now(cmd);
    }
  #endif

  // Heat hotend enough to soften material
  #if HAS_HOTEND
    HOTEND_LOOP() {
      const celsius_t et = _MAX(info.target_temperature[e], 180);
      if (et) {
        #if HAS_MULTI_HOTEND
          sprintf_P(cmd, PSTR("T%iS"), e);
          gcode.process_subcommands_now(cmd);
        #endif
        sprintf_P(cmd, PSTR("M109S%i"), et);
        gcode.process_subcommands_now(cmd);
      }
    }
  #endif

  // Interpret the saved Z according to flags
  const float z_print = info.current_position.z,
              z_raised = z_print + info.zraise;

  //
  // Home the axes that can safely be homed, and
  // establish the current position as best we can.
  //

  gcode.process_subcommands_now_P(PSTR("G92.9E0")); // Reset E to 0

  #if Z_HOME_TO_MAX

    float z_now = z_raised;

    // If Z homing goes to max then just move back to the "raised" position
    sprintf_P(cmd, PSTR(
            "G28R0\n"     // Home all axes (no raise)
            "G1Z%sF1200"  // Move Z down to (raised) height
          ), dtostrf(z_now, 1, 3, str_1));
    gcode.process_subcommands_now(cmd);

  #else

    #if ENABLED(POWER_LOSS_RECOVER_ZHOME) && defined(POWER_LOSS_ZHOME_POS)
      #define HOMING_Z_DOWN 1
    #else
      #define HOME_XY_ONLY 1
    #endif

    float z_now = info.flag.raised ? z_raised : z_print;

    // Reset E to 0 and set Z to the real position
    #if HOME_XY_ONLY
      sprintf_P(cmd, PSTR("G92.9Z%s"), dtostrf(z_now, 1, 3, str_1));
      gcode.process_subcommands_now(cmd);
    #endif

    // Does Z need to be raised now? It should be raised before homing XY.
    // if (z_raised > z_now) {
    //   z_now = z_raised;
    //   sprintf_P(cmd, PSTR("G1Z%sF60"), dtostrf(z_now, 1, 3, str_1));
    //   gcode.process_subcommands_now(cmd);
    // }

    // Home XY with no Z raise, and also home Z here if Z isn't homing down below.
    gcode.process_subcommands_now_P(PSTR("G28R0" TERN_(HOME_XY_ONLY, "XY"))); // No raise during G28

  #endif

  #if HOMING_Z_DOWN
    // Move to a safe XY position and home Z while avoiding the print.
    constexpr xy_pos_t p = POWER_LOSS_ZHOME_POS;
    sprintf_P(cmd, PSTR("G1X%sY%sF1000\nG28Z"), dtostrf(p.x, 1, 3, str_1), dtostrf(p.y, 1, 3, str_2));
    gcode.process_subcommands_now(cmd);
  #endif

  // Mark all axes as having been homed (no effect on current_position)
  set_all_homed();

  #if HAS_LEVELING
    // Restore Z fade and possibly re-enable bed leveling compensation.
    // Leveling may already be enabled due to the ENABLE_LEVELING_AFTER_G28 option.
    // TODO: Add a G28 parameter to leave leveling disabled.
    // sprintf_P(cmd, PSTR("M420S%cZ%s"), '0' + (char)info.flag.leveling, dtostrf(info.fade, 1, 1, str_1));
    // gcode.process_subcommands_now(cmd);

    // #if HOME_XY_ONLY
    //   // The physical Z was adjusted at power-off so undo the M420S1 correction to Z with G92.9.
      sprintf_P(cmd, PSTR("G92.9Z%s"), dtostrf(z_now, 1, 1, str_1));
      gcode.process_subcommands_now(cmd);
    // #endif
  #endif

  #if ENABLED(POWER_LOSS_RECOVER_ZHOME)
    // Z was homed down to the bed, so move up to the raised height.
    z_now = z_raised;
    sprintf_P(cmd, PSTR("G1Z%sF600"), dtostrf(z_now, 1, 3, str_1));
    gcode.process_subcommands_now(cmd);
  #endif

  // Recover volumetric extrusion state
  #if DISABLED(NO_VOLUMETRICS)
    #if HAS_MULTI_EXTRUDER
      for (int8_t e = 0; e < EXTRUDERS; e++) {
        sprintf_P(cmd, PSTR("M200T%iD%s"), e, dtostrf(info.filament_size[e], 1, 3, str_1));
        gcode.process_subcommands_now(cmd);
      }
      if (!info.flag.volumetric_enabled) {
        sprintf_P(cmd, PSTR("M200T%iD0"), info.active_extruder);
        gcode.process_subcommands_now(cmd);
      }
    #else
      if (info.flag.volumetric_enabled) {
        sprintf_P(cmd, PSTR("M200D%s"), dtostrf(info.filament_size[0], 1, 3, str_1));
        gcode.process_subcommands_now(cmd);
      }
    #endif
  #endif

  // Restore all hotend temperatures
  #if HAS_HOTEND
    HOTEND_LOOP() {
      const celsius_t et = info.target_temperature[e];
      if (et) {
        #if HAS_MULTI_HOTEND
          sprintf_P(cmd, PSTR("T%iS"), e);
          gcode.process_subcommands_now(cmd);
        #endif
        sprintf_P(cmd, PSTR("M109S%i"), et);
        gcode.process_subcommands_now(cmd);
      }
    }
  #endif

  // Restore the previously active tool (with no_move)
  #if HAS_MULTI_EXTRUDER || HAS_MULTI_HOTEND
    sprintf_P(cmd, PSTR("T%i S"), info.active_extruder);
    gcode.process_subcommands_now(cmd);
  #endif

  // Restore print cooling fan speeds
  #if HAS_FAN
    FANS_LOOP(i) {
      const int f = info.fan_speed[i];
      if (f) {
        sprintf_P(cmd, PSTR("M106P%iS%i"), i, f);
        gcode.process_subcommands_now(cmd);
      }
    }
  #endif

  // Restore retract and hop state from an active `G10` command
  #if ENABLED(FWRETRACT)
    LOOP_L_N(e, EXTRUDERS) {
      if (info.retract[e] != 0.0) {
        fwretract.current_retract[e] = info.retract[e];
        fwretract.retracted[e] = true;
      }
    }
    fwretract.current_hop = info.retract_hop;
  #endif

  #if ENABLED(GRADIENT_MIX)
    memcpy(&mixer.gradient, &info.gradient, sizeof(info.gradient));
  #endif

  // Un-retract if there was a retract at outage
  #if ENABLED(BACKUP_POWER_SUPPLY) && POWER_LOSS_RETRACT_LEN > 0
    gcode.process_subcommands_now_P(PSTR("G1E" STRINGIFY(POWER_LOSS_RETRACT_LEN) "F3000"));
  #endif

  // Additional purge on resume if configured
  #if POWER_LOSS_PURGE_LEN
    sprintf_P(cmd, PSTR("G1 E%d F3000"), (POWER_LOSS_PURGE_LEN) + (POWER_LOSS_RETRACT_LEN));
    gcode.process_subcommands_now(cmd);
  #endif

  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    gcode.process_subcommands_now_P(PSTR("G12"));
  #endif

  gcode.process_subcommands_now_P(PSTR("G1 E5 F1000"));
  // Move back over to the saved XY
  sprintf_P(cmd, PSTR("G1X%sY%sF3000"),
    dtostrf(info.current_position.x, 1, 3, str_1),
    dtostrf(info.current_position.y, 1, 3, str_2)
  );
  gcode.process_subcommands_now(cmd);

  // Move back down to the saved Z for printing
  sprintf_P(cmd, PSTR("G1Z%sF60"), dtostrf(z_print, 1, 3, str_1));
  gcode.process_subcommands_now(cmd);

  // Restore the feedrate
  sprintf_P(cmd, PSTR("G1F%d"), info.feedrate);
  gcode.process_subcommands_now(cmd);

  // Restore E position with G92.9
  sprintf_P(cmd, PSTR("G92.9E%s"), dtostrf((info.current_position.e - 10), 1, 3, str_1));
  gcode.process_subcommands_now(cmd);

  TERN_(GCODE_REPEAT_MARKERS, repeat = info.stored_repeat);
  TERN_(HAS_HOME_OFFSET, home_offset = info.home_offset);
  TERN_(HAS_POSITION_SHIFT, position_shift = info.position_shift);
  #if HAS_HOME_OFFSET || HAS_POSITION_SHIFT
    LOOP_LINEAR_AXES(i) update_workspace_offset((AxisEnum)i);
  #endif

  // Relative axis modes
  gcode.axis_relative = info.axis_relative;

  #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
    const uint8_t old_flags = marlin_debug_flags;
    marlin_debug_flags |= MARLIN_DEBUG_ECHO;
  #endif

  // Continue to apply PLR when a file is resumed!
  enable(true);

  // Resume the SD file from the last position
  char *fn = info.sd_filename;
  sprintf_P(cmd, M23_STR, fn);
  gcode.process_subcommands_now(cmd);
  sprintf_P(cmd, PSTR("M24S%ldT%ld"), resume_sdpos, info.print_job_elapsed);
  gcode.process_subcommands_now(cmd);

  TERN_(DEBUG_POWER_LOSS_RECOVERY, marlin_debug_flags = old_flags);
}

#endif