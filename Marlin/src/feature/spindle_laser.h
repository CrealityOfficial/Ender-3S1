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
#pragma once

/**
 * feature/spindle_laser.h
 * Support for Laser Power or Spindle Power & Direction
 */

#include "../inc/MarlinConfig.h"

#include "spindle_laser_types.h"

#if USE_BEEPER
  #include "../libs/buzzer.h"
#endif

// Inline laser power
#include "../module/planner.h"

#if ENABLED(IIC_BL24CXX_EEPROM)
  #include "../libs/BL24CXX.h"
#endif

#include "../../Configuration.h"


#define PCT_TO_PWM(X) ((X) * 255 / 100)
#define PCT_TO_SERVO(X) ((X) * 180 / 100)

#ifndef SPEED_POWER_INTERCEPT
  #define SPEED_POWER_INTERCEPT 0
#endif

// Laser/Cutter operation mode
enum CutterMode : int8_t {
  CUTTER_MODE_ERROR = -1,
  CUTTER_MODE_STANDARD,     // M3 power is applied directly and waits for planner moves to sync.
  CUTTER_MODE_CONTINUOUS,   // M3 or G1/2/3 move power is controlled within planner blocks, set with 'M3 I', cleared with 'M5 I'.
  CUTTER_MODE_DYNAMIC       // M4 laser power is proportional to the feed rate, set with 'M4 I', cleared with 'M5 I'.
};

class SpindleLaser {
public:
  static constexpr float
    min_pct = TERN(CUTTER_POWER_RELATIVE, 0, TERN(SPINDLE_FEATURE, round(100.0f * (SPEED_POWER_MIN) / (SPEED_POWER_MAX)), SPEED_POWER_MIN)),
    max_pct = TERN(SPINDLE_FEATURE, 100, SPEED_POWER_MAX);

  static const inline uint8_t pct_to_ocr(const_float_t pct) { return uint8_t(PCT_TO_PWM(pct)); }

  static CutterMode cutter_mode;

  // cpower = configured values (e.g., SPEED_POWER_MAX)

  // Convert configured power range to a percentage
  static const inline uint8_t cpwr_to_pct(const cutter_cpower_t cpwr) {
    constexpr cutter_cpower_t power_floor = TERN(CUTTER_POWER_RELATIVE, SPEED_POWER_MIN, 0),
                              power_range = SPEED_POWER_MAX - power_floor;
    return cpwr ? round(100.0f * (cpwr - power_floor) / power_range) : 0;
  }

  // Convert a cpower (e.g., SPEED_POWER_STARTUP) to unit power (upwr, upower),
  // which can be PWM, Percent, Servo angle, or RPM (rel/abs).
  static const inline cutter_power_t cpwr_to_upwr(const cutter_cpower_t cpwr) { // STARTUP power to Unit power
    const cutter_power_t upwr = (
      #if ENABLED(SPINDLE_FEATURE)
        // Spindle configured values are in RPM
        #if CUTTER_UNIT_IS(RPM)
          cpwr                            // to RPM
        #elif CUTTER_UNIT_IS(PERCENT)     // to PCT
          cpwr_to_pct(cpwr)
        #elif CUTTER_UNIT_IS(SERVO)       // to SERVO angle
          PCT_TO_SERVO(cpwr_to_pct(cpwr))
        #else                             // to PWM
          PCT_TO_PWM(cpwr_to_pct(cpwr))
        #endif
      #else
        // Laser configured values are in PCT
        #if CUTTER_UNIT_IS(PWM255)
          PCT_TO_PWM(cpwr)
        #else
          cpwr                            // to RPM/PCT
        #endif
      #endif
    );
    return upwr;
  }

  static const cutter_power_t mpower_min() { return cpwr_to_upwr(SPEED_POWER_MIN); }
  static const cutter_power_t mpower_max() { return cpwr_to_upwr(SPEED_POWER_MAX); }

  #if ENABLED(LASER_FEATURE)
    static cutter_test_pulse_t testPulse;                 // (ms) Test fire pulse duration
    static uint8_t last_block_power;                      // Track power changes for dynamic power
    static feedRate_t feedrate_mm_m, last_feedrate_mm_m;  // (mm/min) Track feedrate changes for dynamic power
    static inline bool laser_feedrate_changed() {
      if (last_feedrate_mm_m != feedrate_mm_m) { last_feedrate_mm_m = feedrate_mm_m; return true; }
      return false;
    }
  #endif

  static bool isReady;                    // Ready to apply power setting from the UI to OCR
  static uint8_t power;

  #if ENABLED(MARLIN_DEV_MODE)
    static cutter_frequency_t frequency;  // Set PWM frequency; range: 2K-50K
  #endif

  static cutter_power_t menuPower,        // Power as set via LCD menu in PWM, Percentage or RPM
                        unitPower;        // Power as displayed status in PWM, Percentage or RPM

  static void init();

  #if ENABLED(MARLIN_DEV_MODE)
    static inline void refresh_frequency() { set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), frequency); }
  #endif

  // Modifying this function should update everywhere
  static inline bool enabled(const cutter_power_t opwr) { return opwr > 0; }
  static inline bool enabled() { return enabled(power); }

  static void apply_power(const uint8_t inpow);

  FORCE_INLINE static void refresh() { apply_power(power); }
  FORCE_INLINE static void set_power(const uint8_t upwr) { power = upwr; refresh(); }

  #if ENABLED(SPINDLE_LASER_PWM)

    private:

    static void _set_ocr(const uint8_t ocr);

    public:

    static void set_ocr(const uint8_t ocr);
    static inline void set_ocr_power(const uint8_t ocr) { power = ocr; set_ocr(ocr); }
    static void ocr_off();
    // Used to update output for power->OCR translation
    static inline uint8_t upower_to_ocr(const cutter_power_t upwr) {
      return (
        #if CUTTER_UNIT_IS(PWM255)
          uint8_t(upwr)
        #elif CUTTER_UNIT_IS(PERCENT)
          pct_to_ocr(upwr)
        #else
          uint8_t(pct_to_ocr(cpwr_to_pct(upwr)))
        #endif
      );
    }

    // Correct power to configured range
    static inline cutter_power_t power_to_range(const cutter_power_t pwr) {
      return power_to_range(pwr, (
        #if CUTTER_UNIT_IS(PWM255)
          0
        #elif CUTTER_UNIT_IS(PERCENT)
          1
        #elif CUTTER_UNIT_IS(RPM)
          2
        #else
          #error "CUTTER_UNIT_IS(unknown)"
        #endif
      ));
    }

    static inline cutter_power_t power_to_range(const cutter_power_t pwr, const uint8_t unitPwr) {
      if (pwr <= 0) return 0;
      cutter_power_t upwr;
      switch (unitPwr) {
        case 0:                                                 // PWM
          upwr = cutter_power_t(
              (pwr < pct_to_ocr(min_pct)) ? pct_to_ocr(min_pct) // Use minimum if set below
            : (pwr > pct_to_ocr(max_pct)) ? pct_to_ocr(max_pct) // Use maximum if set above
            :  pwr
          );
          break;
        case 1:                                                 // PERCENT
          upwr = cutter_power_t(
              (pwr < min_pct) ? min_pct                         // Use minimum if set below
            : (pwr > max_pct) ? max_pct                         // Use maximum if set above
            :  pwr                                              // PCT
          );
          break;
        case 2:                                                 // RPM
          upwr = cutter_power_t(
              (pwr < SPEED_POWER_MIN) ? SPEED_POWER_MIN         // Use minimum if set below
            : (pwr > SPEED_POWER_MAX) ? SPEED_POWER_MAX         // Use maximum if set above
            : pwr                                               // Calculate OCR value
          );
          break;
        default: break;
      }
      return upwr;
    }

  #endif // SPINDLE_LASER_PWM

  // Enable laser output.
  // If we are in standard mode and no power was spec'd set it to the startup value.
  // With any inline mode set the power to the current one from the last cutter.power value.
  static inline void set_enabled(const bool enable) {
    switch (cutter_mode) {
      case CUTTER_MODE_STANDARD:
        set_power(enable ? TERN(SPINDLE_LASER_PWM, (power ?: (unitPower ? upower_to_ocr(cpwr_to_upwr(SPEED_POWER_STARTUP)) : 0)), 255) : 0);
        break;
      case CUTTER_MODE_CONTINUOUS:
      case CUTTER_MODE_DYNAMIC:
        TERN_(LASER_FEATURE, set_inline_enabled(enable));
        // fallthru
      case CUTTER_MODE_ERROR: // Error mode, no enable and kill power.
        apply_power(0);
    }
  }

  // Wait for spindle to spin up or spin down
  static inline void power_delay(const bool on) {
    safe_delay(on ? SPINDLE_LASER_POWERUP_DELAY : SPINDLE_LASER_POWERDOWN_DELAY);
  }

  #if ENABLED(LASER_FEATURE)
    static inline uint8_t calc_dynamic_power() {
      if (feedrate_mm_m > 65535) return 255;         // Too fast, go always on
      uint16_t rate = uint16_t(feedrate_mm_m);       // 32 bits from the G-code parser float input
      rate >>= 8;                                    // Take the G-code input e.g. F40000 and shift off the lower bits to get an OCR value from 1-255
      return uint8_t(rate);
    }
  #endif

  #if ENABLED(SPINDLE_CHANGE_DIR)
    static void set_reverse(const bool reverse);
    static bool is_reverse() { return READ(SPINDLE_DIR_PIN) == SPINDLE_INVERT_DIR; }
  #else
    static inline void set_reverse(const bool) { cutter_mode = CUTTER_MODE_DYNAMIC; }
    static bool is_reverse() { return false; }
  #endif

  #if ENABLED(AIR_EVACUATION)
    static void air_evac_enable();         // Turn On Cutter Vacuum or Laser Blower motor
    static void air_evac_disable();        // Turn Off Cutter Vacuum or Laser Blower motor
    static void air_evac_toggle();         // Toggle Cutter Vacuum or Laser Blower motor
    static inline bool air_evac_state() {  // Get current state
      return (READ(AIR_EVACUATION_PIN) == AIR_EVACUATION_ACTIVE);
    }
  #endif

  #if ENABLED(AIR_ASSIST)
    static void air_assist_enable();         // Turn on air assist
    static void air_assist_disable();        // Turn off air assist
    static void air_assist_toggle();         // Toggle air assist
    static inline bool air_assist_state() {  // Get current state
      return (READ(AIR_ASSIST_PIN) == AIR_ASSIST_ACTIVE);
    }
  #endif

  static inline void disable() { isReady = false; set_enabled(false); }

  #if HAS_LCD_MENU
    static inline void enable_with_dir(const bool reverse) {
      isReady = true;
      const uint8_t ocr = TERN(SPINDLE_LASER_PWM, upower_to_ocr(menuPower), 255);
      if (menuPower)
        power = ocr;
      else
        menuPower = cpwr_to_upwr(SPEED_POWER_STARTUP);
      unitPower = menuPower;
      set_reverse(reverse);
      set_enabled(true);
    }
    FORCE_INLINE static void enable_forward() { enable_with_dir(false); }
    FORCE_INLINE static void enable_reverse() { enable_with_dir(true); }
    FORCE_INLINE static void enable_same_dir() { enable_with_dir(is_reverse()); }

    #if ENABLED(SPINDLE_LASER_PWM)
      static inline void update_from_mpower() {
        if (isReady) power = upower_to_ocr(menuPower);
        unitPower = menuPower;
      }
    #endif

    #if ENABLED(LASER_FEATURE)
      /**
       * Test fire the laser using the testPulse ms duration
       * Also fires with any PWM power that was previous set
       * If not set defaults to 80% power
       */
      static inline void test_fire_pulse() {
        TERN_(USE_BEEPER, buzzer.tone(30, 3000));
        enable_forward();                  // Turn Laser on (Spindle speak but same funct)
        delay(testPulse);                  // Delay for time set by user in pulse ms menu screen.
        disable();                         // Turn laser off
      }
    #endif

  #endif // HAS_LCD_MENU

  #if ENABLED(LASER_FEATURE)

    // Inline modes of all other functions; all enable planner inline power control
    static inline void set_inline_enabled(const bool enable) { planner.laser_inline.status.isEnabled = enable; }

    // Set the power for subsequent movement blocks
    static void inline_power(const cutter_power_t upwr) {
      unitPower = menuPower = upwr;
      TERN(SPINDLE_LASER_PWM, planner.laser_inline.power = upower_to_ocr(upwr), planner.laser_inline.power = upwr > 0 ? 255 : 0);
    }

  #endif  // LASER_FEATURE

  static inline void kill() { disable(); }
};

extern SpindleLaser cutter;


#if HAS_CUTTER // 107011  -20210907

enum device_header{
	DEVICE_UNKNOWN=0xff, //unknow device
	DEVICE_LASER=1, 	 //laser device
  DEVICE_FDM,        //fdm device
};

enum laser_device_range{
 	LASER_MIN_X = 0,
	LASER_MIN_Y,
	LASER_MAX_X,
	LASER_MAX_Y,
};



// #include "../HAL/STM32F1/timers.h"
#include HAL_PATH(../HAL, timers.h)

class spindle_laser_soft_pwm
{
  private:
    device_header current_device=DEVICE_UNKNOWN; // 当前设备 激光/FDM
    bool need_read_gcode_range = false;
    float laser_range[4]={0.0}; //范围坐标 minx, miny, maxx, maxy
    
  public:
    const char laser_cmp_info[4][6]={"MINX:", "MINY:", "MAXX:", "MAXY:"};
    float pause_before_position_x=0, pause_before_position_y=0;
    bool  remove_card_before_is_printing = false; // 记录打印是否是打印状态， 卡移除恢复时判断是否需要执行M24恢复打印 
    bool  already_show_warning = false; // 是否已经显示安全警告界面
    bool  laser_printing = false; //是否正在雕刻 ture 正在雕刻， false 未雕刻。 用于打印界面正在雕刻是不允许选中设置
    uint16_t remain_time = 0; //模型的总的打印时间
    double laser_z_axis_high = 0; // Z轴高度, current_position.z 的值
    uint8_t power = 0;// 激光功率
    bool is_run_range=false;
  // 快速停止激光 等同命令 "M5 I"
  void quick_stop()
  {
    cutter.set_enabled(false);                  
    cutter.cutter_mode = CUTTER_MODE_STANDARD;  
    cutter.apply_power(0);
  }

  //107011 -20210926
  //从EEPROM中读取当前设备
  void get_device_form_eeprom()
  {
    uint8_t buff[2]={0};
    BL24CXX::read(LASER_FDM_ADDR, &buff[0], 1);
    //SERIAL_ECHOLNPAIR("get_device_form_eeprom", buff[0]);
    if((device_header)buff[0]==DEVICE_LASER || (device_header)buff[0]==DEVICE_FDM){
      current_device = (device_header)buff[0];
    }else{
      current_device = DEVICE_UNKNOWN;
    }
    // SERIAL_ECHOPAIR("\nget_device_form_eeprom()=", current_device);
  }

  //从eeprom中读取Z轴高度 current_position.z 
  double get_z_axis_high_form_eeprom()
  {    
    uint8_t buff[3]={0};
    uint16_t data=0;
    BL24CXX::read(LASER_Z_AXIS_HIGH_ADDR, &buff[0], 2);

    data = buff[0];
    data = (data<<8)|buff[1];

    if(data == 0xFFFF) data = 0;
    
    //>LASER_Z_AXIS_HIGHT_MAX 判断为无效数据
    if(data>LASER_Z_AXIS_HIGH_MAX*100) data = LASER_Z_AXIS_HIGH_MAX*100;

    laser_z_axis_high = data/100.0;
    // SERIAL_ECHOPAIR("\nget_z_axis_high_form_eeprom()=", laser_z_axis_high);
    return data/100.0;
  }

  //107011 -20210926
  //保存当前使用的设备到EEPROM中
  void save_device_to_eeprom()
  {
    BL24CXX::write(LASER_FDM_ADDR, (uint8_t *)&current_device, 1);
    delay(10);
    //SERIAL_ECHOLNPAIR("save_device_to_eeprom", buff[0]);
    //SERIAL_ECHOPAIR("\nsave_device_to_eeprom()=", current_device);
  }

  //保存Z轴高度到EEPROM
  void save_z_axis_high_to_eeprom(float data)
  {
    uint8_t buff[3]={0};
    uint16_t high = data*100;// 放大100倍保存

    if(high >LASER_Z_AXIS_HIGH_MAX*100) high = LASER_Z_AXIS_HIGH_MAX*100;

    buff[0] = (high>>8)&0xff;
    buff[1] = high&0xff;

    BL24CXX::write(LASER_Z_AXIS_HIGH_ADDR, &buff[0], 2);
    delay(10);
    laser_z_axis_high = data;
    //SERIAL_ECHOPAIR("\nlaser_z_axis_high=", laser_z_axis_high);
  }


  //读取当前设备
  device_header get_current_device(void)
  {
    return current_device;
  }

  //设置当前设备
  void set_current_device(device_header dev)
  {
    current_device = dev;
    save_device_to_eeprom(); //107011 -20210926
  }

  bool is_laser_device(void)
  {
    return current_device == DEVICE_LASER;
  }
  bool is_unknown_device(void)
  {
    return current_device == DEVICE_UNKNOWN;
  }
  
	//设置 读取gcode的范围
 	void set_read_gcode_range_on(void)
 	{
 		need_read_gcode_range = true;
 	}

//设置 不读取gcode的范围
 	void set_read_gcode_range_off(void)
 	{
 		need_read_gcode_range = false;
 	}

  bool is_read_gcode_range_on(void)
  {
    return need_read_gcode_range==true;
  }

	void set_laser_range(laser_device_range index, float value)
 	{
 		laser_range[index] = value;
	}

	float get_laser_range(laser_device_range index)
	{
		return laser_range[index];
	}

  

  //0-1000 比例转换为0-255
  uint8_t power16_to_8(uint16_t power)
  {
    double p;
    if( power==0) return 0;

    p = (uint8_t)((power/1000.0)*255);
    p = (p>0)?(p):(1);

    return p;
  }

	// 复位保存的gcode范围的缓存
	void reset_data(void)
 	{
 		laser_range[LASER_MIN_X] = 0.0;
		laser_range[LASER_MIN_Y] = 0.0;
		laser_range[LASER_MAX_X] = 0.0;
		laser_range[LASER_MAX_Y] = 0.0;
 	}

	void soft_pwm_init()
	{
    _SET_OUTPUT(LASER_SOFT_PWM_PIN);
		laser_timer_soft_pwm_init(LASER_TIMER_FREQUENCY);
	}
	
	// 设置并启动timer3 的pwm
  	void laser_power_start(const uint8_t power)
  	{	//0 - 255
      //SERIAL_ECHOLNPAIR("_laser.h 424 =", power);
    	laser_timer_soft_pwm_start(power);
  	}

	// 将pwm占空比设置为1，以最弱的激光输出
  	void laser_power_stop(void)
  	{
   	 	laser_timer_soft_pwm_stop();
  	}

    // 关闭激光PWM输出， 暂停timer3中断，无激光输出
    void laser_power_close()
    {
      laser_timer_soft_pwm_close();
    }

    //打开激光pwm，以最弱的激光输出
    void laser_power_open()
    {
      laser_power_start(1);
    }
};

extern class spindle_laser_soft_pwm laser_device;



// //set PWM 
// // : power 0-255
// void laser_power_start(const uint8_t power);

// // stop pwm
// void laser_power_stop(void);


#endif //#if HAS_CUTTER



