#include "PRE01_Power_loss.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"

#include "../powerloss.h"
#include "../../module/servo.h"
#include "../../module/temperature.h"
#include "../..//gcode/gcode.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
// #include "../../lcd/dwin/i2c_eeprom.h"
#include "../../libs/BL24CXX.h"
#include "../../module/printcounter.h"

#include "../../core/serial.h" 

//#include "../../HAL/DUE/watchdog.h"

// #include "../../HAL/STM32/HAL.h"

#ifdef CREALITY_ENDER3_2021

#define ENSURE_POWER_SUPPLY_STABLE  (5000)  //10s
#define POWER_CHECK_REFERENCE_VOL   (3.30)  //3.3v
#define POWER_DOWN_THRESHOLD        (1.87)  //对应系统的20V
#define CHECK_TIMES                 (1)     //CHECK_TIMES 次以后V才认为是断电  否则认为是抖动不做处理

PRE01PowerLoss *PRE01PowerLoss::Instance = new PRE01PowerLoss();


// Report the logical position for a given machine position
void my_report_logical_position(const xyze_pos_t &rpos) {   //report_logical_position(info->current_position);
  const xyze_pos_t lpos = rpos.asLogical();
  SERIAL_ECHOPAIR_P(X_LBL, lpos.x, SP_Y_LBL, lpos.y, SP_Z_LBL, lpos.z, SP_E_LBL, lpos.e);
}

void minimize_power_consumption()
{
    Temperature::disable_all_heaters();
    disable_all_steppers();        /*Turn off the motor to ensure enough power to write to the EEPROM */
    // timer_pause(STEP_TIMER_DEV);   /*Stop steper timer*/
    // timer_pause(TEMP_TIMER_DEV);   /*Stop temp timer*/
    timer_disable_all();
    rcc_clk_disable(RCC_TIMER5);
    rcc_clk_disable(RCC_TIMER2);
    rcc_clk_disable(RCC_ADC1);
    rcc_clk_disable(RCC_ADC2);
    rcc_clk_disable(RCC_ADC3);
    rcc_clk_disable(RCC_USART1);
    rcc_clk_disable(RCC_USART2);
    rcc_clk_disable(RCC_USART3);
    rcc_clk_disable(RCC_DMA1);
    rcc_clk_disable(RCC_DMA2);
    rcc_clk_disable(RCC_GPIOB);
    rcc_clk_disable(RCC_GPIOC);
    rcc_clk_disable(RCC_GPIOD);
    
    //DISABLE_ISRS(); 
    //WDT_Disable(WDT);   // Make sure to completely disable the Watchdog
}

static void save()
 {   
      // Set Head and Foot to matching non-zero values
    if (!++recovery.info.valid_head) ++recovery.info.valid_head; // non-zero in sequence
    
    // Machine state
    recovery.info.current_position = current_position;
    #if HAS_HOME_OFFSET
      recovery.info.home_offset = home_offset;
    #endif
    #if HAS_POSITION_SHIFT
      recovery.info.position_shift = position_shift;
    #endif
    recovery.info.feedrate = uint16_t(feedrate_mm_s * 60.0f);

    #if EXTRUDERS > 1
      recovery.info.active_extruder = active_extruder;
    #endif

    #if DISABLED(NO_VOLUMETRICS)
      recovery.info.flag.volumetric_enabled = parser.volumetric_enabled;
      #if EXTRUDERS > 1
        for (int8_t e = 0; e < EXTRUDERS; e++)   recovery.info.filament_size[e] = planner.filament_size[e];
      #else
         if (parser.volumetric_enabled) recovery.info.filament_size[0] = planner.filament_size[active_extruder]; 
      #endif
    #endif   
 
    #if EXTRUDERS
      HOTEND_LOOP() recovery.info.target_temperature[e] = thermalManager.temp_hotend[e].target;
      recovery.info.target_temperature[0] = thermalManager.temp_hotend[0].target;  //20210812_rock
    #endif

    #if HAS_HEATED_BED
      recovery.info.target_temperature_bed = thermalManager.temp_bed.target;
    #endif
  
    #if FAN_COUNT
      COPY(recovery.info.fan_speed, thermalManager.fan_speed);
    #endif

    #if HAS_LEVELING
      recovery.info.flag.leveling = planner.leveling_active;   //info.flag.leveling
      recovery.info.fade = (
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          planner.z_fade_height
        #else
          0
        #endif
      );
    #endif

    #if ENABLED(GRADIENT_MIX)
      memcpy(&recovery.info.gradient, &mixer.gradient, sizeof(recovery.info.gradient));
    #endif

    #if ENABLED(FWRETRACT)
      COPY(recovery.info.retract, fwretract.current_retract);
      recovery.info.retract_hop = fwretract.current_hop;
    #endif

    // Relative axis modes
    recovery.info.axis_relative = gcode.axis_relative;

    // Elapsed print job time
    recovery.info.print_job_elapsed = print_job_timer.duration();

    //recovery.info.sd_printing_flag = true;

    //if (!IS_SD_PRINTING()) info.valid_head = 0;
    recovery.info.valid_foot = recovery.info.valid_head;
    
    #ifdef EEPROM_PLR      
    //my_report_logical_position(recovery.info.current_position);
      BL24CXX::write(PLR_ADDR, (uint8_t*)&recovery.info, sizeof(recovery.info));
    #else   //
    write();
    #endif
}

//Check power supply and write to EEPROM
void PRE01PowerLoss::DoSomeThing()
{
  //不在打印中或者刚开机5s中就不执行
    //if (!IS_SD_PRINTING() || millis() < ENSURE_POWER_SUPPLY_STABLE)return; // 为了暂停状态也要断电续打，所以将不在打印中断电必须取消
    if (millis() < ENSURE_POWER_SUPPLY_STABLE) return;
    float raw = Info.raw;
    float voltage = (raw > 0) ? ((raw / (1024 + 1)) * POWER_CHECK_REFERENCE_VOL) : 2; //Ten-bit ADC acquisition 1 times
    static uint8_t Count = 0;
    static bool power_loss_flag = false;     
    if ( voltage <= POWER_DOWN_THRESHOLD)
    {
      //OUT_WRITE(BACKPOWER_CTRL_PIN, HIGH);
        ++Count;
        /**/
        block_t *current =&planner.block_buffer[BLOCK_MOD(planner.block_buffer_tail - 1)];
        if ( current )
        {
            // SERIAL_ECHOLIST("~~~~~~~~~~~~~~~~~~~~~~~~~~~ nSD_pos ", recovery.info.sdpos, "\r\n");
            recovery.info.sdpos = current->sdpos;  //断电续打的指令序列数
        }
    }
    else
        Count = 0;

    if ( Count == CHECK_TIMES && !power_loss_flag)
    {
       OUT_WRITE(BACKPOWER_CTRL_PIN, HIGH);
       delay(50); 
      //minimize_power_consumption();  //绝对不能要
        power_loss_flag = true;
        save();
        delay(100);
        HAL_reboot(); // Reboot the board 
    }
    else 
    {
      // save();
      // delay(50); 
      // nvic_sys_reset();   // MCU复位进入BOOTLOADER
    }
       
}

/* Switch PID according to heating bed voltage */
void switch_pid(Voltage_e __switch)
{
    switch(__switch)
    {
        case HB110V:
          SERIAL_ECHO_MSG("cur supply vol 110V \r\n");
          break;

        default:
        case HB220V:
          SERIAL_ECHO_MSG("cur supply vol 220V or default \r\n");
          thermalManager.temp_bed.pid.Kp = DEFAULT_bedKp;
          thermalManager.temp_bed.pid.Ki = scalePID_i(DEFAULT_bedKi);
          thermalManager.temp_bed.pid.Kd = scalePID_d(DEFAULT_bedKd);
          break;
    }
} 

#endif