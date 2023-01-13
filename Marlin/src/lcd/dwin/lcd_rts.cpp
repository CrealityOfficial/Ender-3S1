#include <wstring.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
// #include <libmaple/usart.h>
#include "lcd_rts.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../module/settings.h"
#include "../../core/serial.h"
#include "../../core/macros.h"
#include "../fontutils.h"
#include "../marlinui.h"
#include "../../sd/cardreader.h"
#include "../../feature/babystep.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"
#include "../../module/probe.h"

#include "../../feature/bedlevel/abl/abl.h"

#include "../../libs/duration_t.h"

#if ENABLED(BLTOUCH)
  #include "../../module/endstops.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #include "../../feature/runout.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../feature/powerloss.h"
#elif ENABLED(CREALITY_POWER_LOSS)
  #include "../../feature/PRE01_Power_loss/PRE01_Power_loss.h"
#endif

#if HAS_CUTTER
#include "../../../src/feature/spindle_laser.h"
#endif


#ifdef LCD_SERIAL_PORT
  #define LCDSERIAL LCD_SERIAL
#elif SERIAL_PORT_2
  #define LCDSERIAL MYSERIAL2
#endif

#if ENABLED(RTS_AVAILABLE)
/*******************************类定义*********************************/
RTSSHOW rtscheck;
/********************************END***********************************/


/*******************************外部声明*****************************/
extern CardReader card;

/**********************************END**********************************/


/*******************************可选择性功能*****************************/
#if ENABLED(HAS_MENU_RESET_WIFI)
  //WIFI状态
  unsigned char WIFI_STATE = INITIAL;
#endif


//错误状态
char errorway = 0;
char errornum = 0;
char home_errornum  = 0; 
char error_sd_num = 0;

//开始打印状态标志位
bool StartPrint_flag = false;

//babystep偏移参数，Z轴偏移参数
#if ENABLED(BABYSTEPPING)
  float zprobe_zoffset;
  float last_zoffset = 0.0;
#endif

int power_off_type_yes = 0;

//运动相关参数设置
const float manual_feedrate_mm_m[] = {50 * 60, 50 * 60, 4 * 60, 60};
// const float manual_feedrate_mm_m[] = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

//喷头默认PID
float default_nozzle_ptemp = DEFAULT_Kp;
float default_nozzle_itemp = DEFAULT_Ki;
float default_nozzle_dtemp = DEFAULT_Kd;

//热床默认PID
float default_hotbed_ptemp = DEFAULT_bedKp;
float default_hotbed_itemp = DEFAULT_bedKi;
float default_hotbed_dtemp = DEFAULT_bedKd;

//打印进度条
uint8_t startprogress = 0;
//读取SD卡GCODE文件名的结构体参数
CRec CardRecbuf; 
// float pause_e = 0;
//SD卡打印是否暂停，true需要检测暂停，false已经暂停完成
bool sdcard_pause_check = true;
//暂停的动作就是回到 X0,Y0处
bool pause_action_flag = false;
// bool print_preheat_check = false;
// bool probe_offset_flag = false;

//更换耗材时的设定温度
float ChangeFilamentTemp = 200;
int heatway = 0;

//下一次更新数据的时间
millis_t next_rts_update_ms      = 0;
// millis_t next_shutdown_update_ms = 0;
// unsigned int count_ms = 0;
// unsigned long count_lcd_down = 0;
// bool flag_lcd_down = false;

//上一次喷头的温度
int last_target_temperature[4] = {0};
//上一次热床的温度
int last_target_temperature_bed;

char waitway = 0;

//当前的页面序列
int change_page_font = 1;
// int recnum = 0;
unsigned char Percentrecord = 0;   //SD卡打印百分比
// represents to update file list
//SD卡文件列表更新，标志位
bool CardUpdate = false;  

//当前的语言标志位
uint8_t lang = 2; 
// represents SD-card status, true means SD is available, false means opposite.
//表示 SD 卡状态，true 表示 SD 可用，false 表示相反。
bool lcd_sd_status;

// char Checkfilenum = 0;
int FilenamesCount = 0;

char cmdbuf[20] = {0};

//耗材加载的默认长度
float FilamentLOAD = 10;

//耗材卸载的默认长度
float FilamentUnLOAD = 10;

// 1 for 10mm, 2 for 1mm, 3 for 0.1mm
//移动轴的单位选择标志 1 代表 10mm，2 代表 1mm，3 代表 0.1mm
unsigned char AxisUnitMode;

//移动轴的每个单位参数
float axis_unit = 10.0;
// bool LEDStatus = true;
//更新迪文屏幕的数据变量时间间隔
int Update_Time_Value = 0;
//断电续打，迪文屏显示逻辑标志位
bool PoweroffContinue = false;
char commandbuf[30];

// uint16_t remain_time = 0;

bool home_flag = false;
// bool G29_flag = false;
// bool AutohomeZflag = false;
// bool home_count = true;
// unsigned char Count_first = 0;
// unsigned char Count_probe = 0; 
// bool flag_over_shutdown = false;
// bool flag_counter_printover_to_shutdown = false;

//保存暂停时喷头的温度
int temphot = 0;  
//选中文件开始打印标志位
bool rts_start_print = false;  

//辅助调平的点
const int manual_level_5position[5][2] = MANUALL_BED_LEVEING_5POSITION;

enum{
  PREHEAT_PLA = 0,
  PREHEAT_ABS = 1,
};

int temp_preheat_nozzle = 0, temp_preheat_bed = 0;
uint8_t preheat_flag = PREHEAT_PLA; // 0=PLA， 1=ABS


/*************************************END***************************************/

//以manual_feedrate_mm_m的默认设置速度，移动到current_position的位置
inline void RTS_line_to_current(AxisEnum axis)
{
  if (!planner.is_full())
  {
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
  }
}

//设置发送和接收串口屏的帧头数据
RTSSHOW::RTSSHOW(void)
{
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  memset(databuf, 0, sizeof(databuf));
}

//SD卡的检测引脚，具有防抖功能
bool RTSSHOW::RTS_SD_Detected(void)
{
  static bool last;
  static bool state;
  static bool flag_stable;
  static uint32_t stable_point_time;

  bool tmp = IS_SD_INSERTED();

  if(tmp != last)
  {
    flag_stable = false;
  }
  else
  {
    if(!flag_stable)
    {
      flag_stable = true;
      stable_point_time = millis();
    }
  }

  if(flag_stable)
  {
    if(millis() - stable_point_time > 30)
    {
      state = tmp;
    }
  }

  last = tmp;

  return state;
}

//迪文屏初始化SD卡
void RTSSHOW::RTS_SDCardInit(void)
{
  if(RTS_SD_Detected())
  {
    if(!card.isMounted()) card.mount();
  }
  if(CardReader::flag.mounted)
  {
    uint16_t fileCnt = card.get_num_Files();  //获取SD卡中Gcode的总数
    card.getWorkDirName();  //获取当前的SD卡目录，如果不是根文件目录则，返回上一级目录
    if(card.filename[0] != '/')
    {
      card.cdup();
    }

    int addrnum = 0; 
    int num = 0;  //当前SD卡的文件序列号
    for(uint16_t i = 0;(i < fileCnt) && (i < (MaxFileNumber + addrnum));i ++)
    {
      card.selectFileByIndex(fileCnt - 1 - i);  //返回对应序列号的SD卡文件的长文件名和短文件名
      char *pointFilename = card.longFilename;  //长文件名
      int filenamelen = strlen(card.longFilename);  //长文件名的长度
      int j = 1;
      //轮询长文件名的整个长度，是否包含.gcode 或者 .GCODE 结尾的文件
      while((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j ++) < filenamelen)); 
      if(j >= filenamelen)
      {
        addrnum++;
        continue;
      }

      if (j >= TEXTBYTELEN)
      {
        strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
        card.longFilename[TEXTBYTELEN - 1] = '\0';
        j = TEXTBYTELEN - 1;
      }

      strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename, j); //复制长文件保存到CardRecbuf结构体变量中

      strcpy(CardRecbuf.Cardfilename[num], card.filename);  //复制短文件名并保存到CardRecbuf结构体变量中
      CardRecbuf.addr[num] = FILE1_TEXT_VP + (num * 20);
      RTS_SndData(CardRecbuf.Cardshowfilename[num], CardRecbuf.addr[num]);
      CardRecbuf.Filesum = (++num);
    }
    //清除剩下的文件名显示地址的数据
    for(int j = CardRecbuf.Filesum;j < MaxFileNumber;j ++)
    {
      CardRecbuf.addr[j] = FILE1_TEXT_VP + (j * 20);
      RTS_SndData(0, CardRecbuf.addr[j]);
    }
    //清除打印主界面的文件名显示
    for(int j = 0;j < 20;j ++)
    {
      // clean print file
      RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
    }
    // lcd_sd_status = IS_SD_INSERTED();
    lcd_sd_status = RTS_SD_Detected();  //SD卡当前的状态
  }
  else  //SD卡挂在失败
  {
    // clean filename Icon
    for(int j = 0;j < MaxFileNumber;j ++)
    {
      // clean filename Icon
      for(int i = 0;i < TEXTBYTELEN;i ++)
      {
        RTS_SndData(0, CardRecbuf.addr[j] + i);
      }
    }
    memset(&CardRecbuf, 0, sizeof(CardRecbuf));
  }
}

void RTSSHOW::RTS_SDCardUpate(void)
{
  const bool sd_status = RTS_SD_Detected();
  if (sd_status != lcd_sd_status)
  {
    if (sd_status)
    {
      // SD card power on
      RTS_SDCardInit();
    }
    else
    {
      // SERIAL_ECHOLNPAIR("\r\ncard 3");
      // if(PoweroffContinue == true)
      // {
      //   return;
      // }
      // else
      // {
        // SERIAL_ECHOLNPAIR("\r\ncard 4");
        card.release();
        for(int i = 0;i < CardRecbuf.Filesum;i ++)
        {
          for(int j = 0;j < 20;j ++)
          {
            RTS_SndData(0, CardRecbuf.addr[i] + j);
          }
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
        }

        for(int j = 0;j < 20;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        memset(&CardRecbuf, 0, sizeof(CardRecbuf));
      // }
    }
    lcd_sd_status = sd_status;
  }

  // represents to update file list
  if(CardUpdate && lcd_sd_status && RTS_SD_Detected())
  {
    for(uint16_t i = 0;i < CardRecbuf.Filesum;i ++)
    {
      // delay(1);
      RTS_SndData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
      RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
    }
    CardUpdate = false;
  }
}

void RTSSHOW::RTS_Init(void)
{
  // LCDSERIAL.begin(LCD_BAUDRATE);

  AxisUnitMode = 1;
  lang = language_change_font;
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    // bool zig = true;
    // int8_t inStart, inStop, inInc, showcount;
    // showcount = 0;
    // settings.load();
    // for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
    // {
    //   // away from origin
    //   if (zig)
    //   {
    //     inStart = 0;
    //     inStop = GRID_MAX_POINTS_X;
    //     inInc = 1;
    //   }
    //   else
    //   {
    //     // towards origin
    //     inStart = GRID_MAX_POINTS_X - 1;
    //     inStop = -1;
    //     inInc = -1;
    //   }
    //   zig ^= true;
    //   for (int x = inStart; x != inStop; x += inInc)
    //   {
    //     RTS_SndData(z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
    //     showcount++;
    //   }
    // }
  #endif
  last_zoffset = zprobe_zoffset = probe.offset.z;
  RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

  //显示语言选择界面图标
  for(int i = 0;i < 9;i ++)
  {
    RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
  }
  //选中当前的语言
  RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
  languagedisplayUpdate();
  // delay(100);

  last_target_temperature[0] = thermalManager.temp_hotend[0].target;
  last_target_temperature_bed = thermalManager.temp_bed.target;
  feedrate_percentage = 100;
  RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);

  /***************turn off motor*****************/
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  /***************transmit temperature to screen*****************/
  RTS_SndData(0, HEAD_SET_TEMP_VP);
  RTS_SndData(0, BED_SET_TEMP_VP);
  RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
  // delay(20);
  RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
  // delay(20);
  RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
  RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
  RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
  RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);

  RTS_SndData(planner.settings.max_feedrate_mm_s[0], MAX_VELOCITY_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[1], MAX_VELOCITY_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[2], MAX_VELOCITY_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[3], MAX_VELOCITY_EAXIS_DATA_VP);

  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[0], MAX_ACCEL_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[1], MAX_ACCEL_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[2], MAX_ACCEL_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[3], MAX_ACCEL_EAXIS_DATA_VP);

  RTS_SndData(planner.max_jerk.x * 100, MAX_JERK_XAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.y * 100, MAX_JERK_YAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.z * 100, MAX_JERK_ZAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.e * 100, MAX_JERK_EAXIS_DATA_VP);

  RTS_SndData(planner.settings.axis_steps_per_mm[0] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[1] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[2] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[3] * 10, MAX_STEPSMM_EAXIS_DATA_VP);

  RTS_SndData(PID_PARAM(Kp, 0) * 100, NOZZLE_TEMP_P_DATA_VP);
  RTS_SndData(unscalePID_i(PID_PARAM(Ki, 0)) * 100, NOZZLE_TEMP_I_DATA_VP);
  RTS_SndData(unscalePID_d(PID_PARAM(Kd, 0)) * 100, NOZZLE_TEMP_D_DATA_VP);
  RTS_SndData(thermalManager.temp_bed.pid.Kp * 100, HOTBED_TEMP_P_DATA_VP);
  RTS_SndData(unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100, HOTBED_TEMP_I_DATA_VP);
  RTS_SndData(unscalePID_d(thermalManager.temp_bed.pid.Kd) * 10, HOTBED_TEMP_D_DATA_VP);

  RTS_SndData( thermalManager.fan_speed[0] , PRINTER_FAN_SPEED_DATA_VP);  

  /***************transmit Fan speed to screen*****************/
  // turn off fans
  thermalManager.set_fan_speed(0, 0);
  // RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
  // RTS_SndData(1, PRINTER_LEDOPEN_TITLE_VP);
  // LEDStatus = true;
  // delay(5);
  // if(flag_over_shutdown)
  // {
  //   rtscheck.RTS_SndData(0, PRINTER_AUTO_SHUTDOWN_ICON_VP);
  // }
  // else
  // {
  //   rtscheck.RTS_SndData(1, PRINTER_AUTO_SHUTDOWN_ICON_VP);
  // }

  /*********transmit SD card filename to screen***************/
  // delay(200);
  RTS_SDCardInit();

  /***************transmit Printer information to screen*****************/
  RTS_SndData(MACVERSION, MACHINE_TYPE_ABOUT_TEXT_VP);
  RTS_SndData(SOFTVERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
  RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
  if(1 == lang)
  {
    RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
  }
  else
  {
    RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
  }

  // if(wifi_enable_flag)
  // {
  //   RTS_SndData(0, ADV_SETTING_WIFI_ICON_VP);
  // }
  // else
  // {
  //   RTS_SndData(1, ADV_SETTING_WIFI_ICON_VP);
  // }

  /**************************some info init*******************************/
  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(0, PRINT_REMAIN_TIME_HOUR_VP);
  RTS_SndData(0, PRINT_REMAIN_TIME_MIN_VP);

  RTS_SndData(1, PREHAEAT_NOZZLE_ICON_VP);
  RTS_SndData(1, PREHAEAT_HOTBED_ICON_VP);
  //rtscheck.RTS_SndData(1, FILAMENT_CONTROL_ICON_VP);

  rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
  change_page_font = 0;
  HAL_watchdog_refresh();
  for(startprogress = 0; startprogress <= 100; startprogress++)
  {
    rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
    HAL_watchdog_refresh();  
    delay(100);
  }
  HAL_watchdog_refresh();
  delay(500);
  // delay(500);
  // delay(500);
  // delay(500);

  Update_Time_Value = RTS_UPDATE_VALUE;
}


int RTSSHOW::RTS_RecData(void)
{
  static int recnum = 0;  //当前接收的数据偏移
  while((LCDSERIAL.available() > 0) && (recnum < SizeofDatabuf))
  {
    delay(1);
    databuf[recnum] = LCDSERIAL.read();
    if(databuf[0] == FHONE)
    {
      recnum++;
    }
    else if(databuf[0] == FHTWO)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      recnum += 2;
    }
    else if(databuf[0] == FHLENG)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = FHLENG;
      recnum += 3;
    }
    else if(databuf[0] == VarAddr_R)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = FHLENG;
      databuf[3] = VarAddr_R;
      recnum += 4;
    }
    else
    {
      recnum = 0;
    }
  }

  // receive nothing
  if(recnum < 1)
  {
    return -1;
  }
  else if((recdat.head[0] == databuf[0]) && (recdat.head[1] == databuf[1]) && (recnum > 2))
  {
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    // response for writing byte
    if((recdat.len == 0x03) && ((recdat.command == 0x82) || (recdat.command == 0x80)) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))
    {
      memset(databuf, 0, sizeof(databuf));
      recnum = 0;
      return -1;
    }
    else if(recdat.command == 0x83)
    {
      // response for reading the data from the variate
      recdat.addr = databuf[4];
      recdat.addr = (recdat.addr << 8) | databuf[5];
      recdat.bytelen = databuf[6];
      for(unsigned int i = 0; i < recdat.bytelen; i += 2)
      {
        recdat.data[i / 2] = databuf[7 + i];
        recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
      }
    }
    else if(recdat.command == 0x81)
    {
      // response for reading the page from the register
      recdat.addr = databuf[4];
      recdat.bytelen = databuf[5];
      for(unsigned int i = 0; i < recdat.bytelen; i ++)
      {
        recdat.data[i] = databuf[6 + i];
        // recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
      }
    }
  }
  else
  {
    memset(databuf, 0, sizeof(databuf));
    recnum = 0;
    // receive the wrong data
    return -1;
  }
  memset(databuf, 0, sizeof(databuf));
  recnum = 0;
  return 2;
}

void RTSSHOW::RTS_SndData(void)
{
  if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && (snddat.len >= 3))
  {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;

    // to write data to the register
    if(snddat.command == 0x80)
    {
      databuf[4] = snddat.addr;
      for(int i = 0;i <(snddat.len - 2);i ++)
      {
        databuf[5 + i] = snddat.data[i];
      }
    }
    else if((snddat.len == 3) && (snddat.command == 0x81))
    {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if(snddat.command == 0x82)
    {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for(int i =0;i <(snddat.len - 3);i += 2)
      {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if((snddat.len == 4) && (snddat.command == 0x83))
    {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    // usart_tx(LCDSERIAL.c_dev(), databuf, snddat.len + 3);
    // LCDSERIAL.flush();
    for(int i = 0;i < (snddat.len + 3); i ++)
    {
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }

    memset(&snddat, 0, sizeof(snddat));
    memset(databuf, 0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if(s.length() < 1)
  {
    return;
  }
  RTS_SndData(s.c_str(), addr, cmd);
}

void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = strlen(str);
  if(len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i = 0;i < len;i ++)
    {
      databuf[6 + i] = str[i];
    }
    for(int i = 0;i < (len + 6);i ++)
    {
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
  }
}

void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned char *str, unsigned long addr, unsigned char cmd) { RTS_SndData((char *)str, addr, cmd); }

void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == RegAddr_W)
  {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd) { RTS_SndData((unsigned long)n, addr, cmd); }

void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SDcard_Stop(void)
{
  // planner.synchronize();
  // card.endFilePrintNow();
  card.flag.abort_sd_printing = true;  //card.flag.abort_sd_printing
  queue.clear();
  if(home_flag) planner.synchronize();
   quickstop_stepper();
  // card.abortFilePrintNow();
  print_job_timer.stop();
  #if DISABLED(SD_ABORT_NO_COOLDOWN)
    thermalManager.disable_all_heaters();
  #endif
  print_job_timer.reset();
  thermalManager.setTargetHotend(0, 0);
  RTS_SndData(0, HEAD_SET_TEMP_VP);
  thermalManager.setTargetBed(0);
  RTS_SndData(0, BED_SET_TEMP_VP);
  temphot = 0;
  thermalManager.zero_fan_speeds();
  wait_for_heatup = wait_for_user = false;
  PoweroffContinue = false;

  // sd_printing_autopause = false;
  if(CardReader::flag.mounted)
  {
    #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
      card.removeJobRecoveryFile();
    #endif
  }

  // shut down the stepper motor.
  // queue.enqueue_now_P(PSTR("M84"));
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_VP);
  delay(2);
  RTS_SndData(0, PRINT_REMAIN_TIME_HOUR_VP);
  RTS_SndData(0, PRINT_REMAIN_TIME_MIN_VP);
  
  RTS_SndData(0, PRINT_TIME_HOUR_VP);
  RTS_SndData(0, PRINT_TIME_MIN_VP);

  for(int j = 0;j < 20;j ++)
  {
    // clean screen.
    RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
    // clean filename
    RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
  }
  planner.synchronize();
  RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
  change_page_font = 1;
  card.flag.abort_sd_printing = true;  //card.flag.abort_sd_printing
}


void RTSSHOW::RTS_HandleData(void)
{
  int Checkkey = -1;
  // for waiting
  if(waitway > 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for(int i = 0;Addrbuf[i] != 0;i++)
  {
    if(recdat.addr == Addrbuf[i])
    {
      if(Addrbuf[i] >= ChangePageKey)
      {
        Checkkey = i;
      }
      break;
    }
  }

  if(Checkkey < 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  // SERIAL_ECHOPAIR("\nCheckkey=", Checkkey, "recdat.data[0]=", recdat.data[0]);

  switch(Checkkey)
  {
    case MainEnterKey:
      if(recdat.data[0] == 1)
      {
        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        RTS_SDCardUpate();
        RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
        change_page_font = 2;
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        
        // if((!axis_is_trusted(X_AXIS)) || (!axis_is_trusted(Y_AXIS))){//
        if(axes_should_home()) {
          waitway = 4;
          queue.enqueue_one_P(PSTR("G28"));
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          change_page_font = 40;
        }else{
          RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
          change_page_font = 16;
          RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
          RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
          RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        }
        //RTS_SndData(1, FILAMENT_CONTROL_ICON_VP);
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
        change_page_font = 25;
        planner.synchronize();
        queue.enqueue_now_P(PSTR("G28\nG1 F200 Z0.0"));
        //RTS_SndData(1, AUTO_BED_LEVEL_TITLE_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
      }
      else if(recdat.data[0] == 5)
      {
        // card.flag.abort_sd_printing = true;  
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_VP);
        delay(2);
        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        RTS_SndData(0, PRINT_REMAIN_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_REMAIN_TIME_MIN_VP);

        print_job_timer.reset();
        // sd_printing_autopause = false;
        for(int j = 0;j < 20;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      else if(recdat.data[0] == 6)
      {
        waitway = 3;
        RTS_SndData(1, AUTO_BED_LEVEL_TITLE_VP);
        RTS_SndData(AUTO_BED_LEVEL_PREHEAT, AUTO_BED_PREHEAT_HEAD_DATA_VP);
        // RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
        // change_page_font = 26;
        rtscheck.RTS_SndData(0 , AUTO_LEVELING_PERCENT_DATA_VP);  
        // thermalManager.setTargetHotend(AUTO_BED_LEVEL_PREHEAT, 0);
        // //thermalManager.disable_all_heaters();
        // RTS_SndData(AUTO_BED_LEVEL_PREHEAT, HEAD_SET_TEMP_VP);
        if(thermalManager.temp_hotend[0].celsius < (AUTO_BED_LEVEL_PREHEAT - 5))
        {
          queue.enqueue_now_P(PSTR("G4 S40"));
        }

        if(axes_should_home())  queue.enqueue_one_P(PSTR("G28"));
        queue.enqueue_one_P(PSTR("G29"));
        //queue.enqueue_one_P(PSTR("G28\nG29"));

        RTS_SndData(0, MOTOR_FREE_ICON_VP);
      }
      else if(recdat.data[0] == 7)
      {
        if(errorway == 1)
        {

        }
        else if(errorway == 2)
        {
          // auto home fail
        }
        else if(errorway == 3)
        {
          // bed leveling fail
        }
        else if(errorway == 4) 
        {

        }
      }
      else if(recdat.data[0] == 8)
      {
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      else if(recdat.data[0] == 9)// 暂停/停止
      {
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        change_page_font = 11;
      }else if(recdat.data[0] == 0x0A)// 继续/停止
      {
        RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
        change_page_font = 13;
      }
      break;

    case AdjustEnterKey:
      if(recdat.data[0] == 1)
      {
        // thermalManager.fan_speed[0] ? RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP) : RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);

        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
      }
      else if(recdat.data[0] == 2)
      {
          // RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          // change_page_font = 10;
        if(card.isPrinting())
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
        // settings.save();
      }
      // else if(recdat.data[0] == 3)
      // {
      //   if(thermalManager.fan_speed[0])
      //   {
      //     RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
      //     thermalManager.set_fan_speed(0, 0);
      //   }
      //   else
      //   {
      //     RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
      //     thermalManager.set_fan_speed(0, 255);
      //   }
      // }
      // else if(recdat.data[0] == 4)
      // {
      //   if(LEDStatus)
      //   {
      //     RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
      //     digitalWrite(LED_CONTROL_PIN, HIGH);
      //     LEDStatus = false;
      //   }
      //   else
      //   {
      //     RTS_SndData(1, PRINTER_LEDOPEN_TITLE_VP);
      //     digitalWrite(LED_CONTROL_PIN, LOW);
      //     LEDStatus = true;
      //   }
      // }
      else if(recdat.data[0] == 5)
      {
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        change_page_font = 15;
      }
      // else if(recdat.data[0] == 6)
      // {
      //   if(flag_over_shutdown)
      //   {
      //     RTS_SndData(1, PRINTER_AUTO_SHUTDOWN_ICON_VP);
      //     flag_over_shutdown = false;
      //   }
      //   else
      //   {
      //     RTS_SndData(0, PRINTER_AUTO_SHUTDOWN_ICON_VP);
      //     flag_over_shutdown = true;
      //   }
      // }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
        settings.save();
      }
      //���ӵĶ��ϼ�⿪��ʹ�ܰ�ť����
      // else if(recdat.data[0] == 8)
      // {
      //   if(runout.enabled)
      //   {
      //     RTS_SndData(1, FILAMENT_CONTROL_ICON_VP);
      //     runout.enabled = false;
      //   }
      //   else
      //   {
      //     RTS_SndData(0, FILAMENT_CONTROL_ICON_VP);
      //     runout.enabled = true;
      //   }
      // }
      //���ӵĶϵ�����ʹ�ܰ�ť����
      // else if(recdat.data[0] == 9)
      // {
      //   if(recovery.enabled)
      //   {
      //     RTS_SndData(1, POWERCONTINUE_CONTROL_ICON_VP);
      //     recovery.enabled = false;
      //   }
      //   else
      //   {
      //     RTS_SndData(0, POWERCONTINUE_CONTROL_ICON_VP);
      //     recovery.enabled = true;
      //   }
      // }
      break;

    case PrintSpeedEnterKey:
      feedrate_percentage = recdat.data[0];
      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      break;

    case StopPrintKey:
      if(recdat.data[0] == 1)
      {
        RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
        change_page_font = 13;
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        Update_Time_Value = 0;
		    temphot = 0;

        runout.reset();
        wait_for_user = false;

        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        change_page_font = 40;
        RTS_SDcard_Stop();
      }
      else if(recdat.data[0] == 3)
      {
        if(card.isPrinting())
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      break;

    case PausePrintKey:
      if(recdat.data[0] == 1)
      {
        if(card.isPrinting() && (thermalManager.temp_hotend[0].celsius > (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_bed.celsius > (thermalManager.temp_bed.target - 3)))
        {
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          change_page_font = 11;
        }
        else 
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(card.isPrinting() && (thermalManager.temp_hotend[0].celsius > (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_bed.celsius > (thermalManager.temp_bed.target - 3)))
        {
        }
        else 
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          break;
        }

        waitway = 1;
        // pause_e = current_position[E_AXIS];
        if(!temphot)
        {
          temphot = thermalManager.temp_hotend[0].target;
        }
        // card.pauseSDPrint();
        // print_job_timer.pause();
        queue.inject_P(PSTR("M25"));
        pause_action_flag = true;
        Update_Time_Value = 0;
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        change_page_font = 40;
        planner.synchronize();
        sdcard_pause_check = false;
      }
      else if(recdat.data[0] == 3)
      {
        if(card.isPrinting())
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      break;

    case ResumePrintKey:
      if(recdat.data[0] == 1)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if(1 == READ(FIL_RUNOUT_PIN))
        //   if(runout.filament_ran_out)
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif

        RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
        change_page_font = 10;

        #if ENABLED(HAS_RESUME_CONTINUE)
          if(wait_for_user)
          {
            wait_for_user = false;
          }
          else
        #endif
          {
            planner.synchronize();
            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
            queue.enqueue_one_now(commandbuf);
            // card.startOrResumeFilePrinting();
            // print_job_timer.start();
            queue.inject_P(PSTR("M24"));
            Update_Time_Value = 0;
            sdcard_pause_check = true;
          }
      }
      else if(recdat.data[0] == 2)
      {
        if(thermalManager.temp_hotend[0].target >= EXTRUDE_MINTEMP)
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        }
        else
        {
          thermalManager.setTargetHotend(200, 0);
        }
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if(1 == READ(FIL_RUNOUT_PIN))
        //   if(runout.filament_ran_out)
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
          }
          else
          {
            #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
              SERIAL_ECHOLNPAIR("\r\ncontinu run filament ", pause_menu_response);
            #endif
            RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
            change_page_font = 8;
          }
        #endif
      }
      else if(recdat.data[0] == 3)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if(1 == READ(FIL_RUNOUT_PIN))
        //   if(runout.filament_ran_out)
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif

        // pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
        // ui.pause_show_message(PAUSE_MESSAGE_RESUME);
        // queue.inject_P(PSTR("M108"));
        wait_for_user = false;
        // runout.filament_ran_out = false;
        runout.reset();

        RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
        change_page_font = 10;

        // card.startFileprint();
        print_job_timer.start();
        Update_Time_Value = 0;
        sdcard_pause_check = true;
      }
      // else if(recdat.data[0] == 4)
      // {
      //   if(!CardReader::flag.mounted)
      //   {
      //     CardUpdate = true;
      //     RTS_SDCardUpate();
      //     // card.mount();
      //     RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
      //     change_page_font = 47;
      //   }
      //   else
      //   {
      //     card.startFileprint();
      //     print_job_timer.start();
      //     Update_Time_Value = 0;
      //     sdcard_pause_check = true;
      //     sd_printing_autopause = false;
      //     RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
      //     change_page_font = 10;
      //   }
      // }
      break;

    case ZoffsetEnterKey:
      last_zoffset = zprobe_zoffset;
      if(recdat.data[0] >= 32768)
      {
        zprobe_zoffset = ((float)recdat.data[0] - 65536) / 100;
        zprobe_zoffset -= 0.001;
      }
      else
      {
        zprobe_zoffset = ((float)recdat.data[0]) / 100;
        zprobe_zoffset += 0.001;
      }
      if(WITHIN((zprobe_zoffset), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
      {
        babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
      }
      probe.offset.z = zprobe_zoffset;
      // settings.save();
      break;

    case TempControlKey:
      if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      else if(recdat.data[0] == 3)
      {
        preheat_flag = PREHEAT_PLA;
        temp_preheat_nozzle = ui.material_preset[0].hotend_temp;
        temp_preheat_bed = ui.material_preset[0].bed_temp;

        RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
        change_page_font = 22;
      }
      else if(recdat.data[0] == 4)
      {
        preheat_flag = PREHEAT_ABS;
        temp_preheat_nozzle = ui.material_preset[1].hotend_temp;
        temp_preheat_bed = ui.material_preset[1].bed_temp;

        RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        change_page_font = 23;
      }
      else if(recdat.data[0] == 5)
      {
        thermalManager.temp_hotend[0].target = ui.material_preset[0].hotend_temp;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        thermalManager.temp_bed.target = ui.material_preset[0].bed_temp;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if(recdat.data[0] == 6)
      {
        thermalManager.temp_hotend[0].target = ui.material_preset[1].hotend_temp;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        thermalManager.temp_bed.target = ui.material_preset[1].bed_temp;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 8)
      {
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      break;

    case CoolDownKey:
      if(recdat.data[0] == 1)
      {
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD_SET_TEMP_VP);
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
        thermalManager.fan_speed[0] = 255;
        // RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      break;

    case HeaterTempEnterKey:
      thermalManager.temp_hotend[0].target = recdat.data[0];
      thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
      break;

    case HotBedTempEnterKey:
      thermalManager.temp_bed.target = recdat.data[0];
      thermalManager.setTargetBed(thermalManager.temp_bed.target);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      break;

    case PrepareEnterKey:
      if(recdat.data[0] == 1)
      {
        RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
        change_page_font = 28;
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if(recdat.data[0] == 3)
      {
        rtscheck.RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        rtscheck.RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
      }
      // else if(recdat.data[0] == 4)
      // {
      //   OUT_WRITE(SHUTIDOWN_PIN, LOW);
      //   delay(2000);
      // }
      else if(recdat.data[0] == 5)
      {
        RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
        RTS_SndData(FIRMWARE_VERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
        // // 激光的版本已经在屏幕中写入UI版本号
        // #if ENABLED(LASER_FEATURE)
        //   // 20220223 屏幕版本改由屏幕工程设定
        // #else
        //   RTS_SndData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP); 
        // #endif
        RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
        RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
        delay(5);
        if(1 == lang)
        {
          RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
        }
        else
        {
          RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
        }
        RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
        change_page_font = 24;
      }
      else if(recdat.data[0] == 6)
      {
        queue.enqueue_now_P(PSTR("M84"));
        queue.enqueue_now_P(PSTR("G92.9Z0"));   //rock_20211224 解决人为下降Z轴，导致撞平台的问题。
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 43, ExchangepageAddr);
        change_page_font = 43;
      }
      else if(recdat.data[0] == 8)
      {
        ui.material_preset[preheat_flag].hotend_temp = temp_preheat_nozzle;
        ui.material_preset[preheat_flag].bed_temp = temp_preheat_bed;

        settings.save();
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 9)
      {
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      else if(recdat.data[0] == 0xA)
      {
        RTS_SndData(ExchangePageBase + 42, ExchangepageAddr);
        change_page_font = 42;
      }
      else if(recdat.data[0] == 0xB)
      {
         #if ENABLED(HAS_MENU_RESET_WIFI)
          WIFI_STATE = PRESSED;
          OUT_WRITE(RESET_WIFI_PIN, LOW);
        #endif
        (void)settings.reset(); 
        (void)settings.save();
        RTS_Init(); 
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      else if(recdat.data[0] == 0xC)
      {
        RTS_SndData(ExchangePageBase + 44, ExchangepageAddr);
        change_page_font = 44;
      }
      else if(recdat.data[0] == 0xD)
      {
        settings.reset();
        settings.save();
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if(recdat.data[0] == 0xE)
      {
        if(!planner.has_blocks_queued())
        {
	        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
	        change_page_font = 33;
	      }
      }
      else if(recdat.data[0] == 0xF)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
        settings.save();delay(100);
      }
      else if(recdat.data[0] == 0x10)
      {
        RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
        change_page_font = 25;
      }
      else if(recdat.data[0] == 0x11)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      break;

    case BedLevelKey:
      if(recdat.data[0] == 1)
      {
        planner.synchronize();
        waitway = 6;
        RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
        change_page_font = 26;

        // 此处造成屏幕显示的位置与实际位置不符,注释掉 //20220310 zhangguanghua
        // if((!axis_is_trusted(X_AXIS)) || (!axis_is_trusted(Y_AXIS))) //axis_is_trusted((AxisEnum)axis) if((!TEST(axis_known_position, X_AXIS)) || (!TEST(axis_known_position, Y_AXIS)))
        // {
        //   queue.enqueue_now_P(PSTR("G28"));
        // }
        // else
        // {
        //   queue.enqueue_now_P(PSTR("G28 Z10"));
        // }
        //queue.enqueue_now_P(PSTR("G1 F200 Z10.0"));

        // queue.enqueue_now_P(PSTR("G28"));
        queue.enqueue_now_P(PSTR("G28"));

        // rtscheck.RTS_SndData(0, AXIS_Z_COORD_VP);
        // queue.enqueue_now_P(PSTR("G28 Z10"));

        // RTS_SndData(0, MOTOR_FREE_ICON_VP);
        rtscheck.RTS_SndData(0 , AUTO_LEVELING_PERCENT_DATA_VP);  
        Update_Time_Value = 0;
      }
      else if(recdat.data[0] == 2)
      {
        last_zoffset = zprobe_zoffset;
        if(WITHIN((zprobe_zoffset + 0.05), -5.02, 5.02))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset + 0.05);
            zprobe_zoffset = zprobe_zoffset - 0.0001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if(recdat.data[0] == 3)
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset - 0.05), -5.02, 5.02))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset - 0.05);
            zprobe_zoffset = zprobe_zoffset + 0.0001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued())
        {
	       RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
	       change_page_font = 25;
        }
      }
      else if(recdat.data[0] == 5)
      {
        char cmd[20];

        // Assitant Level ,  Centre 1
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));
          //   queue.enqueue_now_P(PSTR("G1 X110 Y110 F3000"));

          sprintf_P(cmd, "G1 X%d Y%d F3000", manual_level_5position[0][0],manual_level_5position[0][1]);
          queue.enqueue_now_P(cmd);
          // 提前提交轴数据到屏幕
          rtscheck.RTS_SndData(10 * manual_level_5position[0][0], AXIS_X_COORD_VP);
          rtscheck.RTS_SndData(10 * manual_level_5position[0][1], AXIS_Y_COORD_VP);

          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          rtscheck.RTS_SndData(10 * 0, AXIS_Z_COORD_VP);

          waitway = 0;
        }
      }
      else if (recdat.data[0] == 6)
      {
        char cmd[20];
        // Assitant Level , Front Left 2
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));

          // queue.enqueue_now_P(PSTR("G1 X30 Y30 F3000"));
          sprintf_P(cmd, "G1 X%d Y%d F3000", manual_level_5position[1][0],manual_level_5position[1][1]);
          queue.enqueue_now_P(cmd);
          // 提前提交轴数据到屏幕
          rtscheck.RTS_SndData(10 * manual_level_5position[1][0], AXIS_X_COORD_VP);
          rtscheck.RTS_SndData(10 * manual_level_5position[1][1], AXIS_Y_COORD_VP);

          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          rtscheck.RTS_SndData(10 * 0, AXIS_Z_COORD_VP);
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 7)
      {
        char cmd[20];
        // Assitant Level , Front Right 3
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));

          // queue.enqueue_now_P(PSTR("G1 X190 Y30 F3000"));
          sprintf_P(cmd, "G1 X%d Y%d F3000", manual_level_5position[2][0],manual_level_5position[2][1]);
          queue.enqueue_now_P(cmd);
          // 提前提交轴数据到屏幕
          rtscheck.RTS_SndData(10 * manual_level_5position[2][0], AXIS_X_COORD_VP);
          rtscheck.RTS_SndData(10 * manual_level_5position[2][1], AXIS_Y_COORD_VP);

          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          rtscheck.RTS_SndData(10 * 0, AXIS_Z_COORD_VP);
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 8)
      {
        char cmd[20];
        // Assitant Level , Back Right 4
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));
          // queue.enqueue_now_P(PSTR("G1 X190 Y190 F3000"));
          sprintf_P(cmd, "G1 X%d Y%d F3000", manual_level_5position[3][0],manual_level_5position[3][1]);
          queue.enqueue_now_P(cmd);
          // 提前提交轴数据到屏幕
          rtscheck.RTS_SndData(10 * manual_level_5position[3][0], AXIS_X_COORD_VP);
          rtscheck.RTS_SndData(10 * manual_level_5position[3][1], AXIS_Y_COORD_VP);
          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          rtscheck.RTS_SndData(10 * 0, AXIS_Z_COORD_VP);

          waitway = 0;
        }
      }
      else if (recdat.data[0] == 9)
      {
        char cmd[20];
        // Assitant Level , Back Left 5
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));

          // queue.enqueue_now_P(PSTR("G1 X30 Y190 F3000"));
          sprintf_P(cmd, "G1 X%d Y%d F3000", manual_level_5position[4][0],manual_level_5position[4][1]);
          queue.enqueue_now_P(cmd);
          // 提前提交轴数据到屏幕
          rtscheck.RTS_SndData(10 * manual_level_5position[4][0], AXIS_X_COORD_VP);
          rtscheck.RTS_SndData(10 * manual_level_5position[4][1], AXIS_Y_COORD_VP);

          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          rtscheck.RTS_SndData(10 * 0, AXIS_Z_COORD_VP);

          waitway = 0;
        }
      }
      else if(recdat.data[0] == 0x0A)
      {
        if(!planner.has_blocks_queued())
        {
	        RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
	        change_page_font = 26;
		    }
      }
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      break;

    case AutoHomeKey:
      if(recdat.data[0] == 1)
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
        RTS_SndData(3, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
        change_page_font = 17;
        RTS_SndData(2, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_SndData(ExchangePageBase + 18, ExchangepageAddr);
        change_page_font = 18;
        RTS_SndData(1, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 4)
      {
        waitway = 4;
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        change_page_font = 40;

        queue.enqueue_now_P(PSTR("G28 X Y"));
        Update_Time_Value = 0;
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
      }
      else if(recdat.data[0] == 5)
      {
        waitway = 4;
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        change_page_font = 40;

        queue.enqueue_now_P(PSTR("G28"));
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        Update_Time_Value = 0;
      }
      break;

    case XaxismoveKey:
      float x_min, x_max;
      waitway = 4;
      x_min = 0;
      x_max = X_MAX_POS;
      current_position[X_AXIS] = ((float)recdat.data[0]) / 10;
      if(current_position[X_AXIS] < x_min)
      {
        current_position[X_AXIS] = x_min;
      }
      else if(current_position[X_AXIS] > x_max)
      {
        current_position[X_AXIS] = x_max;
      }
      RTS_line_to_current(X_AXIS);
      RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
      delay(1);
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      waitway = 0;
      break;

    case YaxismoveKey:
      float y_min, y_max;
      waitway = 4;
      y_min = 0;
      y_max = Y_MAX_POS;
      current_position[Y_AXIS] = ((float)recdat.data[0]) / 10;
      if(current_position[Y_AXIS] < y_min)
      {
        current_position[Y_AXIS] = y_min;
      }
      else if(current_position[Y_AXIS] > y_max)
      {
        current_position[Y_AXIS] = y_max;
      }
      RTS_line_to_current(Y_AXIS);
      RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
      delay(1);
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      waitway = 0;
      break;

    case ZaxismoveKey:
      float z_min, z_max;
      waitway = 4;
      z_min = Z_MIN_POS;
      z_max = Z_MAX_POS;
      current_position[Z_AXIS] = ((float)recdat.data[0])/10;
      if (current_position[Z_AXIS] < z_min)
      {
        current_position[Z_AXIS] = z_min;
      }
      else if (current_position[Z_AXIS] > z_max)
      {
        current_position[Z_AXIS] = z_max;
      }
      RTS_line_to_current(Z_AXIS);
      RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
      delay(1);
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      waitway = 0;
      break;

    case HeaterLoadEnterKey:
      FilamentLOAD = ((float)recdat.data[0]) / 10;
      RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
      if(!planner.has_blocks_queued())
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if(1 == READ(FIL_RUNOUT_PIN))
        //   if(runout.filament_ran_out)
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            break;
          }
        #endif
        current_position[E_AXIS] += FilamentLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
          // break;
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
          // break;
        }
        
        while(ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
        {
          idle();
        }

        //else
        {
          RTS_line_to_current(E_AXIS);
          RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
          planner.synchronize();
        }
      }
      break;

    case HeaterUnLoadEnterKey:
      FilamentUnLOAD = ((float)recdat.data[0]) / 10;
      RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      if(!planner.has_blocks_queued())
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if(1 == READ(FIL_RUNOUT_PIN))
        //   if(runout.filament_ran_out)
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            break;
          }
        #endif

        current_position[E_AXIS] -= FilamentUnLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
          // break;
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
          // break;
        }
        // else
        while(ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
        {
          idle();
        }

        {
          RTS_line_to_current(E_AXIS);
          RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
          planner.synchronize();
        }
      }
      break;

    case HeaterLoadStartKey:
      if(recdat.data[0] == 1)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if(1 == READ(FIL_RUNOUT_PIN))
            // if(runout.filament_ran_out)
            {
              RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
              change_page_font = 46;
              break;
            }
            else if(rts_start_print)
            {
              RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
              change_page_font = 1;
              break;
            }
          #endif

          if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
          {
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
            break;
          }
          else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
          {
            thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
            RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
            break;
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            planner.synchronize();
          }
          RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
      }
      break;

    case SelectLanguageKey:
      if(recdat.data[0] != 0)
      {
        lang = recdat.data[0];
      }
      language_change_font = lang;
      for(int i = 0;i < 9;i ++)
      {
        RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
      }
      RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
      languagedisplayUpdate();
      // settings.save();
      break;

    case PowerContinuePrintKey:
      if(recdat.data[0] == 1)
      {

      #if ENABLED(POWER_LOSS_RECOVERY)
        if(recovery.info.recovery_flag)
        {
          power_off_type_yes = 1;
          Update_Time_Value = 0;
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          // recovery.resume();
          queue.enqueue_now_P(PSTR("M1000"));

          PoweroffContinue = true;
          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
        }
      #elif ENABLED(CREALITY_POWER_LOSS)
        if(pre01_power_loss.info.recovery_flag)
        {
          power_off_type_yes = 1;
          Update_Time_Value = 0;
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          pre01_power_loss.resume();  //�ָ�SD����ӡ
          // queue.enqueue_now_P(PSTR("M1000"));

          PoweroffContinue = true;
          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
        }
      #endif
      }
      else if(recdat.data[0] == 2)
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        RTS_SndData(0, PRINT_REMAIN_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_REMAIN_TIME_MIN_VP);

        Update_Time_Value = 0;
        RTS_SDcard_Stop();
      }
      break;

    case PLAHeadSetEnterKey:
      // ui.material_preset[0].hotend_temp = recdat.data[0];
      // RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
      //int temp_preheat_nozzle = 0, temp_preheat__bed = 0;
      temp_preheat_nozzle = recdat.data[0];
      RTS_SndData(temp_preheat_nozzle, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
      break;

    case PLABedSetEnterKey:
      // ui.material_preset[0].bed_temp = recdat.data[0];
      // RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
      temp_preheat_bed = recdat.data[0];
      RTS_SndData(temp_preheat_bed, PREHEAT_PLA_SET_BED_TEMP_VP);
      
      break;

    case ABSHeadSetEnterKey:
      // ui.material_preset[1].hotend_temp = recdat.data[0];
      // RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
      temp_preheat_nozzle = recdat.data[0];
      RTS_SndData(temp_preheat_nozzle, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
      break;

    case ABSBedSetEnterKey:
      // ui.material_preset[1].bed_temp = recdat.data[0];
      // RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
      
      temp_preheat_bed = recdat.data[0];
      RTS_SndData(temp_preheat_bed, PREHEAT_ABS_SET_BED_TEMP_VP);
      break;

    case StoreMemoryKey:
      if(recdat.data[0] == 1)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      if(recdat.data[0] == 2)
      {
        queue.enqueue_now_P(PSTR("M502"));
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        RTS_SndData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
        // delay(20);

        RTS_SndData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
        // delay(20);

        RTS_SndData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
        // delay(20);

        RTS_SndData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
        // delay(20);

        RTS_SndData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
        RTS_SndData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
        RTS_SndData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
        delay(20);
        RTS_SndData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
        RTS_SndData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
        RTS_SndData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
        // delay(100);
      }
      else if(recdat.data[0] == 3)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if(recdat.data[0] == 4)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 34, ExchangepageAddr);
        change_page_font = 34;
      }
      else if(recdat.data[0] == 5)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
        change_page_font = 39;
      }
      // else if(recdat.data[0] == 6)
      // {
      //   if(wifi_enable_flag)
      //   {
      //     wifi_enable_flag = 0;
      //     queue.inject_P(PSTR("M115"));
      //     RTS_SndData(1, ADV_SETTING_WIFI_ICON_VP);
      //     settings.save();
      //   }
      //   else
      //   {
      //     wifi_enable_flag = 1;
      //     queue.inject_P(PSTR("M115"));
      //     RTS_SndData(0, ADV_SETTING_WIFI_ICON_VP);
      //     settings.save();
      //   }
      // }
      else if(recdat.data[0] == 7)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 38, ExchangepageAddr);
        change_page_font = 38;
      }
      else if(recdat.data[0] == 8)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);
        change_page_font = 36;
      }
      else if(recdat.data[0] == 9)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      else if(recdat.data[0] == 0x0A)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
        change_page_font = 35;
      }
      else if(recdat.data[0] == 0x0B)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        delay(100);
      }
      else if(recdat.data[0] == 0x0C)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 34, ExchangepageAddr);
        change_page_font = 34;
        settings.save();
        delay(100);
      }
      else if(recdat.data[0] == 0x0D)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        // settings.save();
        // delay(100);
      }
      break;

    case FanSpeedEnterKey:
      thermalManager.fan_speed[0] = recdat.data[0];
      RTS_SndData(thermalManager.fan_speed[0], FAN_SPEED_CONTROL_DATA_VP);
      break;

    case VelocityXaxisEnterKey:
      float velocity_xaxis;
      velocity_xaxis = planner.settings.max_feedrate_mm_s[0];
      velocity_xaxis = recdat.data[0];
      RTS_SndData(velocity_xaxis, MAX_VELOCITY_XAXIS_DATA_VP);
      planner.set_max_feedrate(X_AXIS, velocity_xaxis);
      break;

    case VelocityYaxisEnterKey:
      float velocity_yaxis;
      velocity_yaxis = planner.settings.max_feedrate_mm_s[1];
      velocity_yaxis = recdat.data[0];
      RTS_SndData(velocity_yaxis, MAX_VELOCITY_YAXIS_DATA_VP);
      planner.set_max_feedrate(Y_AXIS, velocity_yaxis);
      break;

    case VelocityZaxisEnterKey:
      float velocity_zaxis;
      velocity_zaxis = planner.settings.max_feedrate_mm_s[2];
      velocity_zaxis = recdat.data[0];
      RTS_SndData(velocity_zaxis, MAX_VELOCITY_ZAXIS_DATA_VP);
      planner.set_max_feedrate(Z_AXIS, velocity_zaxis);
      break;

    case VelocityEaxisEnterKey:
      float velocity_eaxis;
      velocity_eaxis = planner.settings.max_feedrate_mm_s[3];
      velocity_eaxis = recdat.data[0];
      RTS_SndData(velocity_eaxis, MAX_VELOCITY_EAXIS_DATA_VP);
      planner.set_max_feedrate(E_AXIS, velocity_eaxis);
      break;


    case AccelXaxisEnterKey:
      float accel_xaxis;
      accel_xaxis = planner.settings.max_acceleration_mm_per_s2[0];
      accel_xaxis = recdat.data[0];
      RTS_SndData(accel_xaxis, MAX_ACCEL_XAXIS_DATA_VP);
      planner.set_max_acceleration(X_AXIS, accel_xaxis);
      break;

    case AccelYaxisEnterKey:
      float accel_yaxis;
      accel_yaxis = planner.settings.max_acceleration_mm_per_s2[1];
      accel_yaxis = recdat.data[0];
      RTS_SndData(accel_yaxis, MAX_ACCEL_YAXIS_DATA_VP);
      planner.set_max_acceleration(Y_AXIS, accel_yaxis);
      break;

    case AccelZaxisEnterKey:
      float accel_zaxis;
      accel_zaxis = planner.settings.max_acceleration_mm_per_s2[2];
      accel_zaxis = recdat.data[0];
      RTS_SndData(accel_zaxis, MAX_ACCEL_ZAXIS_DATA_VP);
      planner.set_max_acceleration(Z_AXIS, accel_zaxis);
      break;

    case AccelEaxisEnterKey:
      float accel_eaxis;
      accel_eaxis = planner.settings.max_acceleration_mm_per_s2[3];
      accel_eaxis = recdat.data[0];
      RTS_SndData(accel_eaxis, MAX_ACCEL_EAXIS_DATA_VP);
      planner.set_max_acceleration(E_AXIS, accel_eaxis);
      break;

    case JerkXaxisEnterKey:
      float jerk_xaxis;
      jerk_xaxis = planner.max_jerk.x;
      jerk_xaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_xaxis * 100, MAX_JERK_XAXIS_DATA_VP);
      planner.set_max_jerk(X_AXIS, jerk_xaxis);
      break;

    case JerkYaxisEnterKey:
      float jerk_yaxis;
      jerk_yaxis = planner.max_jerk.y;
      jerk_yaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_yaxis * 100, MAX_JERK_YAXIS_DATA_VP);
      planner.set_max_jerk(Y_AXIS, jerk_yaxis);
      break;

    case JerkZaxisEnterKey:
      float jerk_zaxis;
      jerk_zaxis = planner.max_jerk.z;
      jerk_zaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_zaxis * 100, MAX_JERK_ZAXIS_DATA_VP);
      planner.set_max_jerk(Z_AXIS, jerk_zaxis);
      break;

    case JerkEaxisEnterKey:
      float jerk_eaxis;
      jerk_eaxis = planner.max_jerk.e;
      jerk_eaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_eaxis * 100, MAX_JERK_EAXIS_DATA_VP);
      planner.set_max_jerk(E_AXIS, jerk_eaxis);
      break;

    case StepsmmXaxisEnterKey:
      float stepsmm_xaxis;
      stepsmm_xaxis = planner.settings.axis_steps_per_mm[0];
      stepsmm_xaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_xaxis * 10, MAX_STEPSMM_XAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[X_AXIS] = stepsmm_xaxis;
      break;

    case StepsmmYaxisEnterKey:
      float stepsmm_yaxis;
      stepsmm_yaxis = planner.settings.axis_steps_per_mm[1];
      stepsmm_yaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_yaxis * 10, MAX_STEPSMM_YAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Y_AXIS] = stepsmm_yaxis;
      break;

    case StepsmmZaxisEnterKey:
      float stepsmm_zaxis;
      stepsmm_zaxis = planner.settings.axis_steps_per_mm[2];
      stepsmm_zaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_zaxis * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Z_AXIS] = stepsmm_zaxis;
      break;

    case StepsmmEaxisEnterKey:
      float stepsmm_eaxis;
      stepsmm_eaxis = planner.settings.axis_steps_per_mm[3];
      stepsmm_eaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_eaxis * 10, MAX_STEPSMM_EAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[E_AXIS] = stepsmm_eaxis;
      break;

    case NozzlePTempEnterKey:
      float nozzle_ptemp;
      nozzle_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
      PID_PARAM(Kp, 0) = nozzle_ptemp;
      break;

    case NozzleITempEnterKey:
      float nozzle_itemp;
      nozzle_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
      PID_PARAM(Ki, 0) = scalePID_i(nozzle_itemp);
      break;

    case NozzleDTempEnterKey:
      float nozzle_dtemp;
      nozzle_dtemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
      PID_PARAM(Kd, 0) = scalePID_d(nozzle_dtemp);
      break;

    case HotbedPTempEnterKey:
      float hotbed_ptemp;
      hotbed_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
      thermalManager.temp_bed.pid.Kp = hotbed_ptemp;
      break;

    case HotbedITempEnterKey:
      float hotbed_itemp;
      hotbed_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
      thermalManager.temp_bed.pid.Ki = scalePID_i(hotbed_itemp);
      break;

    case HotbedDTempEnterKey:
      float hotbed_dtemp;
      hotbed_dtemp = (float)recdat.data[0] / 10;
      RTS_SndData(hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
      thermalManager.temp_bed.pid.Kd = scalePID_d(hotbed_dtemp);
      break;

    case PrintFanSpeedkey:
      uint8_t fan_speed;
      fan_speed = (uint8_t)recdat.data[0];
      RTS_SndData(fan_speed , PRINTER_FAN_SPEED_DATA_VP);
      thermalManager.set_fan_speed(0, fan_speed);
      break;

    case SelectFileKey:
      if (RTS_SD_Detected())
      {
        if (recdat.data[0] > CardRecbuf.Filesum)
        {
          break;
        }

        CardRecbuf.recordcount = recdat.data[0] - 1;

        for(int j = 0; j < 10; j ++)
        {
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        delay(2);
        for(int j = 1;j <= CardRecbuf.Filesum;j ++)
        {
          RTS_SndData((unsigned long)0x073F, FilenameNature + j * 16);
          RTS_SndData(0, FILE1_SELECT_ICON_VP - 1 + j);
        }
        RTS_SndData((unsigned long)0xFFFF, FilenameNature + recdat.data[0] * 16);
        RTS_SndData(1, FILE1_SELECT_ICON_VP + (recdat.data[0] - 1));
      }

      RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      change_page_font = 1;
      rts_start_print = true;
      delay(20);
      RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);
      break;

    case StartFileKey:
      if((recdat.data[0] == 1) && RTS_SD_Detected())
      {
        if(CardRecbuf.recordcount < 0)
        {
          break;
        }
        if(!rts_start_print)
        {
          //SERIAL_ECHOLNPAIR("\r\nrts_start_print: ", rts_start_print);
          break;
        }

        char cmd[30];
        char *c;
        sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
        for (c = &cmd[4]; *c; c++)
        {
          *c = tolower(*c);
        }

        memset(cmdbuf, 0, sizeof(cmdbuf));
        strcpy(cmdbuf, cmd);
        FilenamesCount = CardRecbuf.recordcount;
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if(1 == READ(FIL_RUNOUT_PIN))
        //   if(runout.filament_ran_out)
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            sdcard_pause_check = false;
            break;
          }
        #endif

        rts_start_print = false;

        queue.enqueue_one_now(cmd);
        delay(20);
        queue.enqueue_now_P(PSTR("M24"));
        // clean screen.
        for (int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        }

        RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

        delay(2);

        #if ENABLED(BABYSTEPPING)
          RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
        #endif
        feedrate_percentage = 100;
        RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
        // zprobe_zoffset = last_zoffset;
        zprobe_zoffset = probe.offset.z;
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        PoweroffContinue = true;
        RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
        Update_Time_Value = 0;
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 3, ExchangepageAddr);
        change_page_font = 3;
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
        change_page_font = 2;
      }
      else if(recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 4, ExchangepageAddr);
        change_page_font = 4;
      }
      else if(recdat.data[0] == 5)
      {
        RTS_SndData(ExchangePageBase + 3, ExchangepageAddr);
        change_page_font = 3;
      }
      else if(recdat.data[0] == 6)
      {
        RTS_SndData(ExchangePageBase + 5, ExchangepageAddr);
        change_page_font = 5;
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 4, ExchangepageAddr);
        change_page_font = 4;
      }
      else if(recdat.data[0] == 8)
      {
        RTS_SndData(ExchangePageBase + 5, ExchangepageAddr);   //by jankin 20210731
        change_page_font = 5;
      }
      else if(recdat.data[0] == 9)
      {
        RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
        change_page_font = 2;
      }
      else if(recdat.data[0] == 0x0A)
      {
        RTS_SndData(ExchangePageBase + 5, ExchangepageAddr);
        change_page_font = 5;
      }
      break;

    case ChangePageKey:
      for(int i = 0; i < MaxFileNumber; i ++)
      {
        for (int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, FILE1_TEXT_VP + i * 20 + j);
        }
      }

      for (int i = 0; i < CardRecbuf.Filesum; i++)
      {
        for (int j = 0; j < 20; j++)
        {
          RTS_SndData(0, CardRecbuf.addr[i] + j);
        }
        RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
      }

      for (int j = 0; j < 20; j ++)
      {
        // clean screen.
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        // clean filename
        RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
      }
      // clean filename Icon
      for (int j = 0; j < 20; j ++)
      {
        RTS_SndData(0, FILE1_SELECT_ICON_VP + j);
      }

      RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

      // represents to update file list
      // if (CardUpdate && lcd_sd_status && IS_SD_INSERTED())
      if (CardUpdate && lcd_sd_status && RTS_SD_Detected())
      {
        for (uint16_t i = 0; i < CardRecbuf.Filesum; i++)
        {
          delay(3);
          RTS_SndData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
          RTS_SndData(0, FILE1_SELECT_ICON_VP + i);
        }
      }

      RTS_SndData(MACVERSION, MACHINE_TYPE_ABOUT_TEXT_VP);
      RTS_SndData(SOFTVERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
      RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);

      if(1 == lang)
      {
        RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
      }
      else
      {
        RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
      }

      // if(thermalManager.fan_speed[0] == 0)
      // {
      //   RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
      // }
      // else
      // {
      //   RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
      // }

      // if(LEDStatus)
      // {
      //   RTS_SndData(1, PRINTER_LEDOPEN_TITLE_VP);
      // }
      // else
      // {
      //   RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
      // }
      // Percentrecord = card.percentDone() + 1;
      // if (Percentrecord <= 100)
      // {
      //   rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);

      // }
      rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);

      RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      languagedisplayUpdate();

      RTS_SndData(change_page_font + ExchangePageBase, ExchangepageAddr);
      break;

    #if HAS_CUTTER
      case SwitchDeviceKey:
        if(recdat.data[0] == 1)// FDM
        {
          RTS_SndData(ExchangePageBase + 57, ExchangepageAddr);
          //change_page_font = 57;
        }else if(recdat.data[0] == 2) // Laser
        {
          RTS_SndData(ExchangePageBase + 56, ExchangepageAddr);
          //change_page_font = 56;
        }else if(recdat.data[0] == 0x03)// 切换到FDM 确认
        {
          if(change_page_font == 64)
          {
            RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
            change_page_font = 33;
          }else{
            RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            change_page_font = 1;
          }
          laser_device.set_current_device(DEVICE_FDM);
        }else if(recdat.data[0] == 0x04)// 切换到FDM 取消
        {
          if(change_page_font == 64)
          {
            RTS_SndData(ExchangePageBase + 64, ExchangepageAddr);
            change_page_font = 64;
          }else{
            RTS_SndData(ExchangePageBase + 50, ExchangepageAddr);
          }
        }else if(recdat.data[0] == 0x05)// 切换到激光 确定
        {
          uint8_t language; // 此处不改变语言
          RTS_SndData(ExchangePageBase + 77, ExchangepageAddr);
          //change_page_font = 77;
          laser_device.set_current_device(DEVICE_LASER);
          language = language_change_font; 
          settings.reset();
          language_change_font = language;
          settings.save();
          probe.offset.z = zprobe_zoffset = 0;
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

          // queue.inject_P(PSTR("M999"));
          queue.enqueue_now_P(PSTR("M999\nG92.9 Z0"));

          planner.synchronize();
           // 同步当前位置到 显示控件
          RTS_SndData(0, SW_FOCUS_Z_VP);
          laser_device.laser_power_open();
        }else if(recdat.data[0] == 0x06)// 切换到激光 取消
        {
          if(change_page_font == 33){
            RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
            change_page_font = 33;
          }else{
            RTS_SndData(ExchangePageBase + 50, ExchangepageAddr);
            change_page_font = 50;
          }
        }
        // else if(recdat.data[0] == 8) //调整焦距 √
        // {
        //   queue.inject_P(PSTR("G92.9 Z0"));
        //   RTS_SndData(0, AXIS_Z_COORD_VP);
        //   RTS_SndData(0, SW_FOCUS_Z_VP);
        //   SERIAL_ECHOPAIR("\nchange_page_font=",change_page_font);
        //   if(change_page_font == 64){
        //     RTS_SndData(ExchangePageBase + 64, ExchangepageAddr);
        //     change_page_font = 64;
        //   }else{
        //     RTS_SndData(ExchangePageBase + 51, ExchangepageAddr);
        //     change_page_font = 51;
        //   }
        // }
        else if(recdat.data[0] == 0x0B)// 切换到激光
        {
          RTS_SndData(ExchangePageBase + 56, ExchangepageAddr);
          //change_page_font = 56;
        }
      break;
    #endif
    case ErrorKey:
      {
        if(recdat.data[0] == 1)
        {
          if(printingIsActive())
          {
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
          }
          else if(printingIsPaused())
          {
            SERIAL_ECHOPAIR("\ncase ErrorKey:", recdat.data[0]);
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
            change_page_font = 12;
          }
          else
          {
            RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            change_page_font = 1;
          }

          if(errorway == 4)
          {
            // reboot
            HAL_reboot();
          }
        }
      }
      break;

    default:
      break;
  }
  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

void EachMomentUpdate(void)
{
  millis_t ms = millis();
  if(ms > next_rts_update_ms)
  {
  #if ENABLED(POWER_LOSS_RECOVERY)
    // print the file before the power is off.
    if((power_off_type_yes == 0) && lcd_sd_status && (recovery.info.recovery_flag == true))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      // delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        for(uint16_t i = 0;i < CardRecbuf.Filesum;i ++) 
        {
          if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1]))
          {
            rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
            rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], SELECT_FILE_TEXT_VP);
            rtscheck.RTS_SndData(ExchangePageBase + 27, ExchangepageAddr);
            change_page_font = 27;
            break;
          }
        }
      }
      return;
    }
  #elif ENABLED(CREALITY_POWER_LOSS)
    // print the file before the power is off.
    if((power_off_type_yes == 0) && lcd_sd_status && (pre01_power_loss.info.recovery_flag == true))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      // delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        for(uint16_t i = 0;i < CardRecbuf.Filesum;i ++) 
        {
          if(!strcmp(CardRecbuf.Cardfilename[i], &pre01_power_loss.info.sd_filename[1]))
          {
            rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
            rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], SELECT_FILE_TEXT_VP);
            rtscheck.RTS_SndData(ExchangePageBase + 27, ExchangepageAddr);
            change_page_font = 27;
            break;
          }
        }
      }
      return;
    }
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    else if((power_off_type_yes == 0) && (recovery.info.recovery_flag == false))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      // delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        Update_Time_Value = RTS_UPDATE_VALUE; 
        
        #if HAS_CUTTER
          if(laser_device.is_laser_device()){
            rtscheck.RTS_SndData(ExchangePageBase + 51, ExchangepageAddr);
            change_page_font = 51;
          }else if(laser_device.get_current_device()== DEVICE_UNKNOWN){
            rtscheck.RTS_SndData(ExchangePageBase + 50, ExchangepageAddr);
            change_page_font = 50;
          }else
        #endif
        {
          rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
        }
      }
      
      return;
    }
    else
    {
      // need to optimize
      // if(recovery.info.print_job_elapsed != 0)
      // {
      //   duration_t elapsed = print_job_timer.duration();
      //   static unsigned char last_cardpercentValue = 100;
      //   rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
      //   rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
        static unsigned char last_cardpercentValue = 100;
        if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
        {
          // duration_t elapsed = print_job_timer.duration();
          // rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
          // rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
          if((unsigned char) card.percentDone() > 0)
          {
            Percentrecord = card.percentDone();
            if(Percentrecord <= 100)
            {
              rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
            }
          }
          else
          {
            rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
          }
          rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
          last_cardpercentValue = card.percentDone();
          rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        }
        else if(card.isPrinting())
        {
          duration_t elapsed = print_job_timer.duration();
          Percentrecord = card.percentDone();

          rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
          rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);

          //剩余时间计算
          if(Percentrecord<2) //使用gcode中 读取到的时间 - 已打印时间
          {
            //文件中- elapsed
            rtscheck.RTS_SndData(0, PRINT_REMAIN_TIME_HOUR_VP);
            rtscheck.RTS_SndData(0, PRINT_REMAIN_TIME_MIN_VP);
          }else{
              int _remain_time = 0;
              _remain_time = ((elapsed.value) * ((float)card.getFileSize() / (float)card.getIndex())) - (elapsed.value);
              
              if(_remain_time < 0) _remain_time = 0;
              rtscheck.RTS_SndData(_remain_time / 3600, PRINT_REMAIN_TIME_HOUR_VP);
              rtscheck.RTS_SndData((_remain_time % 3600) / 60, PRINT_REMAIN_TIME_MIN_VP);
          }
        }
      // }

      if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        queue.enqueue_now_P(PSTR("G0 F3000 X0 Y0"));
        thermalManager.setTargetHotend(0, 0);
        rtscheck.RTS_SndData(0, HEAD_SET_TEMP_VP);
      }

      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

      #if ENABLED(SDSUPPORT)
        // if(PoweroffContinue == true)
        // {
        //   if(true == sdcard_pause_check)
        //   {
        //     if(!CardReader::flag.mounted)
        //     {
        //       card.pauseSDPrint();
        //       print_job_timer.pause();
        //       Update_Time_Value = 0;
        //       rtscheck.RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
        //       change_page_font = 47;
        //       sdcard_pause_check = false;
        //     }
        //   }
        // }

        if((false == sdcard_pause_check) && (false == card.isPrinting()) && !planner.has_blocks_queued())
        {
          if(CardReader::flag.mounted)
          {
            rtscheck.RTS_SndData(1, CHANGE_SDCARD_ICON_VP);
          }
          else
          {
            rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
          }
        }
      #endif

      if((last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
      {
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
      }

      if((thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target) && (heatway == 1))
      {
        rtscheck.RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
        heatway = 0;
        rtscheck.RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        rtscheck.RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }
      #if ENABLED(FILAMENT_RUNOUT_SENSOR)
        if(1 == READ(FIL_RUNOUT_PIN))
        // if(runout.filament_ran_out)
        {
          rtscheck.RTS_SndData(0, FILAMENT_LOAD_ICON_VP);
        }
        else
        {
          rtscheck.RTS_SndData(1, FILAMENT_LOAD_ICON_VP);
        }
      #endif
      rtscheck.RTS_SndData(thermalManager.fan_speed[0] , PRINTER_FAN_SPEED_DATA_VP);
    }
  #elif ENABLED(CREALITY_POWER_LOSS)
    else if((power_off_type_yes == 0) && (pre01_power_loss.info.recovery_flag == false))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      // delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        Update_Time_Value = RTS_UPDATE_VALUE;
        
        if(laser_device.is_laser_device()){
          rtscheck.RTS_SndData(ExchangePageBase + 51, ExchangepageAddr);
          change_page_font = 51;
        }else if(laser_device.get_current_device()== DEVICE_UNKNOWN){
          rtscheck.RTS_SndData(ExchangePageBase + 50, ExchangepageAddr);
          change_page_font = 0;
        }else{
          rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
        }
        // rtscheck.RTS_SndData(ExchangePageBase + 51, ExchangepageAddr);
        // change_page_font = 51;
      }
      return;
    }
    else
    {
      // need to optimize
      // if(pre01_power_loss.info.print_job_elapsed != 0)
      // {
      //   duration_t elapsed = print_job_timer.duration();
      //   static unsigned char last_cardpercentValue = 100;
      //   rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
      //   rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
        static unsigned char last_cardpercentValue = 100;
        if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
        {
          duration_t elapsed = print_job_timer.duration();
          
          rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
          rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
          if((unsigned char) card.percentDone() > 0)
          {
            Percentrecord = card.percentDone();
            if(Percentrecord <= 100)
            {
              rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
            }
          }
          else
          {
            rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
          }
          rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
          last_cardpercentValue = card.percentDone();
          rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        }
      // }

      if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        queue.enqueue_now_P(PSTR("G0 F3000 X0 Y0"));
        thermalManager.setTargetHotend(0, 0);
        rtscheck.RTS_SndData(0, HEAD_SET_TEMP_VP);
      }

      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

      #if ENABLED(SDSUPPORT)
        // if(PoweroffContinue == true)
        // {
        //   if(true == sdcard_pause_check)
        //   {
        //     if(!CardReader::flag.mounted)
        //     {
        //       card.pauseSDPrint();
        //       print_job_timer.pause();
        //       Update_Time_Value = 0;
        //       rtscheck.RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
        //       change_page_font = 47;
        //       sdcard_pause_check = false;
        //     }
        //   }
        // }

        if((false == sdcard_pause_check) && (false == card.isPrinting()) && !planner.has_blocks_queued())
        {
          if(CardReader::flag.mounted)
          {
            rtscheck.RTS_SndData(1, CHANGE_SDCARD_ICON_VP);
          }
          else
          {
            rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
          }
        }
      #endif

      if((last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
      {
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
      }

      if((thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target) && (heatway == 1))
      {
        rtscheck.RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
        heatway = 0;
        rtscheck.RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        rtscheck.RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }
      #if ENABLED(FILAMENT_RUNOUT_SENSOR)
        if(1 == READ(FIL_RUNOUT_PIN))
        // if(runout.filament_ran_out)
        {
          rtscheck.RTS_SndData(0, FILAMENT_LOAD_ICON_VP);
        }
        else
        {
          rtscheck.RTS_SndData(1, FILAMENT_LOAD_ICON_VP);
        }
      #endif
      rtscheck.RTS_SndData(thermalManager.fan_speed[0] , PRINTER_FAN_SPEED_DATA_VP);
    }
  #endif

    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
}

void RTSSHOW::languagedisplayUpdate(void)
{
  RTS_SndData(lang, MAIN_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, SELECT_FILE_BLUE_TITLE_VP);
  RTS_SndData(lang, PREPARE_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, SETTING_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, MAIN_PAGE_BLACK_TITLE_VP);
  RTS_SndData(lang, SELECT_FILE_BLACK_TITLE_VP);
  RTS_SndData(lang, PREPARE_PAGE_BLACK_TITLE_VP);
  RTS_SndData(lang, SETTING_PAGE_BLACK_TITLE_VP);

  RTS_SndData(lang, PRINT_ADJUST_MENT_TITLE_VP);
  RTS_SndData(lang, PRINT_SPEED_TITLE_VP);
  RTS_SndData(lang, HEAD_SET_TITLE_VP);
  RTS_SndData(lang, BED_SET_TITLE_VP);
  RTS_SndData(lang, LEVEL_ZOFFSET_TITLE_VP);
  RTS_SndData(lang, FAN_CONTROL_TITLE_VP);
  RTS_SndData(lang, LED_CONTROL_TITLE_VP);

  RTS_SndData(lang, MOVE_AXIS_ENTER_GREY_TITLE_VP);
  RTS_SndData(lang, CHANGE_FILAMENT_GREY_TITLE_VP);
  RTS_SndData(lang, PREHAET_PAGE_GREY_TITLE_VP);
  RTS_SndData(lang, MOVE_AXIS_ENTER_BLACK_TITLE_VP);
  RTS_SndData(lang, CHANGE_FILAMENT_BLACK_TITLE_VP);
  RTS_SndData(lang, PREHAET_PAGE_BLACK_TITLE_VP);

  RTS_SndData(lang, PREHEAT_PLA_BUTTON_TITLE_VP);
  RTS_SndData(lang, PREHEAT_ABS_BUTTON_TITLE_VP);
  RTS_SndData(lang, COOL_DOWN_BUTTON_TITLE_VP);

  RTS_SndData(lang, FILAMENT_LOAD_BUTTON_TITLE_VP);
  RTS_SndData(lang, FILAMENT_UNLOAD_BUTTON_TITLE_VP);

  RTS_SndData(lang, LANGUAGE_SELECT_ENTER_VP);
  RTS_SndData(lang, FACTORY_DEFAULT_ENTER_TITLE_VP);
  RTS_SndData(lang, LEVELING_PAGE_TITLE_VP);

  RTS_SndData(lang, PRINTER_DEVICE_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_ADVINFO_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_INFO_ENTER_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_DEVICE_BLACK_TITLE_VP);
  RTS_SndData(lang, PRINTER_ADVINFO_BLACK_TITLE_VP);
  RTS_SndData(lang, PRINTER_INFO_ENTER_BLACK_TITLE_VP);

  RTS_SndData(lang, PREHEAT_PLA_SET_TITLE_VP);
  RTS_SndData(lang, PREHEAT_ABS_SET_TITLE_VP);

  RTS_SndData(lang, STORE_MEMORY_CONFIRM_TITLE_VP);
  RTS_SndData(lang, STORE_MEMORY_CANCEL_TITLE_VP);

  RTS_SndData(lang, FILAMENT_UNLOAD_IGNORE_TITLE_VP);
  RTS_SndData(lang, FILAMENT_USEUP_TITLE_VP);
  RTS_SndData(lang, BUTTON_CHECK_CONFIRM_TITLE_VP);
  RTS_SndData(lang, BUTTON_CHECK_CANCEL_TITLE_VP);
  RTS_SndData(lang, FILAMENT_LOAD_TITLE_VP);
  RTS_SndData(lang, FILAMENT_LOAD_RESUME_TITLE_VP);
  RTS_SndData(lang, PAUSE_PRINT_POP_TITLE_VP);
  RTS_SndData(lang, STOP_PRINT_POP_TITLE_VP);
  RTS_SndData(lang, POWERCONTINUE_POP_TITLE_VP);
  RTS_SndData(lang, AUTO_HOME_WAITING_POP_TITLE_VP);

  RTS_SndData(lang, BEDLEVELING_WAIT_TITLE_VP);
  RTS_SndData(lang, RESTORE_FACTORY_TITLE_VP);
  RTS_SndData(lang, RESET_WIFI_SETTING_TITLE_VP);
  RTS_SndData(lang, KILL_THERMAL_RUNAWAY_TITLE_VP);
  RTS_SndData(lang, KILL_HEATING_FAIL_TITLE_VP);
  RTS_SndData(lang, KILL_THERMISTOR_ERROR_TITLE_VP);
  RTS_SndData(lang, WIND_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndData(lang, RESET_WIFI_SETTING_BUTTON_VP);
  RTS_SndData(lang, PRINTER_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndData(lang, WIND_AUTO_SHUTDOWN_PAGE_VP);
  RTS_SndData(lang, AUTO_LEVELING_START_TITLE_VP);
  RTS_SndData(lang, AUX_LEVELING_GREY_TITLE_VP);
  RTS_SndData(lang, AUTO_LEVELING_GREY_TITLE_VP);
  RTS_SndData(lang, AUX_LEVELING_BLACK_TITLE_VP);
  RTS_SndData(lang, AUTO_LEVELING_BLACK_TITLE_VP);
  RTS_SndData(lang, LANGUAGE_SELECT_PAGE_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_MOTION_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_PID_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_WIFI_TITLE_VP);

  RTS_SndData(lang, MOTION_SETTING_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_STEPSMM_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_ACCEL_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_JERK_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_VELOCITY_TITLE_VP);

  RTS_SndData(lang, MAX_VELOCITY_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_ACCEL_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_JERK_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_STEPSMM_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_EAXIS_TITLE_VP);

  RTS_SndData(lang, TEMP_PID_SETTING_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_P_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_I_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_D_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_P_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_I_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_D_TITLE_VP);

  RTS_SndData(lang, FILAMENT_CONTROL_TITLE_VP);
  RTS_SndData(lang, POWERCONTINUE_CONTROL_TITLE_VP);
  RTS_SndData(lang, MACHINE_TYPE_ABOUT_CHAR_VP);
  RTS_SndData(lang, FIREWARE_VERSION_ABOUT_CHAR_VP);
  RTS_SndData(lang, PRINTER_DISPLAY_VERSION_TITLE_VP);
  RTS_SndData(lang, HARDWARE_VERSION_ABOUT_TITLE_VP);
  RTS_SndData(lang, WIFI_DN_CODE_CHAR_VP);
  RTS_SndData(lang, WEBSITE_ABOUT_CHAR_VP);
  RTS_SndData(lang, PRINTER_PRINTSIZE_TITLE_VP);

  RTS_SndData(lang, PRINT_FINISH_ICON_VP);

  #if HAS_CUTTER // 新增激光 20220208

    RTS_SndData(lang, SELECT_LASER_WARNING_TIPS_VP);
    RTS_SndData(lang, SELECT_FDM_WARNING_TIPS_VP);
    RTS_SndData(lang, PRINT_MOVE_AXIS_VP);
    RTS_SndData(lang, PRINT_DIRECT_ENGRAV_VP);
    RTS_SndData(lang, PRINT_RUN_RANGE_VP);
    RTS_SndData(lang, PRINT_RETURN_VP);
    RTS_SndData(lang, PRINT_WARNING_TIPS_VP);
    RTS_SndData(lang, DEVICE_SWITCH_LASER_VP);
    RTS_SndData(lang, FIRST_SELECT_DEVICE_TYPE);
    RTS_SndData(lang, HOME_LASER_ENGRAVE_VP);
    RTS_SndData(lang, PREPARE_ADJUST_FOCUS_VP);
    RTS_SndData(lang, PREPARE_SWITCH_FDM_VP);
    RTS_SndData(lang, FIRST_DEVICE_FDM);
    RTS_SndData(lang, FIRST_DEVICE_LASER);
    RTS_SndData(lang, FOCUS_SET_FOCUS_TIPS);
    
  #endif

}

// looping at the loop function
void RTSUpdate(void)
{
  // Check the status of card
  rtscheck.RTS_SDCardUpate();

  EachMomentUpdate();
  // wait to receive massage and response
  if (rtscheck.RTS_RecData() > 0)
  {
    rtscheck.RTS_HandleData();
  }
}

void RTS_PauseMoveAxisPage(void)
{
  #if HAS_CUTTER
    if(laser_device.is_laser_device()) return;
  #endif

  if(waitway == 1)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
    change_page_font = 12;
    waitway = 0;
  }
  else if(waitway == 5)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
    change_page_font = 7;
    waitway = 0;
  }
}

void RTS_AutoBedLevelPage(void)
{
  if(waitway == 3)
  {
    rtscheck.RTS_SndData(0, AXIS_Z_COORD_VP);// G29 最后的位置为Z0
    rtscheck.RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
    change_page_font = 26;
    waitway = 0;
  }
}

void RTS_MoveAxisHoming(void)
{
  if(waitway == 4)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 16 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 16;
    waitway = 0;
  }
  else if(waitway == 6)
  {
    // rtscheck.RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
    // change_page_font = 25;
    waitway = 0;
  }
  else if(waitway == 7)
  {
    // Click Print finish
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
    change_page_font = 1;
    waitway = 0;
  }else if(waitway == 8)// 雕刻警告界面的轴移动
  {
    rtscheck.RTS_SndData(ExchangePageBase + 78 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 78 + (AxisUnitMode - 1);
    waitway = 0;
  }else if(waitway == 9)// 激光的轴移动
  {
    rtscheck.RTS_SndData(ExchangePageBase + 70 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 70 + (AxisUnitMode - 1);
    waitway = 0;
  }else if(waitway == 10){
    rtscheck.RTS_SndData(ExchangePageBase + 51 , ExchangepageAddr);
    change_page_font = 51;
    waitway = 0;
  }

  #if HAS_CUTTER
    if(laser_device.is_laser_device())
    {
      
    }else
  #endif
  {
    rtscheck.RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
    rtscheck.RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
  }
  rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
}

void RTS_CommandPause(void)
{
  if(printingIsActive())
  {
    rtscheck.RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
    change_page_font = 10;
    // card.pauseSDPrint();
    // print_job_timer.pause();
    // pause_action_flag = true; 
  }
}

void ErrorHanding(void)
{
  // No more operations
  if(errorway == 1)
  {
    errorway = errornum = 0;
  }
  else if(errorway == 2)
  {
    // Z axis home failed
    home_errornum ++;
    if(home_errornum <= 3)
    {
      errorway = 0;
      waitway  = 4;
      queue.enqueue_now_P(PSTR("G28"));
      rtscheck.RTS_SndData(0, MOTOR_FREE_ICON_VP);
      Update_Time_Value = 0;
    }
    else
    {
      // After three failed returns to home, it will report the failure interface
      home_errornum = 0;
      errorway = 0;
      rtscheck.RTS_SndData(ExchangePageBase + 41, ExchangepageAddr);
      change_page_font = 41;
      // Z axis home failed
      rtscheck.RTS_SndData(Error_202, ABNORMAL_PAGE_TEXT_VP);
      
      if(printingIsActive())
      {
        rtscheck.RTS_SndData(0, PRINT_TIME_HOUR_VP);
        rtscheck.RTS_SndData(0, PRINT_TIME_MIN_VP);
        rtscheck.RTS_SndData(0, PRINT_REMAIN_TIME_HOUR_VP);
        rtscheck.RTS_SndData(0, PRINT_REMAIN_TIME_MIN_VP);

        Update_Time_Value = 0;

        rtscheck.RTS_SDcard_Stop();
      }
    }
  }
  else if(errorway == 3)
  {
    // No more operations
    reset_bed_level();
    errorway = 0;
    errornum = 0;
  }
  else if(errorway == 4)
  {

  }
}

#endif
