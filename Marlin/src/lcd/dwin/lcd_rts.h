#ifndef LCD_RTS_H
#define LCD_RTS_H

#include "string.h"
#include <arduino.h>

#include "../../inc/MarlinConfig.h"

// extern int power_off_type_yes;

/*********************************/
#define FHONE   (0x5A)
#define FHTWO   (0xA5)
#define FHLENG  (0x06)
#define TEXTBYTELEN     20
#define MaxFileNumber   20

#define AUTO_BED_LEVEL_PREHEAT  0

#define FileNum             MaxFileNumber
#define FileNameLen         TEXTBYTELEN
#define RTS_UPDATE_INTERVAL 500
#define RTS_UPDATE_VALUE    RTS_UPDATE_INTERVAL

#define SizeofDatabuf       26

/*************Register and Variable addr*****************/
#define RegAddr_W   0x80
#define RegAddr_R   0x81
#define VarAddr_W   0x82
#define VarAddr_R   0x83
#define ExchangePageBase    ((unsigned long)0x5A010000)
#define StartSoundSet       ((unsigned long)0x060480A0)

/*Error value*/
#define Error_201   "201 (Command Timeout)"   // The command too much inactive time
#define Error_202   "202 (Homing Failed)"     // Homing Failed
#define Error_203   "203 (Probing Failed)"    // Probing Failed
#define Error_204   "204 (Click Reboot)"      // SD Read Error

/*variable addr*/
#define ExchangepageAddr      0x0084
#define SoundAddr             0x00A0

#define START_PROCESS_ICON_VP              0x1000
#define PRINT_SPEED_RATE_VP                0x1006
#define PRINT_PROCESS_ICON_VP              0x100E
#define PRINT_TIME_HOUR_VP                 0x1010
#define PRINT_TIME_MIN_VP                  0x1012
#define PRINT_PROCESS_VP                   0x1016
// #define PRINTER_FANOPEN_TITLE_VP           0x101E
#define PRINTER_LEDOPEN_TITLE_VP           0x101F
#define PRINTER_AUTO_SHUTDOWN_ICON_VP      0x1020
#define ADV_SETTING_WIFI_ICON_VP           0x1021
#define AUTO_BED_LEVEL_ZOFFSET_VP          0x1026

#define HEAD_SET_TEMP_VP                   0x1034
#define HEAD_CURRENT_TEMP_VP               0x1036
#define BED_SET_TEMP_VP                    0x103A
#define BED_CURRENT_TEMP_VP                0x103C
#define AXIS_X_COORD_VP                    0x1048
#define AXIS_Y_COORD_VP                    0x104A
#define AXIS_Z_COORD_VP                    0x104C
#define HEAD_FILAMENT_LOAD_DATA_VP         0x1052
#define HEAD_FILAMENT_UNLOAD_DATA_VP       0x1054
#define AUTO_BED_PREHEAT_HEAD_DATA_VP      0x108A
#define AUTO_BED_LEVEL_TITLE_VP            0x108D
#define FILAMENT_LOAD_ICON_VP              0x108E
#define PREHEAT_PLA_SET_NOZZLE_TEMP_VP     0x1090
#define PREHEAT_PLA_SET_BED_TEMP_VP        0x1092
#define PREHEAT_ABS_SET_NOZZLE_TEMP_VP     0x1094
#define PREHEAT_ABS_SET_BED_TEMP_VP        0x1096
#define FAN_SPEED_CONTROL_DATA_VP          0x109A
#define AUTO_LEVELING_PERCENT_DATA_VP      0x109C

#define MAX_VELOCITY_XAXIS_DATA_VP         0x109E
#define MAX_VELOCITY_YAXIS_DATA_VP         0x10A0
#define MAX_VELOCITY_ZAXIS_DATA_VP         0x10A2
#define MAX_VELOCITY_EAXIS_DATA_VP         0x10A4

#define MAX_ACCEL_XAXIS_DATA_VP            0x10A6
#define MAX_ACCEL_YAXIS_DATA_VP            0x10A8
#define MAX_ACCEL_ZAXIS_DATA_VP            0x10AA
#define MAX_ACCEL_EAXIS_DATA_VP            0x10AC

#define MAX_JERK_XAXIS_DATA_VP             0x10AE
#define MAX_JERK_YAXIS_DATA_VP             0x10B0
#define MAX_JERK_ZAXIS_DATA_VP             0x10B2
#define MAX_JERK_EAXIS_DATA_VP             0x10B4

#define MAX_STEPSMM_XAXIS_DATA_VP          0x10B6
#define MAX_STEPSMM_YAXIS_DATA_VP          0x10B8
#define MAX_STEPSMM_ZAXIS_DATA_VP          0x10BA
#define MAX_STEPSMM_EAXIS_DATA_VP          0x10BC

#define NOZZLE_TEMP_P_DATA_VP              0x10BE
#define NOZZLE_TEMP_I_DATA_VP              0x10C0
#define NOZZLE_TEMP_D_DATA_VP              0x10C2
#define HOTBED_TEMP_P_DATA_VP              0x10C4
#define HOTBED_TEMP_I_DATA_VP              0x10C6
#define HOTBED_TEMP_D_DATA_VP              0x10C8
#define PRINTER_FAN_SPEED_DATA_VP          0x10CA   //

#define AUTO_BED_LEVEL_1POINT_VP           0x1100
#define AUTO_BED_LEVEL_2POINT_VP           0x1102
#define AUTO_BED_LEVEL_3POINT_VP           0x1104
#define AUTO_BED_LEVEL_4POINT_VP           0x1106
#define AUTO_BED_LEVEL_5POINT_VP           0x1108
#define AUTO_BED_LEVEL_6POINT_VP           0x110A
#define AUTO_BED_LEVEL_7POINT_VP           0x110C
#define AUTO_BED_LEVEL_8POINT_VP           0x110E
#define AUTO_BED_LEVEL_9POINT_VP           0x1110
#define AUTO_BED_LEVEL_10POINT_VP          0x1112
#define AUTO_BED_LEVEL_11POINT_VP          0x1114
#define AUTO_BED_LEVEL_12POINT_VP          0x1116
#define AUTO_BED_LEVEL_13POINT_VP          0x1118
#define AUTO_BED_LEVEL_14POINT_VP          0x111A
#define AUTO_BED_LEVEL_15POINT_VP          0x111C
#define AUTO_BED_LEVEL_16POINT_VP          0x111E

#define CHANGE_SDCARD_ICON_VP              0x1168
#define MOVEAXIS_UNIT_ICON_VP              0x116A
#define PREHAEAT_NOZZLE_ICON_VP            0x116B
#define PREHAEAT_HOTBED_ICON_VP            0x116C
#define FILAMENT_CONTROL_ICON_VP           0x116D
#define POWERCONTINUE_CONTROL_ICON_VP      0x116E

#define PRINT_FINISH_ICON_VP               0x1170 // 打印完成
#define PRINT_REMAIN_TIME_HOUR_VP          0x1171  //剩余时间， 小时数
#define PRINT_REMAIN_TIME_MIN_VP           0x1173  //剩余时间， 分钟数
//#define PRINT_FINISH_ICON_VP               0x1174 // 占用
// #define PRINT_MAIN_PAGE_TIME_VP           0x1175  //

#define MOTOR_FREE_ICON_VP                 0x1200
#define FILE1_SELECT_ICON_VP               0x1221
#define FILE2_SELECT_ICON_VP               0x1222
#define FILE3_SELECT_ICON_VP               0x1223
#define FILE4_SELECT_ICON_VP               0x1224
#define FILE5_SELECT_ICON_VP               0x1225
#define FILE6_SELECT_ICON_VP               0x1226
#define FILE7_SELECT_ICON_VP               0x1227
#define FILE8_SELECT_ICON_VP               0x1228
#define FILE9_SELECT_ICON_VP               0x1229
#define FILE10_SELECT_ICON_VP              0x122A
#define FILE11_SELECT_ICON_VP              0x122B
#define FILE12_SELECT_ICON_VP              0x122C
#define FILE13_SELECT_ICON_VP              0x122D
#define FILE14_SELECT_ICON_VP              0x122E
#define FILE15_SELECT_ICON_VP              0x122F
#define FILE16_SELECT_ICON_VP              0x1230
#define FILE17_SELECT_ICON_VP              0x1231
#define FILE18_SELECT_ICON_VP              0x1232
#define FILE19_SELECT_ICON_VP              0x1233
#define FILE20_SELECT_ICON_VP              0x1234

#define FILE1_TEXT_VP                      0x200A
#define FILE2_TEXT_VP                      0x201E
#define FILE3_TEXT_VP                      0x2032
#define FILE4_TEXT_VP                      0x2046
#define FILE5_TEXT_VP                      0x205A
#define FILE6_TEXT_VP                      0x206E
#define FILE7_TEXT_VP                      0x2082
#define FILE8_TEXT_VP                      0x2096
#define FILE9_TEXT_VP                      0x20AA
#define FILE10_TEXT_VP                     0x20BE
#define FILE11_TEXT_VP                     0x20D2
#define FILE12_TEXT_VP                     0x20E6
#define FILE13_TEXT_VP                     0x20FA
#define FILE14_TEXT_VP                     0x210E
#define FILE15_TEXT_VP                     0x2122
#define FILE16_TEXT_VP                     0x2136
#define FILE17_TEXT_VP                     0x214A
#define FILE18_TEXT_VP                     0x215E
#define FILE19_TEXT_VP                     0x2172
#define FILE20_TEXT_VP                     0x2186

#define SELECT_FILE_TEXT_VP                0x219A
#define PRINT_FILE_TEXT_VP                 0x21C0
#define ABNORMAL_PAGE_TEXT_VP              0x21D4

#define MAIN_PAGE_BLUE_TITLE_VP            0x1300
#define SELECT_FILE_BLUE_TITLE_VP          0x1301
#define PREPARE_PAGE_BLUE_TITLE_VP         0x1302
#define SETTING_PAGE_BLUE_TITLE_VP         0x1303
#define MAIN_PAGE_BLACK_TITLE_VP           0x1304
#define SELECT_FILE_BLACK_TITLE_VP         0x1305
#define PREPARE_PAGE_BLACK_TITLE_VP        0x1306
#define SETTING_PAGE_BLACK_TITLE_VP        0x1307

#define PRINT_ADJUST_MENT_TITLE_VP         0x130D
#define PRINT_SPEED_TITLE_VP               0x130E
#define HEAD_SET_TITLE_VP                  0x130F
#define BED_SET_TITLE_VP                   0x1310
#define LEVEL_ZOFFSET_TITLE_VP             0x1311
#define FAN_CONTROL_TITLE_VP               0x1312
#define LED_CONTROL_TITLE_VP               0x1313

#define MOVE_AXIS_ENTER_GREY_TITLE_VP      0x1314
#define CHANGE_FILAMENT_GREY_TITLE_VP      0x1315
#define PREHAET_PAGE_GREY_TITLE_VP         0x1316
#define MOVE_AXIS_ENTER_BLACK_TITLE_VP     0x1317
#define CHANGE_FILAMENT_BLACK_TITLE_VP     0x1318
#define PREHAET_PAGE_BLACK_TITLE_VP        0x1319

#define PREHEAT_PLA_BUTTON_TITLE_VP        0x131A
#define PREHEAT_ABS_BUTTON_TITLE_VP        0x131B
#define COOL_DOWN_BUTTON_TITLE_VP          0x131C

#define FILAMENT_LOAD_BUTTON_TITLE_VP      0x1321
#define FILAMENT_UNLOAD_BUTTON_TITLE_VP    0x1322

#define LANGUAGE_SELECT_ENTER_VP           0x1323
#define FACTORY_DEFAULT_ENTER_TITLE_VP     0x1324
#define LEVELING_PAGE_TITLE_VP             0x1325

#define PRINTER_DEVICE_GREY_TITLE_VP       0x1326
#define PRINTER_ADVINFO_GREY_TITLE_VP      0x1327
#define PRINTER_INFO_ENTER_GREY_TITLE_VP   0x1328
#define PRINTER_DEVICE_BLACK_TITLE_VP      0x1329
#define PRINTER_ADVINFO_BLACK_TITLE_VP     0x132A
#define PRINTER_INFO_ENTER_BLACK_TITLE_VP  0x132B

#define PREHEAT_PLA_SET_TITLE_VP           0x132D
#define PREHEAT_ABS_SET_TITLE_VP           0x132E

#define STORE_MEMORY_CONFIRM_TITLE_VP      0x1332
#define STORE_MEMORY_CANCEL_TITLE_VP       0x1333

#define FILAMENT_UNLOAD_IGNORE_TITLE_VP    0x133E
#define FILAMENT_USEUP_TITLE_VP            0x133F
#define BUTTON_CHECK_CONFIRM_TITLE_VP      0x1340
#define BUTTON_CHECK_CANCEL_TITLE_VP       0x1341
#define FILAMENT_LOAD_TITLE_VP             0x1342
#define FILAMENT_LOAD_RESUME_TITLE_VP      0x1343
#define PAUSE_PRINT_POP_TITLE_VP           0x1344
#define STOP_PRINT_POP_TITLE_VP            0x1347
#define POWERCONTINUE_POP_TITLE_VP         0x1348
#define AUTO_HOME_WAITING_POP_TITLE_VP     0x1349

#define BEDLEVELING_WAIT_TITLE_VP          0x134B
#define RESTORE_FACTORY_TITLE_VP           0x134D
#define RESET_WIFI_SETTING_TITLE_VP        0x134E
#define KILL_THERMAL_RUNAWAY_TITLE_VP      0x134F
#define KILL_HEATING_FAIL_TITLE_VP         0x1350
#define KILL_THERMISTOR_ERROR_TITLE_VP     0x1351
#define WIND_AUTO_SHUTDOWN_TITLE_VP        0x1352
#define RESET_WIFI_SETTING_BUTTON_VP       0x1353
#define PRINTER_AUTO_SHUTDOWN_TITLE_VP     0x1354
#define WIND_AUTO_SHUTDOWN_PAGE_VP         0x1355
#define AUTO_LEVELING_START_TITLE_VP       0x1356
#define AUX_LEVELING_GREY_TITLE_VP         0x1357
#define AUTO_LEVELING_GREY_TITLE_VP        0x1358
#define AUX_LEVELING_BLACK_TITLE_VP        0x1359
#define AUTO_LEVELING_BLACK_TITLE_VP       0x135A
#define LANGUAGE_SELECT_PAGE_TITLE_VP      0x135B
#define ADV_SETTING_MOTION_TITLE_VP        0x135C
#define ADV_SETTING_PID_TITLE_VP           0x135D
#define ADV_SETTING_WIFI_TITLE_VP          0x135E

#define MOTION_SETTING_TITLE_VP            0x135F
#define MOTION_SETTING_STEPSMM_TITLE_VP    0x1360
#define MOTION_SETTING_ACCEL_TITLE_VP      0x1361
#define MOTION_SETTING_JERK_TITLE_VP       0x1362
#define MOTION_SETTING_VELOCITY_TITLE_VP   0x1363

#define MAX_VELOCITY_SETTING_TITLE_VP      0x1364
#define MAX_VELOCITY_XAXIS_TITLE_VP        0x1365
#define MAX_VELOCITY_YAXIS_TITLE_VP        0x1366
#define MAX_VELOCITY_ZAXIS_TITLE_VP        0x1367
#define MAX_VELOCITY_EAXIS_TITLE_VP        0x1368

#define MAX_ACCEL_SETTING_TITLE_VP         0x1369
#define MAX_ACCEL_XAXIS_TITLE_VP           0x136A
#define MAX_ACCEL_YAXIS_TITLE_VP           0x136B
#define MAX_ACCEL_ZAXIS_TITLE_VP           0x136C
#define MAX_ACCEL_EAXIS_TITLE_VP           0x136D

#define MAX_JERK_SETTING_TITLE_VP          0x136E
#define MAX_JERK_XAXIS_TITLE_VP            0x136F
#define MAX_JERK_YAXIS_TITLE_VP            0x1370
#define MAX_JERK_ZAXIS_TITLE_VP            0x1371
#define MAX_JERK_EAXIS_TITLE_VP            0x1372

#define MAX_STEPSMM_SETTING_TITLE_VP       0x1373
#define MAX_STEPSMM_XAXIS_TITLE_VP         0x1374
#define MAX_STEPSMM_YAXIS_TITLE_VP         0x1375
#define MAX_STEPSMM_ZAXIS_TITLE_VP         0x1376
#define MAX_STEPSMM_EAXIS_TITLE_VP         0x1377

#define TEMP_PID_SETTING_TITLE_VP          0x1378
#define NOZZLE_TEMP_P_TITLE_VP             0x1379
#define NOZZLE_TEMP_I_TITLE_VP             0x137A
#define NOZZLE_TEMP_D_TITLE_VP             0x137B
#define HOTBED_TEMP_P_TITLE_VP             0x137C
#define HOTBED_TEMP_I_TITLE_VP             0x137D
#define HOTBED_TEMP_D_TITLE_VP             0x137E

#define FILAMENT_CONTROL_TITLE_VP          0x137F
#define POWERCONTINUE_CONTROL_TITLE_VP     0x1380

#define MACHINE_TYPE_ABOUT_CHAR_VP         0x1400
#define FIREWARE_VERSION_ABOUT_CHAR_VP     0x1401
#define PRINTER_DISPLAY_VERSION_TITLE_VP   0x1402
#define HARDWARE_VERSION_ABOUT_TITLE_VP    0x1403
#define WIFI_DN_CODE_CHAR_VP               0x1404
#define WEBSITE_ABOUT_CHAR_VP              0x1405
#define PRINTER_PRINTSIZE_TITLE_VP         0x1406

#define LANGUAGE_CHINESE_TITLE_VP          0x1411
#define LANGUAGE_ENGLISH_TITLE_VP          0x1412
#define LANGUAGE_GERMAN_TITLE_VP           0x1413
#define LANGUAGE_SPANISH_TITLE_VP          0x1414
#define LANGUAGE_FRENCH_TITLE_VP           0x1415
#define LANGUAGE_ITALIAN_TITLE_VP          0x1416
#define LANGUAGE_PORTUGUESE_TITLE_VP       0x1417
#define LANGUAGE_RUSSIAN_TITLE_VP          0x1418
#define LANGUAGE_TURKISH_TITLE_VP          0x1419

#define MACHINE_TYPE_ABOUT_TEXT_VP         0x17B0
#define FIREWARE_VERSION_ABOUT_TEXT_VP     0x17C4
#define PRINTER_DISPLAY_VERSION_TEXT_VP    0x17D8
#define HARDWARE_VERSION_ABOUT_TEXT_VP     0x17EC
#define PRINTER_PRINTSIZE_TEXT_VP          0x1800
#define WEBSITE_ABOUT_TEXT_VP              0x1814

#define FilenameNature                     0x6003

#define LCD_LED_CONFIG_REGISTER            0x0082
#define LCD_TP_STATUS_REGISTER             0x0016
#define LCD_SHUTDOWN_LIGHT_LEVEL           0x0A
#define LCD_OPEN_LIGHT_LEVEL               0x64

#define TIME_PRINT_OVER_SHUTDOWN           300

#if HAS_CUTTER // 增加激光

  #define SELECT_LASER_WARNING_TIPS_VP      0x1381
  #define SELECT_FDM_WARNING_TIPS_VP        0x1382  
  // 打印警告界面
  #define PRINT_MOVE_AXIS_VP                0x1383
  #define PRINT_DIRECT_ENGRAV_VP            0x1384
  #define PRINT_RUN_RANGE_VP                0x1385
  #define PRINT_RETURN_VP                   0x1386
  #define PRINT_WARNING_TIPS_VP             0x1387
  //切换激光雕刻
  #define DEVICE_SWITCH_LASER_VP            0x1388 //FDM->设备->切换激光雕刻
  // 首次启动 的设备选择界面
  #define FIRST_SELECT_DEVICE_TYPE          0x1389 //选择打印头类型
  #define HOME_LASER_ENGRAVE_VP             0x138A  //主页 激光雕刻 标题
  //
  #define PREPARE_ADJUST_FOCUS_VP           0x138B
  #define PREPARE_SWITCH_FDM_VP             0x138C

  #define  FIRST_DEVICE_FDM                 0x138D
  #define  FIRST_DEVICE_LASER               0x138E
  #define  FOCUS_SET_FOCUS_TIPS             0x138F
  
  #define SW_FOCUS_Z_VP                     0x2207
//#define SW_FOCUS_Z_VP                     0x138E 占用

#endif

/************struct**************/
typedef struct DataBuf
{
  unsigned char len;
  unsigned char head[2];
  unsigned char command;
  unsigned long addr;
  unsigned long bytelen;
  unsigned short data[32];
  unsigned char reserv[4];
} DB;

typedef struct CardRecord
{
  int recordcount;   //当前选中的文件序列
  int Filesum;       //文件总数目
  unsigned long addr[FileNum];   //迪文屏上显示对应文件名的地址
  char Cardshowfilename[FileNum][FileNameLen];   //长文件名
  char Cardfilename[FileNum][FileNameLen];       //短文件名
}CRec;

// extern CRec CardRecbuf;

class RTSSHOW
{
  public:
    RTSSHOW(void);
    int RTS_RecData(void);
    void RTS_SDCardInit(void);
    bool RTS_SD_Detected(void);
    void RTS_SDCardUpate(void);
    void languagedisplayUpdate(void);
    void RTS_SndData(void);
    void RTS_SndData(const String &, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(const char[], unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(char, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned char*, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(int, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(float, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned int,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SDcard_Stop(void);
    void RTS_HandleData(void);
    void RTS_Init(void);

    #if HAS_CUTTER
      void RTS_HandleData_Laser(void);
      void RTS_SDcard_Stop_laser(void);
    #endif   

    DB recdat;
    DB snddat;
  private:
    unsigned char databuf[SizeofDatabuf];
};

extern RTSSHOW rtscheck;

#if ENABLED(HAS_MENU_RESET_WIFI)
  enum sWIFI_STATE
  {
    INITIAL = 0,
    PRESSED,
    RECORDTIME,
  };
#endif

#if ENABLED(HAS_MENU_RESET_WIFI)
  extern unsigned char WIFI_STATE;
#endif

enum PROC_COM
{
  MainEnterKey          = 0,
  AdjustEnterKey        = 1,
  PrintSpeedEnterKey    = 2,
  StopPrintKey          = 3,
  PausePrintKey         = 4,
  ResumePrintKey        = 5,
  ZoffsetEnterKey       = 6,
  TempControlKey        = 7,
  CoolDownKey           = 8,
  HeaterTempEnterKey    = 9,
  HotBedTempEnterKey    = 10,
  PrepareEnterKey       = 11,
  BedLevelKey           = 12,
  AutoHomeKey           = 13,
  XaxismoveKey          = 14,
  YaxismoveKey          = 15,
  ZaxismoveKey          = 16,
  HeaterLoadEnterKey    = 17,
  HeaterUnLoadEnterKey  = 18,
  HeaterLoadStartKey    = 19,
  SelectLanguageKey     = 20,
  PowerContinuePrintKey = 21,
  PLAHeadSetEnterKey    = 22,
  PLABedSetEnterKey     = 23,
  ABSHeadSetEnterKey    = 24,
  ABSBedSetEnterKey     = 25,
  StoreMemoryKey        = 26,
  FanSpeedEnterKey      = 27,
  VelocityXaxisEnterKey = 28,
  VelocityYaxisEnterKey = 29,
  VelocityZaxisEnterKey = 30,
  VelocityEaxisEnterKey = 31,
  AccelXaxisEnterKey    = 32,
  AccelYaxisEnterKey    = 33,
  AccelZaxisEnterKey    = 34,
  AccelEaxisEnterKey    = 35,
  JerkXaxisEnterKey     = 36,
  JerkYaxisEnterKey     = 37,
  JerkZaxisEnterKey     = 38,
  JerkEaxisEnterKey     = 39,
  StepsmmXaxisEnterKey  = 40,
  StepsmmYaxisEnterKey  = 41,
  StepsmmZaxisEnterKey  = 42,
  StepsmmEaxisEnterKey  = 43,
  NozzlePTempEnterKey   = 44,
  NozzleITempEnterKey   = 45,
  NozzleDTempEnterKey   = 46,
  HotbedPTempEnterKey   = 47,
  HotbedITempEnterKey   = 48,
  HotbedDTempEnterKey   = 49,
  PrintFanSpeedkey      = 50,
  ChangePageKey         = 51,
  ErrorKey              = 52,
  StartFileKey          = 53,
  SelectFileKey         = 54,
  
  //新增激光
  #if HAS_CUTTER 
    SwitchDeviceKey     = 55,
    PauseEngraveingKey  = 56,
    EngraveWarningKey   = 57,
    AdjustFocusKey      = 58, 
    SwAdjustFocusKey    = 59,
    LaserMoveAxis       = 60,
    FocusZAxisKey       = 61,
    //FocusZAxisKey       = 62, //被占用
  #endif

};

const unsigned long Addrbuf[] = 
{
  0x1002,
  0x1004,
  0x1006,
  0x1008,
  0x100A,   // 4
  0x100C,
  0x1026,
  0x1030,
  0x1032,
  0x1034,   // 9
  0x103A,
  0x103E,
  0x1044,
  0x1046,
  0x1048,   // 14
  0x104A,
  0x104C,
  0x1052,
  0x1054,
  0x1056,   // 19
  0x105C,
  0x105F,
  0x1090,
  0x1092,
  0x1094,   // 24
  0x1096,
  0x1098,
  0x109A,
  0x109E,
  0x10A0,   // 29
  0x10A2,
  0x10A4,
  0x10A6,
  0x10A8,
  0x10AA,   // 34
  0x10AC,
  0x10AE,
  0x10B0,
  0x10B2,
  0x10B4,   // 39
  0x10B6,
  0x10B8,
  0x10BA,
  0x10BC,
  0x10BE,   // 44
  0x10C0,
  0x10C2,
  0x10C4,
  0x10C6,
  0x10C8,   // 49
  0x10CA,
  0x110E,
  0x111A,
  0x2198,
  0x2199,  // 54

  //新增激光
  #if HAS_CUTTER
    0x2201, //SwitchDeviceKey
    0x2202,
    0x2203, //EngraveWarningKey
    0x2204,
    0x2205, // 调整激光焦距
    0x2206, // 移动轴
    0x2207, // 调整焦距
    //0x2208, 被占用
  #endif
  0
};

void ErrorHanding(void);
extern void RTSUpdate(void);
extern void RTSInit(void);

#if HAS_CUTTER
  extern void RTSUpdateLaser(void);
#endif

extern float zprobe_zoffset;
extern char waitway;
extern int change_page_font;
extern uint8_t language_change_font;
extern uint8_t lang;
// extern uint8_t wifi_enable_flag;
extern int Update_Time_Value;
extern bool PoweroffContinue;
// extern bool sdcard_pause_check;
// extern bool sd_printing_autopause;

// extern unsigned char AxisUnitMode;
extern bool home_flag;
// extern bool G29_flag;
extern bool heat_flag;
// extern bool AutohomeZflag;
extern char commandbuf[30];

extern bool StartPrint_flag;
// extern bool pause_action_flag;

extern char errorway;
extern char errornum;
extern char error_sd_num;

// extern bool home_count;

extern unsigned char Count_first;

extern unsigned char Count_probe; 

extern float z_offset;

extern bool eeprom_save_flag;

#define EEPROM_SAVE_LANGUAGE()      {if(eeprom_save_flag) { settings.save(); eeprom_save_flag = false; }}

// extern bool flag_over_shutdown; 
// extern bool flag_counter_printover_to_shutdown; 

void Read_lcd_Register(unsigned char len, unsigned int addr);
void Write_lcd_Register(unsigned int addr,unsigned char data);
void lcd_eight_language(void);

void RTS_PauseMoveAxisPage(void);
void RTS_AutoBedLevelPage(void);
void RTS_MoveAxisHoming(void);
void RTS_MoveParkNozzle(void);
void RTS_CommandPause(void);
#endif
