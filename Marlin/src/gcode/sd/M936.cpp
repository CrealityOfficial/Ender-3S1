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
#include "../../core/serial.h"
#include "../../../Configuration.h"
#include "../../libs/BL24CXX.h"
#include "../gcode.h"
#include "stdio.h"
#include "string.h"
#include "../parser.h"

#include "../../lcd/dwin/dwin_lcd.h" 

#define ENABLE_OTA
#if ENABLED(ENABLE_OTA)
typedef unsigned char u8;
u8* strchr_pointer; 

#define OTA_FLAG_EEPROM  90

/**
 * M936: OTA update firmware.
 */
void GcodeSuite::M936()
{
  // TODO: feedrate support?
  static u8 ota_updata_flag = 0X00;
  // char temp[20];
  if(parser.str_contain("V2"))
  {
    ota_updata_flag = 0X01;
    BL24CXX::write(OTA_FLAG_EEPROM, &ota_updata_flag, sizeof(ota_updata_flag));
    delay(10);
    SERIAL_ECHOLN("M936 V2");
    // 需要将OTA升级标志位设置成1，表示下次上电需要OTA升级。
    delay(50);
    SERIAL_ECHOLN("\r\n MCU upgrade \r\n");
    delay(50);
    DWIN_Backlight_SetLuminance(0); //
    //nvic_sys_reset();   // MCU复位进入BOOTLOADER
  }
  else if(parser.str_contain("V3"))
  {
    ota_updata_flag = 0X02;
    BL24CXX::write(OTA_FLAG_EEPROM, &ota_updata_flag, sizeof(ota_updata_flag));
    delay(10);
    SERIAL_ECHOLN("M936 V3");
    // 需要将OTA升级标志位设置成1，表示下次上电需要OTA升级。
    delay(50);
    SERIAL_ECHOLN("\r\n DIWIN upgrade！！ \r\n");
    delay(50);
     DWIN_Backlight_SetLuminance(255); //
    //nvic_sys_reset();   // MCU复位进入BOOTLOADER
  }

  // BL24CXX::read(OTA_FLAG_EEPROM, &read_arr, sizeof(read_arr));
  // if((0 == f) && (0 == j))
  // {
  //   SERIAL_ECHOLN("updata ota start!!!!");
  // }
  // else
  // {
  //   SERIAL_ECHOLN("OTA UPDATA FAILED  ");
  // }
}

#endif // DIRECT_STEPPING
