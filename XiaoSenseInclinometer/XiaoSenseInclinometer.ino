// XIAO INCLINOMETER PREAMBLE ###############################################################

  /*****************************************************************************/
  //  HighLevelExample.ino
  //  Hardware:      Grove - 6-Axis Accelerometer&Gyroscope
  //	Arduino IDE:   Arduino-1.65
  //	Author:	       Lambor
  //	Date: 	       Oct,2015
  //	Version:       v1.0
  //
  //  Modified by:  ~MHz (a.k.a. Tod Wulff)
  //  Data:         13-14 Apr 2024
  //  Description:  Adapted for use with a Seeed Studio XIAO nRF52840 Sense
  //                Added 128x32 i2c oled display for pitch/roll (FPV use)
  //                Added LEDs for driver visual queue of approaching tip over
  //                Added switched output for driving an aural warning device
  //                See video:  https://youtu.be/2NaQaHJ9XEI
  //
  //  Inspiration/References:
  //  Scale Built RC's Roll-o-Meter - Thanks brother!
  //  https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
  //  https://github.com/camerontech/inclinometer
  //  https://forum.seeedstudio.com/t/xiao-sense-accelerometer-examples-and-low-power/270801
  //  https://forum.seeedstudio.com/t/seeed-xiao-ble-nrf52840-sense-giving-bad-gyroscope-data/274134/11
  //  https://adam-meyer.com/arduino/sensing-orientation-with-the-adxl335-arduino  <-- this
  //  + a bunch of others for the oled display (using u8g2 lib)
  //
  //  14Apr24 Sketch Stats on Xiao nRF52840 Sense Target: 
  //  Sketch uses 70664 bytes (8%) of program storage space. Maximum is 811008 bytes.
  //  Global variables use 8412 bytes (3%) of dynamic memory, leaving 229156 bytes for 
  //  local variables. Maximum is 237568 bytes.
  //
  //  15APR24:  Ok, explored writing config values to flash.  Decided on making use of LittleFS.
  //  Installed the library.  Went to run the test sketch and was forced to migrate to the
  //  MBed context on the Xiao nRF52840 Sense.  It ended up working once I did so.
  //  The migration over to the MBed core however, was met with some IO weirdness - pin asssignments
  //  have changed - the IO I am using is slik screened 10 to 6 (0 indexed).  Now those same pins are
  //  referenced as pin 11 to 7 (maybe now indexed from 1.?.).  See the following link for specifics:
  //  https://github.com/Seeed-Studio/ArduinoCore-mbed/commit/e33b182c4b935b25742595d2907a0f04b965cce6#diff-55b586ca1ce12fa1ac4423cdcb31f05ec84ceaa151784a25d555f3a15620864a
  //
  //  Possible enhancements to consider:
  //    - Add cute startup splash screen w/ static, or moving, graphics
  //    - Flashing LEDs in caution and/or warning contexts
  //    - Pulsing Aural Warnings (vs. static tone) in caution and/or warning contexts
  //    - Graphics depicting truck and angle indicators (see https://www.youtube.com/watch?v=PfeZpKERkO8)
  //    - Flashing Display graphics or tossing a warning display on the oled
  //      (see https://www.facebook.com/watch/?v=2394412913939607)
  //    - High resolution color UI on appropriate glass 
  //      (see https://www.facebook.com/scalebuilt/videos/605197420078915)
  //
  //  by www.seeedstudio.com
  //
  //  This library is free software; you can redistribute it and/or
  //  modify it under the terms of the GNU Lesser General Public
  //  License as published by the Free Software Foundation; either
  //  version 2.1 of the License, or (at your option) any later version.
  //
  //  This library is distributed in the hope that it will be useful,
  //  but WITHOUT ANY WARRANTY; without even the implied warranty of
  //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  //  Lesser General Public License for more details.
  //
  //  You should have received a copy of the GNU Lesser General Public
  //  License along with this library; if not, write to the Free Software
  //  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  //
  /*******************************************************************************/

  // todos (as of 18APR24): 
  // - adapt multiple LEDs to one or more neopixel-ish APA106 (5mm and 8mm are on order)
  // - to support multiple OLED geometries, make dynamic by query of u8g2 for display dims
  // - make orientation assertions more intuitive, add lateral orientations support too
  // - consider supporting truly dynamic installation orientations (may need to make use of 
  //   rotation matricies || quaternions to prevent gimbal lock)
  // - evaluate expanding sensed accelerometer range, as sensor is 8G/16G, I perceive
  //   but it works as is, so may choose to not muck with what isn't broken...
  // - layer on some UI frosting - visual flashing/iconage, aural toneage, etc.
  // - consider muting the aural warning after _ seconds if static
  // - consider having single short button press when running to 'cage' orientation (i.e.
  //   applying axial offsets, if practical, given an accelerometer-only employment) 
  // - nRF52840 WDT implementation is pretty underwhelming - likely by conservative design.
  //   Once started, can't pause it, can't reconfigure until reset, pretty much locked down
  //   once started.  As such only using it for software reset purposes in this app.  As
  //   such, this todo is to evaluate timer interrupts to implement a software WDT that calls 
  //   an ISR to start WDT with a CRV of 1 to implement a reset in ~31uS - that ISR is the 
  //   existing resetInclinometer(); function which is indeed using the WDT w/ CRV of 1...

  // completed (as of 18APR24)
  // - consider employing preprocessor #defs in favor of const.?.
  // - employ use of a file system for persistent config storage
  // - add switch input (temporary connection?) to enable intuitive calibration once installed
  //   then tip to just prior tipping point to set Warning (maybe with a buffer) via button
  //   press then have a Caut offset of _ degrees.  This would be done in all four directions
  //   as CG offsets will likely impute deltas between fore/aft and left/right tip points.
  //   'Calibrated Tip Points' will require NVM storage in flash
  // - add rotating char to corner to depict loop running cycle through: - \ | / 
  // - figure out how to do a SW reset on the nRF52840 so Inclinometer can be forcibly reset
  //   after a configuration event <-- made use of the WDT with a CRV of 0x01 = ~31uS

  /*

  musings to help get a coa established.  So the FS is installed, so that calibration can be
  completed once the Xiao is mounted.  Thinking the following:

  on boot,
  - if config.ini file is MIA, assign and display default tip caution/warning points
  - if config.ini file exists, read, assign, and display config'd tip caution/warning points

  with the tip caution and warning points instantiated and main loop running, enter
  into the config state by tipping the vehicle into a warning context (led red w/ aural warn)
  and then pressing the button.  
  
  display instruction to orient to rt tip point and press button to record +roll value
  display instruction to orient to lt tip point and press button to record -roll value
  display instruction to orient to fore tip point and press button to record +pitch value
  display instruction to orient to back tip point and press button to record -pitch value

  now, could repeat same process for caution points, but I think the better approach is to
  have an option to select warn less 5, 10, 15, ... degrees.  I anticipate that selecting same
  could be done by displaying 5 and if button pressed, save the caution point as warn less 5
  can increment the 5 to 10 by tipping the venicle to the warning value and the caution offset
  would be incremented by 5 degrees - i.e. 5 to 10 and if tipped again, 10 to 15 to ...
  could have a right tip increment by 5 and a left tip decrement by 1.?.  The minimum delta
  between warning and caution is 1.  max delta could be set to w/e is needed for an absolute
  30 degree tip (all? vehicles should be able to handle a 30 degree grade on the cardinal
  points, I suspect...)

  tip into warning and if button pressed, set warning and cautions to some not attainable value
  to disable the warnings/cautions and enter into a config session

  during a config sesion display

  WnRt: xxx (xxx is sensed roll value is positive roll and greater than 30 deg) - --- otherwise
  when button pressed, store the WnRt value as a cal point - then proceed to WnLt

  WnLt: xxx (xxx is sensed roll value is negative roll and greater than 30 deg) - --- otherwise
  when button pressed, store the WnLt value as a cal point - then proceed to WnFw

  WnFw: xxx (xxx is sensed roll value is positive pitch and greater than 30 deg) - --- otherwise
  when button pressed, store the WnFw value as a cal point - then proceed to WnBk

  WnBk: xxx (xxx is sensed roll value is negative pitch and greater than 30 deg) - --- otherwise
  when button pressed, store the WnBk value as a cal point - then proceed to set CaOSRt

  enable warnings at this point so that warnings can be used as input queueing

  CaOSRt: xxx (xxx starts at 001) increment by 5 when tipped rt, decrement by 1 when tipped lt
  Caution Offset val to be bound from 1/-1 to 20/-20 (rt/lt) during adj gestures cycle, looped
  when button pressed, store the CaOSRt value as a cal point - then proceed to CaOSLt

  CaOSLt: xxx (xxx starts at !CaOSRt) increment by 5 when tipped rt, decrement by 1 when tipped lt
  Caution Offset val to be bound from 1/-1 to 20/-20 (rt/lt) during adj gestures cycle, looped
  when button pressed, store the CaOSLt value as a cal point - then proceed to CaOSFw

  CaOSFw: xxx (xxx starts at 001) increment by 5 when tipped rt, decrement by 1 when tipped lt
  Caution Offset val to be bound from 1/-1 to 20/-20 (rt/lt) during adj gestures cycle, looped
  when button pressed, store the CaOSFw value as a cal point - then proceed to CaOSBk

  CaOSBk: xxx (xxx starts at !CaOSFw) increment by 5 when tipped rt, decrement by 1 when tipped lt
  Caution Offset val to be bound from 1/-1 to 20/-20 (rt/lt) during adj gestures cycle, looped
  when button pressed, store the CaOSBk value as a cal point - then proceed to displaying settings

  cycle through the 8 set points - literal degrees for Wn__, and literal degrees for CaOS__ with
  (offset degrees in parens)

  cycle sequence:

  WnRt: +xx°R
  WnLt: -xx°R
  WnFw: +xx°P
  WnBk: -xx°P
  CaOSRt: +xx°R/xx°
  CaOSLt: -xx°R/xx°
  CaOSFw: +xx°P/xx°
  CaOSBk: -xx°P/xx°

  on button press in non-warn state, save the values to the config.ini file and return to running mode

  */

  // ########################################################################################

// LITTLE FS PREPROCESSOR DIRECTIVES ########################################################

  // LittleFS lib stuiffs  -  Added the first #def because:
  // https://forum.seeedstudio.com/t/how-to-store-persistent-data-on-xiao-ble-sense/266711/3

  #define ARDUINO_SEEED_XIAO_NRF52840     1

  #define FS_NANO33BLE_VERSION_MIN_TARGET "FS_Nano33BLE v1.2.1"
  #define FS_NANO33BLE_VERSION_MIN        1002001

  // Debug is enabled by default on Serial.
  // #define FS_DEBUG_OUTPUT    Serial

  // You can also change the debugging level (FS_LOGLEVEL) from 0 to 4 - default was 1 
  #define _FS_LOGLEVEL_                   4

  // Min NANO33BLE_FS_SIZE_KB must be  64KB. If defined smalller => auto adjust to  64KB
  // Max NANO33BLE_FS_SIZE_KB must be 512KB. If defined larger   => auto adjust to 512KB
  #define NANO33BLE_FS_SIZE_KB            256

  #define FORCE_REFORMAT                  false

  // Default USING_LITTLEFS. Uncomment to not USING_LITTLEFS => USING_FATFS.
  // It's advisable not to use FATFS, as the NANO33BLE_FS_SIZE_KB must be auto-adjusted to 512KB
  //#define USING_LITTLEFS                false

  #include <FS_Nano33BLE.h>

  // actual instantiation of LittleFS mBed wrapper
  FileSystem_MBED *myFS;

  // Xiao Inclinometer FS stuffs
  #define _fs_BUFF_SIZE                   512
  static uint8_t _fs_buf[_fs_BUFF_SIZE];

  // lfs error numbers
  #define LFS_ERR_OK           0
  #define LFS_ERR_IO           -5
  #define LFS_ERR_CORRUPT      -84
  #define LFS_ERR_NOENT        -2
  #define LFS_ERR_EXIST        -17
  #define LFS_ERR_NOTDIR       -20
  #define LFS_ERR_ISDIR        -21
  #define LFS_ERR_NOTEMPTY     -39
  #define LFS_ERR_BADF         -9
  #define LFS_ERR_FBIG         -27
  #define LFS_ERR_INVAL        -22
  #define LFS_ERR_NOSPC        -28
  #define LFS_ERR_NOMEM        -12
  #define LFS_ERR_NOATTR       -61
  #define LFS_ERR_NAMETOOLONG  -36
  
  // ########################################################################################

// LITTLE FS LIBRARY PROCS ##################################################################

  void readCharsFromFile(const char * path) {
    Serial.print("readCharsFromFile: ");
    Serial.print(path);

    FILE *file = fopen(path, "r");

    if (file) {
      Serial.println(" => Open OK");
      } else {
      Serial.println(" => Open Failed");
      return;
      }

    char c;

    while (true) {
      c = fgetc(file);

      if ( feof(file) ) {
        break;
        } else {
        Serial.print(c);
        }
      }

    fclose(file);
    }

  void readFile(const char * path) {
    Serial.print("Reading file: ");
    Serial.print(path);

    FILE *file = fopen(path, "r");

    if (file) {
      Serial.println(" => Open OK");
      } else {
      Serial.println(" => Open Failed");
      return;
      }

    char c;
    uint32_t numRead = 1;

    while (numRead) {
      numRead = fread((uint8_t *) &c, sizeof(c), 1, file);

      if (numRead)
        Serial.print(c);
      }

    fclose(file);
    }

  void writeFile(const char * path, const char * message, size_t messageSize) {
    Serial.print("Writing file: ");
    Serial.print(path);

    FILE *file = fopen(path, "w");

    if (file) {
        Serial.println(" => Open OK");
      } else {
        Serial.println(" => Open Failed");
        return;
      }

    if (fwrite((uint8_t *) message, 1, messageSize, file)) {
        Serial.println("* Writing OK");
      } else {
        Serial.println("* Writing failed");
      }

    fclose(file);
    }

  void appendFile(const char * path, const char * message, size_t messageSize) {
    Serial.print("Appending file: ");
    Serial.print(path);

    FILE *file = fopen(path, "a");

    if (file)
    {
      Serial.println(" => Open OK");
    }
    else
    {
      Serial.println(" => Open Failed");
      return;
    }

    if (fwrite((uint8_t *) message, 1, messageSize, file))
    {
      Serial.println("* Appending OK");
    }
    else
    {
      Serial.println("* Appending failed");
    }

    fclose(file);
    }

  void deleteFile(const char * path) {
    Serial.print("Deleting file: ");
    Serial.print(path);

    if (remove(path) == 0)
    {
      Serial.println(" => OK");
    }
    else
    {
      Serial.println(" => Failed");
      return;
    }
    }

  void renameFile(const char * path1, const char * path2) {
    Serial.print("Renaming file: ");
    Serial.print(path1);
    Serial.print(" to: ");
    Serial.print(path2);

    if (rename(path1, path2) == 0)
    {
      Serial.println(" => OK");
    }
    else
    {
      Serial.println(" => Failed");
      return;
    }
    }

  // ########################################################################################




// XIAO INCLINOMETER PREPROCESSOR DIRECTIVES ################################################

  // digital io assignments for LEDs and Aural Warning Device
  #define _io_GLed 11   //10 w/ arduino core - mbed core was met with an update to IO IDs...
  #define _io_OLed 10   //9
  #define _io_RLed 9    //8
  #define _io_WHorn 8   //7
  #define _io_UsrSw 7   //6

  // states
  #define led_On  0
  #define led_Off  1
  #define wHorn_On  0
  #define wHorn_Off  1

  // for readability
  #define _gLedOff  digitalWrite(_io_GLed,led_Off)
  #define _gLedOn   digitalWrite(_io_GLed,led_On)
  #define _oLedOff  digitalWrite(_io_OLed,led_Off)
  #define _oLedOn   digitalWrite(_io_OLed,led_On)
  #define _rLedOff  digitalWrite(_io_RLed,led_Off)
  #define _rLedOn   digitalWrite(_io_RLed,led_On)
  #define _wHornOff digitalWrite(_io_WHorn,wHorn_Off)
  #define _wHornOn  digitalWrite(_io_WHorn,wHorn_On)

  #define _switchReleased (digitalRead(_io_UsrSw))
  #define _switchPressed  (!_switchReleased)

  // normalizing accelerometer values per guidance: 
  // https://adam-meyer.com/arduino/sensing-orientation-with-the-adxl335-arduino
  // given raw is -1.x|1.x, 'normalized' to 0|2047  
  // normalized = (raw * multiplier) + offset
  #define _accelNormOffset  1024
  #define _accelNormMulti   1023

  // max/min 'normalized' values - from -1.0|1.0 map'd to 0|2047 - a -1G-1G range
  #define  _accNormMin 0
  #define  _accNormMax 2047

  // delays
  #define _splashDly  3000
  #define _imuDly     3000
  #define _lfsDly     3000
  #define _cfgDly     3000
  #define _initDly    3000
  #define _dispDly    3000
  #define _ioDly      3000
  #define _commsDly   3000

  // common _ui Window element ##############################################################

  #define _uiColor_BlackOnWhite 0
  #define _uiColor_WhiteOnBlack 1
  
  // experimental _ui display controls ######################################################

  #define _splashOffset1X
  #define _splashOffset2X
    
  // for config file key/value pair #########################################################

  #define kvStrLen  63
    
  // _ui_Data Window Display Elements #######################################################

    // font for labels/data/symbols/etc
    #define _ui_DataFont  u8g2_font_10x20_te

    // top half is for data display
    #define _ui_DataWinX  0
    #define _ui_DataWinY  0
    #define _ui_DataWinW  128
    #define _ui_DataWinH  16

    // labels
    #define _ui_PitchLblTxtOffsetX 0
    #define _ui_PitchLblTxtOffsetY 14

    #define _ui_RollLblTxtOffsetX 65
    #define _ui_RollLblTxtOffsetY 14

    // pitch text offsets in data win
    #define _ui_PitchDataTxtOffsetX 13
    #define _ui_PitchDataTxtOffsetY 14

    // roll text offsets in data win
    #define _ui_RollDataTxtOffsetX 78
    #define _ui_RollDataTxtOffsetY 14

    #define _ui_DataDegSymOffsetX  -7
    #define _ui_DataDegSymOffsetY  -1

    // ######################################################################################

  // _ui_Status Window Display Elements #####################################################

    // font for status/prompts/etc.
    #define _ui_StatFont  u8g2_font_6x12_te

    // bottom half is for icons/status/prompts/etc
    #define _ui_StatWinX  0
    #define _ui_StatWinY  17
    #define _ui_StatWinW  128
    #define _ui_StatWinH  16

    // heartbeat text offsets in stat win
    #define _ui_HBStatTxtOffsetX 0
    #define _ui_HBStatTxtOffsetY 12

    // heartbeat text offsets in stat win
    #define _ui_PromptTxtOffsetX 9
    #define _ui_PromptTxtOffsetY 12

    // period in mS to force a device reset if the switch is continuously held
    #define _switchHeldResetPeriod 5000

    // runtime configuration elements 
    #define _configFileName MBED_FS_FILE_PREFIX "/XiaoInclinometer.ini"

  // #Includes ##############################################################################

    #include "LSM6DS3.h"
    #include <Arduino.h>
    #include <U8g2lib.h>
    #include <SPI.h>
    #include "Wire.h"

  // ########################################################################################

// XIAO INCLINOMETER SETUP ##################################################################

  // Create a instance of class LSM6DS3 (the 6Dof IMU)
  LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

  // assert constructor for given OLED display
  U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); // this is the 0.91" 128x32 small OLED display w/ ssd1306 driver IC

  // required by u8g2 lib - needs ram-based vars for use
  char msgStr[63];     // for use w/ u8g2 lib var string
  char statusStr[63];  // for use w/ u8g2 lib var string

  // for use in parsing config file
  char kvBuf[kvStrLen];    // buffer string 
  char keyStr[kvStrLen];    // key
  char valStr[kvStrLen];  // value

  int cWidth = 9;         // todo: set via query of u8g2 lib for specified font during setup

  int_fast64_t timeStart = 0;
  int_fast64_t timeNow = 0;
  int_fast64_t timeEnd = 0;
  int_fast64_t loopCnt = 0;
  double loopRate = 0;

  char heartBeat[] = "-\\|/";
  int hbIdx = 0;

  int pitch = 0;
  int roll = 0;


  // these are ini file items:

  // Xiao Sense SBC installed orientation correction flag and sensed angle trip point defaults
  // these will be updated with ini file values during setup
  // currently (15APR24), supports one of 4 longitudinal orientations only (i.e. usb fore/aft, either inverted or not)

  // orientation inverted with usb connector towards nose
  int bInv = -1;    // board inversion flag: 1 for non-inverted or -1 to invert
  int pDir = 1;     // pitch inversion flag: 1 for non-inverted or -1 to invert
  int rDir = -1;    //  roll inversion flag: 1 for non-inverted or -1 to invert

  // uncomment one of the following blocks as appropriate for installed orientation
  // for inverted with usb connector towards nose
  //int bInv = -1;    // board inversion flag: 1 for non-inverted or -1 to invert
  //int pDir = 1;     // pitch inversion flag: 1 for non-inverted or -1 to invert
  //int rDir = -1;    //  roll inversion flag: 1 for non-inverted or -1 to invert

  // for non-inverted with usb connector towards bow
  //int bInv = 1;     // board inversion flag: 1 for non-inverted or -1 to invert
  //int pDir = -1;    // pitch inversion flag: 1 for non-inverted or -1 to invert
  //int rDir = -1;    //  roll inversion flag: 1 for non-inverted or -1 to invert

  // for inverted with usb connector towards tail
  // int bInv = -1;   // board inversion flag: 1 for non-inverted or -1 to invert
  // int pDir = -1;   // pitch inversion flag: 1 for non-inverted or -1 to invert
  // int rDir = 1;    //  roll inversion flag: 1 for non-inverted or -1 to invert

  // for non-inverted with usb connector towards tail
  // int bInv = 1;    // board inversion flag: 1 for non-inverted or -1 to invert
  // int pDir = 1;    // pitch inversion flag: 1 for non-inverted or -1 to invert
  // int rDir = 1;    //  roll inversion flag: 1 for non-inverted or -1 to invert

  // setup: determine/set these after the sensor is installed in the vehicle and tested for pitch and roll stability
  int pTipFwdAngCaut = 30;
  int pTipFwdAngWarn = 45;

  int pTipAftAngCaut = -30;
  int pTipAftAngWarn = -45;

  int rTipRtAngCaut = 30;
  int rTipRtAngWarn = 45;

  int rTipLtAngCaut = -30;
  int rTipLtAngWarn = -45;

  // ########################################################################################

// XIAO INCLINOMETER APP PROCS ##############################################################
  void resetInclinometer() {
    //Configure & start WDT with shortest period, to force an 'immediate' reset via SW
    Serial.println("\nResetting Processor via WDT");
    NRF_WDT->CONFIG         = 0x01;     // Configure WDT to run when CPU is asleep
    NRF_WDT->CRV            = 1;        // load CRV - with 1 - should reset at Start + ~30uS (32.768KHz based)
    NRF_WDT->RREN           = 0x01;     // Enable the RR[0] reload register
    NRF_WDT->TASKS_START    = 1;        // Triggers the Start WDT - not stoppable hereafter , until post-reset...      
    }

  void showSplash() {

    clearDisplay();
    u8g2.setFont(_ui_DataFont);
    u8g2.drawStr(0,15,"   ~MHz");
    u8g2.drawStr(0,30,"Roll-o-Meter");
    u8g2.sendBuffer();

    delay(_splashDly);

    }

  void mountFS() {

    clearDisplay();
    u8g2.setFont(_ui_DataFont);

    // spin up and mount fs
    #if defined(FS_NANO33BLE_VERSION_MIN)

      if (FS_NANO33BLE_VERSION_INT < FS_NANO33BLE_VERSION_MIN) {
        Serial.print("Warning. Must use this example on Version equal or later than : ");
        Serial.println(FS_NANO33BLE_VERSION_MIN_TARGET);
        }

      #endif
    //

    Serial.print("\nLittle File System Version: ");
    Serial.println(FS_NANO33BLE_VERSION);
    Serial.print("#Defined FS_size (KB) = ");
    Serial.println(NANO33BLE_FS_SIZE_KB);
    Serial.print("Calculated FS_ Start Address = 0x");
    Serial.print(NANO33BLE_FS_START, HEX);
    Serial.println(" (derived from above #def)");

    Serial.println("\nMounting LittleFS now...");

    myFS = new FileSystem_MBED();

    if (!myFS->init()) {  // FS Mount Failed

      Serial.println("FS Mount Failed - exiting app...");

      u8g2.clearBuffer(); // clear the internal memory
      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"FS Borked");
      u8g2.sendBuffer();

      delay(_lfsDly);

      u8g2.clearBuffer(); // clear the internal memory
      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Exiting App...");
      u8g2.sendBuffer();

      return; // consider not exiting but rather running with instantiated defaults regardless

    } else {  // FS Mount Succeeded

      u8g2.clearBuffer(); // clear the internal memory
      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"FS is Mounted");
      u8g2.sendBuffer();

      }

    delay(_lfsDly);

    }  

  void deleteConfig() {

      Serial.print("Deleting file: ");
      Serial.print(_configFileName);

      if (remove(_configFileName) == 0)
      {
        Serial.println(" => OK");
      }
      else
      {
        Serial.println(" => Failed");
        return;
      }
    }

  void createConfig() {

    char message[63];
    deleteConfig();

    strcpy(message,"bInv=-1\n");
    writeFile(_configFileName, message, sizeof(message));

    strcpy(message,"pDir=1\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"rDir=-1\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"pTipFwdAngCaut=30\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"pTipFwdAngWarn=45\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"pTipAftAngCaut=-30\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"pTipAftAngWarn=-45\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"rTipRtAngCaut=30\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"rTipRtAngWarn=45\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"rTipLtAngCaut=-30\n");
    appendFile(_configFileName, message, sizeof(message));

    strcpy(message,"rTipLtAngWarn=-45\n");
    appendFile(_configFileName, message, sizeof(message));

    readFile(_configFileName);

    }
  
  void updateConfig() {
  
    }

  void loadConfig() {
    
    /*
    clearDisplay();
    u8g2.setFont(_ui_DataFont);

    // init app from config file, if exists, or defaults -> n ew config file, if it doesn't...

    char configFN[] = _configFileName;

    // try to open file
    // on error, create file and write values to same
    // on OK, parse and assign var values from content

    Serial.print("\nChecking Config File: ");
    Serial.print(configFN);

    FILE *file = fopen(configFN, "r");

    if (file) {
      Serial.println(" => Open OK");

      // parse and assign

      char c;
      int kvFlag = 0;  // 0 = key, 1 = value
      int kvpIdx = 0;  // counter of key/value pair 
      int kvBufIdx = 0; // buf pos index

      // zero out kv/buf char arrays
      for (int i=0;i<kvStrLen;i++) {kvBuf[i] = 0; keyStr[i] = 0; valStr[i] = 0; }

      while (true) {
        c = fgetc(configFN); // get char from file

        if (feof(configFN)) {// see if it is end of file, if so
          // test to ensure that we're building a value, raising an error if not, else
          // assign buf as key's val and break out of loop

          break;
        } else if (0) { // test for "=" and !kvFlag to see if we are done with key building 
          // ?? do a switch case here to compare key with expected keys and assign val to the matching key
          // ?? if no matching key (case default), raise an exception, else
          // reset bufidx, toggle kvflag, 

        } else if (0) { // test for "\n" and kvFlag to see if we are done with val building  if so
          // do a test to affirm that there was a preceeding key and raise exception if not
          // do a switch case here to compare key with expected keys and assign val to the matching key
          // ++kvpIdx
          // if no matching key (case default), raise an exception

        } else if (0) { // test for valid ascii ranges and if good, add c to buff, ++kvBufIdx, and carry on

        } else { // emit debug to advise unexpected 
          Serial.print(c);
        }
        }

      fclose(configFN);

      u8g2.clearBuffer(); // clear the internal memory
      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Config Loaded");
      u8g2.sendBuffer();

      }  else   {

      Serial.println(" => Open Failed\n");

      // create new config file
      // write values (default) to file

      u8g2.clearBuffer(); // clear the internal memory
      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Config is MIA");
      u8g2.sendBuffer();

      strcpy(statusStr, "Using Defaults");
      showStatus();

      }
      */
      delay(_cfgDly);

    }

  void startIMU() {

    clearDisplay();
    u8g2.setFont(_ui_DataFont);

    //Call .begin() to configure the IMU & show status on oled
    if (myIMU.begin() != 0) {

        u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"IMU BORKED");
        u8g2.sendBuffer();

        _gLedOff;
        _oLedOn;
        _rLedOn;
        _wHornOn;

        return; // IMU is required so an overt exiting is appropriate

      } else {

        u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"IMU is Good");
        u8g2.sendBuffer();

        _gLedOn;
        _oLedOff;
        _rLedOff;
        _wHornOff;

      }

    Serial.println("\IMU Started");

    delay(_imuDly);

    }

  void displayPitchRoll() {

    // uses globals pitch, roll

    u8g2.setFont(_ui_DataFont);   // 10w x 20h A=13

    // draw black box vs. clearing the whole display - helps to prevent flickering of Status Window Contents
    u8g2.setDrawColor(_uiColor_BlackOnWhite);
    u8g2.drawBox(_ui_DataWinX,_ui_DataWinY,_ui_DataWinW,_ui_DataWinH);
    u8g2.setDrawColor(_uiColor_WhiteOnBlack);

    // draw labels and populate data passed
    u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"P");
    u8g2.drawStr(_ui_DataWinX+_ui_RollLblTxtOffsetX,_ui_DataWinY+_ui_RollLblTxtOffsetY,"R");

    // draw values
    itoa(pitch,msgStr,10);
    u8g2.drawStr(_ui_DataWinX+_ui_PitchDataTxtOffsetX,_ui_DataWinY+_ui_PitchDataTxtOffsetY,msgStr);

    itoa(roll,msgStr,10);
    u8g2.drawStr(_ui_DataWinX+_ui_RollDataTxtOffsetX,_ui_DataWinY+_ui_RollDataTxtOffsetY,msgStr);

    // draw degree symbol (pos dependent on signs/values of pitch and roll values)
    String temp1 = String(pitch);
    String temp2 = String(roll);
    u8g2.drawUTF8(_ui_DataWinX+_ui_PitchDataTxtOffsetX+((temp1.length()+1)*cWidth)+_ui_DataDegSymOffsetX,_ui_DataWinY+_ui_PitchDataTxtOffsetY+_ui_DataDegSymOffsetY,"°");
    u8g2.drawUTF8(_ui_DataWinX+_ui_RollDataTxtOffsetX+((temp2.length()+1)*cWidth)+_ui_DataDegSymOffsetX,_ui_DataWinY+_ui_RollDataTxtOffsetY+_ui_DataDegSymOffsetY,"°");

    // and push the buffer out to the display
    u8g2.sendBuffer();

    }

  void assertOrientationStates() {

    // uses globals pitch, roll

    // clear flags on each iteration
    // _Warn higher priority than _Caut
    // instantiated separate flags for future annunciation of separate pitch/roll states

    int pCaut = 0;
    int pWarn = 0;
    int rCaut = 0;
    int rWarn = 0;

    // check pitch attitude and set caution/warning flags
    if (pitch >= pTipFwdAngWarn || pitch <= pTipAftAngWarn) {
        pWarn = 1;
      } else if (pitch >= pTipFwdAngCaut || pitch <= pTipAftAngCaut) {
        pCaut = 1;
      } 

    // check roll attitude and set caution/warning flags 
    if (roll >= rTipRtAngWarn || roll <= rTipLtAngWarn) {
        rWarn = 1;
      } else if (roll >= rTipRtAngCaut || roll <= rTipLtAngCaut) {
        rCaut = 1;
      } 

    // set led and horn based on sensed and set warning/caution flags
    if (!pWarn && !pCaut && !rWarn && !rCaut) {  // Good attitude...
      // annunciate non-Warning/non-Caution
      _gLedOn;
      _oLedOff;
      _rLedOff;
      _wHornOff;

      char strg[32] = "";

      sprintf (strg, "Orientation OK %1.0fHz", loopRate);
      //sprintf (strg, "");
      strcpy(statusStr, strg);
      }
    //
    if (pWarn || rWarn ) {
      // annunciate Warning
      if (pWarn && rWarn) {   // pWarn and rWarn
        _gLedOff;
        _oLedOff;
        _rLedOn;
        _wHornOn;
        strcpy(statusStr, "W: Pitch/Roll");
      } else if (pWarn) {     // pWarn, see if rCaut exists
          if (rCaut) {        // yes, pWarn,rCaut
            _gLedOff;
            _oLedOn;
            _rLedOn;
            _wHornOn;
            strcpy(statusStr, "W: Pitch C: Roll");
          } else {            // no, pWarn only
            _gLedOn;
            _oLedOff;
            _rLedOn;
            _wHornOn;
            strcpy(statusStr, "W: Pitch");
            }
      } else if (rWarn) {     // rWarn, see if pCaut exists
          if (pCaut) {        // yes, pCaut,rWarn
            _gLedOff;
            _oLedOn;
            _rLedOn;
            _wHornOn;
            strcpy(statusStr, "W: Roll C: Pitch");
          } else {            // no, rWarn only
            _gLedOn;
            _oLedOff;
            _rLedOn;
            _wHornOn;
            strcpy(statusStr, "W: Roll");
            }
        }
      }
    //
    if (pCaut || rCaut) {
      // annunciate Caution
      if (pCaut && rCaut) {   // pCaut and rCaut
        _gLedOff;
        _oLedOn;
        _rLedOff;
        _wHornOff;
        strcpy(statusStr, "C: Pitch/Roll");
      } else if (pCaut) {     // pCaut, see if rWarn exists
          if (rWarn) {        // yes, rWarn,pCaut
            _gLedOff;
            _oLedOn;
            _rLedOn;
            _wHornOn;
            strcpy(statusStr, "W: Roll C: Pitch");
          } else {            // no, pCaut only
            _gLedOn;
            _oLedOn;
            _rLedOff;
            _wHornOff;
            strcpy(statusStr, "C: Pitch");
            }
      } else if (rCaut) {     // rCaut, see if pWarn exists
          if (pWarn) {        // yes, pWarn,rCaut
            _gLedOff;
            _oLedOn;
            _rLedOn;
            _wHornOn;
            strcpy(statusStr, "W: Pitch C: Roll");
          } else {            // no, rCaut only
            _gLedOn;
            _oLedOn;
            _rLedOff;
            _wHornOff;
            strcpy(statusStr, "C: Roll");
            }
        }
      }
    }

  void printLine() {
    Serial.println("****************************************************");
    }

  void testFileIO(const char * path) {
    Serial.print("Testing file I/O with: ");
    Serial.print(path);

    #define BUFF_SIZE     512

    static uint8_t buf[BUFF_SIZE];

    FILE *file = fopen(path, "w");

    if (file) {
        Serial.println(" => Open OK");
      }  else   {
        Serial.println(" => Open Failed");
        return;
      }

    size_t i;
    Serial.println("- writing" );

    uint32_t start = millis();

    size_t result = 0;

    // Write a file only 1/4 of NANO33BLE_FS_SIZE_KB
    for (i = 0; i < NANO33BLE_FS_SIZE_KB / 2; i++) {
      result = fwrite(buf, BUFF_SIZE, 1, file);

      if ( result != 1) {
        Serial.print("Write result = ");
        Serial.println(result);
        Serial.print("Write error, i = ");
        Serial.println(i);

        break;
        }
      }

    Serial.println("");
    uint32_t end = millis() - start;

    Serial.print(i / 2);
    Serial.print(" Kbytes written in (ms) ");
    Serial.println(end);

    fclose(file);

    printLine();

    /////////////////////////////////

    file = fopen(path, "r");

    start = millis();
    end = start;
    i = 0;

    if (file) {
      start = millis();
      Serial.println("- reading" );

      result = 0;

      fseek(file, 0, SEEK_SET);

      // Read file only 1/4 of NANO33BLE_FS_SIZE_KB
      for (i = 0; i < NANO33BLE_FS_SIZE_KB / 2; i++) {
        result = fread(buf, BUFF_SIZE, 1, file);

        if ( result != 1 ) {
          Serial.print("Read result = ");
          Serial.println(result);
          Serial.print("Read error, i = ");
          Serial.println(i);

          break;
          }
        }

      Serial.println("");
      end = millis() - start;

      Serial.print((i * BUFF_SIZE) / 1024);
      Serial.print(" Kbytes read in (ms) ");
      Serial.println(end);

      fclose(file);
      } else {
        Serial.println("- failed to open file for reading");
      }
    }

  void calIncl() {
    Serial.println("\nCalibrating Inclinometer ...");
    //    clearDisplay();

    delay(3000);   // 10hz rate

    Serial.println("\n... Inclinometer Calibrated");
    }

  void shortBeep() {

    delay(100);
    _wHornOn;
    delay(100);
    _wHornOff;

    }

  void longBeep() {

    delay(750);
    _wHornOn;
    delay(750);
    _wHornOff;

    }

  void doubleBeep() {

    delay(100);
    _wHornOn;
    delay(100);
    _wHornOff;

    delay(100);

    _wHornOn;
    delay(100);
    _wHornOff;

    }

  void errorBeep() {
    longBeep();
    delay(250);
    longBeep();
    delay(250);
    longBeep();
    }

  void tripleBeep() {
    shortBeep();
    doubleBeep();
    }

  void startDisplay() {

    if (!u8g2.begin()) {  // Display Start Failed

      Serial.println("\nDisplay Start Failed - exiting app...");
      return;

    } else {  // Display Start Succeeded

      Serial.println("\nDisplay Started");
      clearDisplay();

      // suppress - splash will be enuf... 
    //      u8g2.setFont(_ui_DataFont);
    //      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Display OK");
    //      u8g2.sendBuffer();

      }

    //    delay(_dispDly);

    }

  void setupIO() {

    // setup IO states, enabling each after being set
    _gLedOff;
    pinMode(_io_GLed , OUTPUT);

    _oLedOff;
    pinMode(_io_OLed , OUTPUT);

    _rLedOff;
    pinMode(_io_RLed , OUTPUT);

    _wHornOff;
    pinMode(_io_WHorn , OUTPUT);

    // open sw needs pullup
    pinMode(_io_UsrSw, INPUT_PULLUP);

    Serial.println("\nIO configured");

    clearDisplay();
    u8g2.setFont(_ui_DataFont);
    u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"IO is Setup");
    u8g2.sendBuffer();

    delay(_ioDly);

    }

  void initComms() {

    // instantiate console comms
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    delay(1000);

    // clear the arduino ide serial console
    //Serial.print(char(27)); Serial.print("[2J");
    //Serial.print("\x1B" "[2J");
    Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"); // really, my eyes just went zenith and gimbal locked there. smdh...

    // emit app vitals to console
    Serial.println("\nSerial Comms Up");
    Serial.println("\n~MHz Roll-o-Meter ...");
    //Serial.print("Xiao Inclinometer with LittleFS running on ");
    Serial.print("Xiao Inclinometer running on ");
    Serial.println(BOARD_NAME);

    clearDisplay();
    u8g2.setFont(_ui_DataFont);
    u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Good Comms");
    u8g2.sendBuffer();

    delay(_commsDly);

    }

  void clearDisplay() {
    u8g2.clearBuffer();					// clear the internal memory
    u8g2.sendBuffer();
    }
  
  void fsVerboseErr(int fsErrNum = 0) {
    // learning/t/s'g aid - take FS error number and
    // - return error string, or
    // - emit verbose error to serial, or                 <-- this for now...
    // - return pointer to verbose error string, or ... 
    itoa(fsErrNum, msgStr, 10);
    Serial.print("\nfsErr ");
    Serial.print(msgStr);
    Serial.print(": ");
    switch (fsErrNum) {
      case LFS_ERR_OK:
        Serial.println("OK - No FS Error");
        break;
      case LFS_ERR_IO:
        Serial.println("LFS_ERR_IO");
        break;
      case LFS_ERR_CORRUPT:
        Serial.println("LFS_ERR_CORRUPT");
        break;
      case LFS_ERR_NOENT:
        Serial.println("LFS_ERR_NOENT");
        break;
      case LFS_ERR_EXIST:
        Serial.println("LFS_ERR_EXIST");
        break;
      case LFS_ERR_NOTDIR:
        Serial.println("LFS_ERR_NOTDIR");
        break;
      case LFS_ERR_ISDIR:
        Serial.println("LFS_ERR_ISDIR");
        break;
      case LFS_ERR_NOTEMPTY:
        Serial.println("LFS_ERR_NOTEMPTY");
        break;
      case LFS_ERR_BADF:
        Serial.println("LFS_ERR_BADF");
        break;
      case LFS_ERR_FBIG:
        Serial.println("LFS_ERR_FBIG");
        break;
      case LFS_ERR_INVAL:
        Serial.println("LFS_ERR_INVAL");
        break;
      case LFS_ERR_NOSPC:
        Serial.println("LFS_ERR_NOSPC");
        break;
      case LFS_ERR_NOMEM:
        Serial.println("LFS_ERR_NOMEM");
        break;
      case LFS_ERR_NOATTR:
        Serial.println("LFS_ERR_NOATTR");
        break;
      case LFS_ERR_NAMETOOLONG:
        Serial.println("LFS_ERR_NAMETOOLONG");
        break;
      default:
        Serial.println("Unknown LFS Error");
        break;     
      }
    }

  void showStatus() {

    if (!heartBeat[hbIdx]) {
      hbIdx = 0;
      }
    msgStr[0] = heartBeat[hbIdx++];
    msgStr[1] = 0;

    // draw black box vs. clearing the whole display - helps to prevent flickering of Data Window Contents
    u8g2.setDrawColor(_uiColor_BlackOnWhite);
    u8g2.drawBox(_ui_StatWinX,_ui_StatWinY,_ui_StatWinW,_ui_StatWinH);
    u8g2.setDrawColor(_uiColor_WhiteOnBlack);

    // draw the heart beat and status text
    u8g2.setFont(_ui_StatFont);
    u8g2.drawStr(_ui_StatWinX+_ui_HBStatTxtOffsetX,_ui_StatWinY+_ui_HBStatTxtOffsetY,msgStr);     // heartbeat
    u8g2.drawStr(_ui_StatWinX+_ui_PromptTxtOffsetX,_ui_StatWinY+_ui_PromptTxtOffsetY,statusStr);  // status texp

    // transfer display memory to display
    u8g2.sendBuffer();
    }

  
  void setSSD1306VcomDeselect(uint8_t v) {
	  // value from 0 to 7, higher values more brighter
    u8x8_cad_StartTransfer(u8g2.getU8x8());
    u8x8_cad_SendCmd(u8g2.getU8x8(), 0x0db);
    u8x8_cad_SendArg(u8g2.getU8x8(), v << 4);
    u8x8_cad_EndTransfer(u8g2.getU8x8());
    }

  void setSSD1306PreChargePeriod(uint8_t p1, uint8_t p2) {	
    // p1: 1..15, higher values, more darker, however almost no difference with 3 or more
    // p2: 1..15, higher values, more brighter
    u8x8_cad_StartTransfer(u8g2.getU8x8());
    u8x8_cad_SendCmd(u8g2.getU8x8(), 0x0d9);
    u8x8_cad_SendArg(u8g2.getU8x8(), (p2 << 4) | p1 );
    u8x8_cad_EndTransfer(u8g2.getU8x8());
    }

  void updateOrientation() {	
    // init vars for angle calculations
    int xAng = 0;
    int yAng = 0;
    int zAng = 0;

    // get current accelerometer values on each axis
    double xAccRaw = myIMU.readFloatAccelX();
    double yAccRaw = myIMU.readFloatAccelY();
    double zAccRaw = myIMU.readFloatAccelZ();

    // apply axial accelerometer scaler and offset for maths
    int xAccNrm = (xAccRaw*_accelNormMulti) + _accelNormOffset;
    int yAccNrm = (yAccRaw*_accelNormMulti) + _accelNormOffset;
    int zAccNrm = (zAccRaw*_accelNormMulti) + _accelNormOffset;

    // convert to range of -90 to +90 degrees, or +90 to -90 degrees if inverted orientation
    if (bInv < 0) {
      xAng = map(xAccNrm, _accNormMin, _accNormMax, 90, -90);
      yAng = map(yAccNrm, _accNormMin, _accNormMax, 90, -90);
      zAng = map(zAccNrm, _accNormMin, _accNormMax, 90, -90);
      } else {
      xAng = map(xAccNrm, _accNormMin, _accNormMax, -90, 90);
      yAng = map(yAccNrm, _accNormMin, _accNormMax, -90, 90);
      zAng = map(zAccNrm, _accNormMin, _accNormMax, -90, 90);
      }

    // convert radians to degrees, inverting if so needed
    pitch = (RAD_TO_DEG * (atan2(-xAng, -zAng) + PI)) * pDir;
    roll = (RAD_TO_DEG * (atan2(-yAng, -zAng) + PI)) * rDir;

    // correct per orientation flags
    if (pDir < 0) {
      pitch = pitch + 360;
      }

    if (rDir < 0) {
      roll = roll + 360;
      }

    // convert left roll and forward pitch to negative degrees
    if (pitch > 180) {
      pitch = pitch - 360;
      }

    if (roll > 180) {
      roll = roll - 360;
      }

    }

  void pollUI() {

    if (_switchPressed) {

      shortBeep();

      Serial.println("\nCalibration Mode Triggered");
      strcpy(statusStr, "Cal Triggered");
      showStatus();

      int pressTime = millis();

      while (_switchPressed) {
        //dwell until released, counting to see how long held and, if held more than 5 seconds, reset the inclinometer

        if ((millis() - pressTime) > _switchHeldResetPeriod) {
          resetInclinometer();
          }

        }
      createConfig();
      doubleBeep();

      //calIncl();
      }

    }

  // ########################################################################################

// XIAO INCLINOMETER APP ####################################################################
  void setup() {

    timeStart = millis(); // catch init start time - ~49 day rollover

    startDisplay();
    showSplash();
    initComms();
    setupIO();
    mountFS();
    loadConfig();
    startIMU();

    timeEnd = millis() - timeStart; // catch init end time

    Serial.print("\nInit complete in (ms) ");
    Serial.println(timeEnd);
    Serial.println("\n... Inclinometer Startup Complete - Entering Main Loop");

    clearDisplay();
    u8g2.setFont(_ui_DataFont);
    u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Starting ~MHz");
    u8g2.sendBuffer();

    strcpy(statusStr, "Roll-o-Meter :)");
    showStatus();

    tripleBeep();
    delay(_initDly);

    clearDisplay();

    timeStart = millis();

    }

  void loop() {

    loopCnt++;
    loopRate = (loopCnt*100000)/((millis() - timeStart));
    loopRate /= 100;

    updateOrientation();
    displayPitchRoll();
    assertOrientationStates();
    showStatus();
    pollUI();

    // delay(160);   // ~5hz rate

    }

  // ########################################################################################


 





























