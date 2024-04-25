// XIAO nRF52840 BLE SENSE INCLINOMETER PREAMBLE ############################################
  /*****************************************************************************/
  //  XiaoSenseInclinometer.ino
    //
    //  Repository:     https://github.com/TodWulff/-MHz-Roll-o-Meter
    //
    //  Hardware:       Seeed Studio Xiao nRF52840 BLE Sense SBC
    //                  128*32 OLED Display with SSD1306 driver
    //                  Active buzzer with active low digital input to trigger 
    //                  3ea Discrete LEDs with current 220ohm limiting resistors
    //                  1ea Discrete PB switch for user interaction
    //
    //	Arduino IDE:    Arduino-2.3.x
    //	Author:         Tod Wulff (a.k.a. MegaHurtz/~MHz)
    //	Date:           23 April 2024
    //	Version:        v1.0
    //
    //  Description:   See video:  https://youtu.be/2NaQaHJ9XEI
    //
    //  Inspiration/References:
    //  Scale Built RC's Roll-o-Meter - Thanks brother!
    //    - https://www.youtube.com/watch?v=PfeZpKERkO8
    //  https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
    //  https://github.com/camerontech/inclinometer
    //  https://forum.seeedstudio.com/t/xiao-sense-accelerometer-examples-and-low-power/270801
    //  https://forum.seeedstudio.com/t/seeed-xiao-ble-nrf52840-sense-giving-bad-gyroscope-data/274134/11
    //  https://adam-meyer.com/arduino/sensing-orientation-with-the-adxl335-arduino  <-- this
    //  + a bunch of others for the oled display (using u8g2 lib)
    //
    //  by www.seeedstudio.com
    //
    //  This is free software; you can redistribute it and/or
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

  // Possible enhancements to consider / todos (as of 23APR24): 
    // - Add cute startup splash screen w/ static, or moving, graphics
    // - 'Flashing' LEDs in caution and/or warning contexts - currently 'works' so might not fix it
    // - Pulsing Aural Warnings (vs. static tone) in caution and/or warning contexts - meh, maybe...
    // - Graphics depicting truck and angle indicators (see https://www.youtube.com/watch?v=PfeZpKERkO8)
    // - Flashing Display graphics or tossing a warning display on the oled
    //   (see https://www.facebook.com/watch/?v=2394412913939607)
    // - High resolution color UI on appropriate glass 
    //   (see https://www.facebook.com/scalebuilt/videos/605197420078915)
    // - implement pwm control of leds and aural warning device
    // - adapt multiple LEDs to one or more neopixel-ish APA106 (5mm and 8mm are on order)
    // - to support multiple OLED geometries, make dynamic by query of u8g2 for display dims
    // - add lateral orientations support
    // - consider supporting truly dynamic installation orientations (may need to make use of 
    //   rotation matricies || quaternions to prevent gimbal lock)
    // - layer on some UI frosting - visual flashing/iconage, aural toneage, etc.
    // - evaluate expanding sensed accelerometer range, as sensor is 8G/16G, I perceive
    //   but it works as is, so may choose to not muck with what isn't broken...
    // - evaluate timer interrupts to implement a software WDT that calls an ISR to start WDT with 
    //   a CRV of 1 to implement a reset in ~31uS - that ISR is the existing resetInclinometer();
    //   function which is indeed using the WDT w/ CRV of 1.  But, user initiated reset is working, so...
    // - revisit the possible use of a file system for persistent config storage - low roi at this time
  
  // 22APR24: No longer considering a calibration effrort as I perceive that the roi is simply not worth
    //  all that would be needed to implement same.  Nix'd all config and LittleFS content.  May revisit
    //  doing so in the future - tbd...  For now, just hard-coded via #defined init of related variables.
  
  // Completed (as of 23APR24):
    // - make supported orientation assertions more intuitive - added comment blocks to aid in this
    // - consider employing preprocessor #defs in favor of const - did so, with appropriate comments
    // - add switch input to enable user reset and possibly intuitive calibration once implemented
    // - add rotating char to corner to depict loop running cycle through: - \ | / 
    // - figure out how to do a SW reset on the nRF52840 so Inclinometer can be forcibly reset
    //   after a configuration event <-- made use of the WDT with a CRV of 0x01 = ~31uS
    // - mute aural warning after _ seconds
    // - consider having single short button press when running to 'cage' orientation (i.e.
    //   applying axial offsets, if practical, given an accelerometer-only employment) 
    //   Actually implemented via WDT reset
    // - nRF52840 WDT implementation is pretty underwhelming - likely by conservative design.
    //   Once started, can't pause it, can't reconfigure until reset, pretty much locked down
    //   once started.  As such only using it for software reset purposes in this app.
  
  // 23Apr24 Sketch Stats on Xiao nRF52840 Sense Target (Arduino Core (not MBed core...)): 
    //  Sketch uses 84088 bytes (10%) of program storage space. Maximum is 811008 bytes.
    //  Global variables use 9248 bytes (3%) of dynamic memory, leaving 228320 bytes for local variables. 
    //  Maximum is 237568 bytes.

  // ########################################################################################

// XIAO nRF52840 BLE SENSE INCLINOMETER PREPROCESSOR DIRECTIVES #############################
  // debug #defs

    //#define _debug  // uncomment this line to enable serial debug
    //#define _debug_smooth  // uncomment this line to enable serial debug in the smoothAttitude() proc

    #define _BOARD_NAME                  "Seeed Xiao nRF52840 BLE Sense"

  // digital io assignments for LEDs and Aural Warning Device

    // if mbed core is being used - larger fw core, a bit slower, and has altered IO port ids

      // #define _io_GLed             11
      // #define _io_OLed             10
      // #define _io_RLed             9
      // #define _io_WHorn            8
      // #define _io_UsrSw            7
      
    // if arduino core being used - smaller core and faster

      #define _io_GLed                10
      #define _io_OLed                9
      #define _io_RLed                8
      #define _io_WHorn               7
      #define _io_UsrSw               6
    
  // digital state assignments

    #define _do_led_On                0
    #define _do_led_Off               1
    #define _do_wHorn_On              0
    #define _do_wHorn_Off             1

  // for readability

    #define _gLedOff                  digitalWrite(_io_GLed,_do_led_Off)
    #define _gLedOn                   digitalWrite(_io_GLed,_do_led_On)

    #define _oLedOff                  digitalWrite(_io_OLed,_do_led_Off)
    #define _oLedOn                   digitalWrite(_io_OLed,_do_led_On)

    #define _rLedOff                  digitalWrite(_io_RLed,_do_led_Off)
    #define _rLedOn                   digitalWrite(_io_RLed,_do_led_On)

    #define _wHornOff                 digitalWrite(_io_WHorn,_do_wHorn_Off)
    #define _wHornOn                  digitalWrite(_io_WHorn,_do_wHorn_On)

    #define _switchReleased           (digitalRead(_io_UsrSw))
    #define _switchPressed            (!_switchReleased)

  // normalizing accelerometer values per guidance: 

    // https://adam-meyer.com/arduino/sensing-orientation-with-the-adxl335-arduino
    // given raw is -1.x|1.x, 'normalized' to 0|2047  
    // normalized = (raw * multiplier) + offset
    #define _accelNormOffset          1024
    #define _accelNormMulti           1023

  // max/min 'normalized' values - from -1.0|1.0 map'd to 0|2047 - a -1G-1G range

    #define  _accNormMin              0
    #define  _accNormMax              (_accelNormOffset + _accelNormMulti)

  // config affirmation delays

    #define _splashDly                1000
    #define _imuDly                   0
    #define _lfsDly                   0
    #define _cfgDly                   0
    #define _initDly                  0
    #define _dispDly                  0
    #define _ioDly                    0
    #define _commsDly                 0

  // _ui_Data Window Display Elements #######################################################

   // colors for labels/data/symbols/etc

    #define _uiColor_BlkOnWhi         0
    #define _uiColor_WhtOnBlk         1

   // font for labels/data/symbols/etc

    #define _ui_DataFont              u8g2_font_10x20_te

   // top half is for data display

    #define _ui_DataWinX              0
    #define _ui_DataWinY              0
    #define _ui_DataWinW              128
    #define _ui_DataWinH              16

   // labels

    #define _ui_PitchLblTxtOffsetX    0
    #define _ui_PitchLblTxtOffsetY    14

    #define _ui_RollLblTxtOffsetX     65
    #define _ui_RollLblTxtOffsetY     14

   // pitch text offsets in data win

    #define _ui_PitchDataTxtOffsetX   13
    #define _ui_PitchDataTxtOffsetY   14

   // roll text offsets in data win

    #define _ui_RollDataTxtOffsetX    78
    #define _ui_RollDataTxtOffsetY    14

   // degree symbol offsets in data win

    #define _ui_DataDegSymOffsetX     -7
    #define _ui_DataDegSymOffsetY     -1

   // font for status/prompts/etc.

    #define _ui_StatFont              u8g2_font_6x12_te

   // bottom half is for icons/status/prompts/etc

    #define _ui_StatWinX              0
    #define _ui_StatWinY              17
    #define _ui_StatWinW              128
    #define _ui_StatWinH              16

   // heartbeat text offsets in stat win

    #define _ui_HBStatTxtOffsetX      0
    #define _ui_HBStatTxtOffsetY      12

   // status/prompt text offsets in stat win

    #define _ui_PromptTxtOffsetX      9
    #define _ui_PromptTxtOffsetY      12

   // period in mS to force a device reset if the ui switch is continuously held

    #define _swResetPeriod            2000

   // avg attitude over last _attAvgIterations+1 (0-based) sensed values

    #define _attAvgIterations         9 
    // given loop rate of 20-30Hz (w/ no loop delay), values of ~9 are appropriate - >9 impute a latency given loop rate
    // values <9 impute a context where noisy accelerometer values are likely/possible
    // It's appropriate to target <= ~1/2 of the frequency of the main loop - i.e. 4+1 iterations at a 10hz rate works.
    // That way attitude calcs remain responsive (< 0.5S latent) and benefits from averaging/smoothing...

   // for use to cast char array variables - 64 is very likely excessive...

    #define _tmpStringLen             64

    // ######################################################################################

  // default attitude caution/warning parameter setup #######################################

   // warning horn firing period - low values here are useful to keep warning horn from becoming an irritant vs useful
    // fyi, loop rates will impace lowest practical value - higher rate = lower setting having a material effect

    #define _wHornActivePeriod        10

   // caution and warning safety buffers: set these values based on user preference

    #define _att_CautBuff             3       // degrees - buffer before warning buffer - 3 is min recommended - 5 is less risk
    #define _att_WarnBuff             3       // degrees - buffer before tipover/rollover - 3 is min recommended - 5 is less risk

   // tip point setup: set after the sensor is installed in a fully configured vehicle and tested for pitch and roll tip points

    // these are pretty well tuned for the 7.5# ~MHz Capra with 60/40 forward weight bias on 2.2 wheels/5.12 tires with 
    // 2-stage foam inserts, with gravity-only compressed suspension (no winch tension) 
    // was done on carpet and hardwood flooring

    #define _att_PosPitTip            79      // degrees - set to last whole degree where a nose up tipover doesn't happen
    #define _att_NegPitTip            -74     // degrees - set to last whole degree where a nose down tipover doesn't happen

    // suspect following delta is due to lateral cg offset of motor/electronics in cabin, considering suspecsion droop
    #define _att_PosRollTip           68      // degrees - set to last whole degree where a leaning-right rollover doesn't happen
    #define _att_NegRollTip           -68     // degrees - set to last whole degree where a leaning-left rollover doesn't happen

   // These default pitch/roll caution/warning points should not need to be altered as they are derived from above
    // nose up pitch
      #define _att_DefPosPitCaut      (_att_PosPitTip - _att_WarnBuff - _att_CautBuff)
      #define _att_DefPosPitWarn      (_att_PosPitTip - _att_WarnBuff)

    // nose down pitch
      #define _att_DefNegPitCaut      (_att_NegPitTip + _att_WarnBuff + _att_CautBuff)
      #define _att_DefNegPitWarn      (_att_NegPitTip + _att_WarnBuff)

    // right roll (adjusted by -5 as part of fine tuning & post-install testing)
      #define _att_DefPosRollCaut     (_att_PosRollTip - _att_WarnBuff - _att_CautBuff)
      #define _att_DefPosRollWarn     (_att_PosRollTip - _att_WarnBuff)

    // left roll 
      #define _att_DefNegRollCaut     (_att_NegRollTip + _att_WarnBuff + _att_CautBuff)
      #define _att_DefNegRollWarn     (_att_NegRollTip + _att_WarnBuff)

   // the following modifies limits based on sensed attitude - a wip...

    // This vehicle seems to have lower roll over thresholds with pitch attitude deviations from level
    // As such, try to decrease roll limits by a value of 1/10th of absolute pitch attitude
    // this is a bit of a test as part of fine tuning post-install testing

    #define _p2r_CautMod              ((abs(pitch)/10) * ((roll >= 0) ? -1 : 1))   //ternary ftw
    #define _p2r_WarnMod              _p2r_CautMod

    // modify pitch limits based on sensed roll attitude

    // may not be needed, hence 0, but while my gray matter is engaged, coded for same, 
    // to ease future implementation of same

    #define _r2p_CautMod              0   // ((abs(roll)/10) * ((pitch >= 0) ? -1 : 1))   //ternary ftw
    #define _r2p_WarnMod              _r2p_CautMod

   // ######################################################################################

  // #Includes ##############################################################################

    #include "LSM6DS3.h"    // for IMU
    #include <U8g2lib.h>    // for B&W OLED Display

   // ########################################################################################

// XIAO nRF52840 BLE SENSE INCLINOMETER VARIABLE INSTANTIATION ##############################

  // Create a instance of class LSM6DS3 (the 6Dof IMU)
  LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

  // assert constructor for given OLED display
  U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); // this is the 0.91" 128x32 small OLED display w/ ssd1306 driver IC

  // required by u8g2 lib - needs ram-based vars for use
  char tmpStr[_tmpStringLen];
  char msgStr[_tmpStringLen];
  char statusStr[_tmpStringLen];

  int pitchArray[_attAvgIterations];
  int rollArray[_attAvgIterations];

  int cWidth = 9;         // todo: set via query of u8g2 lib for specified font during setup

  uint64_t timeStart = 0;
  uint64_t timeNow = 0;
  uint64_t timeEnd = 0;
  uint64_t loopCnt = 0;
  double loopRate = 0;

  // wHorn_Active_Period
  int wHornStart = 0;
  int wHornStartPitch = 0;
  int wHornStartRoll = 0;
 
  char heartBeat[] = "-\\|/";
  //char heartBeat[] = "-\\|~MHz|/";
  //char heartBeat[] = "-\\|/ ~MHz -\\|/ Roll -\\|/ o -\\|/ Meter";
  int hbIdx = 0;

  int pitch = 0;
  int roll = 0;

  int lastPitch = 0;
  int lastRoll = 0;

  int pitchAccum = 0;
  int rollAccum = 0;

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

  // defaults are instantiated here - to be possibly modified by config file if/when implemented
  int pTipFwdAngCaut = _att_DefPosPitCaut;
  int pTipFwdAngWarn = _att_DefPosPitWarn;

  int pTipAftAngCaut = _att_DefNegPitCaut;
  int pTipAftAngWarn = _att_DefNegPitWarn;

  int rTipRtAngCaut = _att_DefPosRollCaut;
  int rTipRtAngWarn = _att_DefPosRollWarn;

  int rTipLtAngCaut = _att_DefNegRollCaut;
  int rTipLtAngWarn = _att_DefNegRollWarn;

  // ########################################################################################

// XIAO nRF52840 BLE SENSE INCLINOMETER APP PROCS ###########################################

  #ifdef _debug                     // if debug asserted
    void initComms() {              // init serial comms

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
      Serial.println(_BOARD_NAME);

      clearDisplay();
      u8g2.setFont(_ui_DataFont);
      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Good Comms");
      u8g2.sendBuffer();

      delay(_commsDly);

    #endif

  void startDisplay() {             // init display via library proc

    if (!u8g2.begin()) {  // Display Start Failed

      #ifdef _debug
        Serial.println("\nDisplay Start Failed - exiting app...");
        #endif
      return;

    } else {  // Display Start Succeeded

      #ifdef _debug
        Serial.println("\nDisplay Started");
        #endif
      clearDisplay();

      // suppress - splash will be enuf... 
      //      u8g2.setFont(_ui_DataFont);
      //      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"Display OK");
      //      u8g2.sendBuffer();

      }

    //    delay(_dispDly);

    }

  void setupIO() {                  // init the GPIO and related hardware connected thereto

    // setup IO states, enabling each after being set
    _gLedOff;
    pinMode(_io_GLed , OUTPUT);

    _oLedOff;
    pinMode(_io_OLed , OUTPUT);

    _rLedOff;
    pinMode(_io_RLed , OUTPUT);

    deactivateWarnHorn();
    pinMode(_io_WHorn , OUTPUT);

    // open sw needs pullup
    pinMode(_io_UsrSw, INPUT_PULLUP);

    #ifdef _debug
      Serial.println("\nIO configured");
      #endif

    clearDisplay();
    u8g2.setFont(_ui_DataFont);
    u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"IO is Setup");
    u8g2.sendBuffer();

    delay(_ioDly);

    }

  void startIMU() {                 // start the imu sensor

    clearDisplay();
    u8g2.setFont(_ui_DataFont);

    //Call .begin() to configure the IMU & show status on oled
    if (myIMU.begin() != 0) {

      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"IMU BORKED");
      u8g2.sendBuffer();

      _gLedOff;
      _oLedOn;
      _rLedOn;
      activateWarnHorn();

      return; // IMU is required so an overt exiting is appropriate

    } else {

      u8g2.drawStr(_ui_DataWinX+_ui_PitchLblTxtOffsetX,_ui_DataWinY+_ui_PitchLblTxtOffsetY,"IMU is Good");
      u8g2.sendBuffer();

      _gLedOn;
      _oLedOff;
      _rLedOff;
      deactivateWarnHorn();

      }

    #ifdef _debug
      Serial.println("\nIMU Started");
      #endif

    delay(_imuDly);

    }

  void resetInclinometer() {        // use WDT to implement a SWR

    //Configure & start WDT with shortest period, to force an 'immediate' reset via SW
    #ifdef _debug
      Serial.println("\nResetting Processor via WDT");
     #endif
    NRF_WDT->CONFIG         = 0x01;     // Configure WDT to run when CPU is asleep
    NRF_WDT->CRV            = 1;        // load CRV - with 1 - should reset at Start + ~30uS (32.768KHz based)
    NRF_WDT->RREN           = 0x01;     // Enable the RR[0] reload register
    NRF_WDT->TASKS_START    = 1;        // Triggers the Start WDT - not stoppable hereafter , until post-reset...

    }

  void showSplash() {               // for ui goodness

    clearDisplay();
    u8g2.setFont(_ui_DataFont);
    u8g2.drawStr(0,15,"   ~MHz");
    u8g2.drawStr(0,30,"Roll-o-Meter");
    u8g2.sendBuffer();

    delay(_splashDly);

    }

  void shortBeep() {                // aural ui element

    delay(100);
    activateWarnHorn();
    delay(100);
    deactivateWarnHorn();

    }

  void longBeep() {                 // aural ui element

    delay(750);
    activateWarnHorn();
    delay(750);
    deactivateWarnHorn();

    }

  void doubleBeep() {               // aural ui element

    delay(100);
    activateWarnHorn();
    delay(100);
    deactivateWarnHorn();

    delay(100);

    activateWarnHorn();
    delay(100);
    deactivateWarnHorn();

    }

  void errorBeep() {                // aural ui element

    longBeep();
    delay(250);
    longBeep();
    delay(250);
    longBeep();

    }

  void tripleBeep() {               // aural ui element

    shortBeep();
    doubleBeep();

    }

  void clearDisplay() {             // helper proc to simply clear the glass

    u8g2.clearBuffer();					// clear the internal memory
    u8g2.sendBuffer();

    }
  
  void pollUI() {                   // poll hmi to see if user input needs processing

    if (_switchPressed) {

      _gLedOn;
      shortBeep();

      #ifdef _debug
        Serial.println("\nSwitch Triggered");
       #endif

      strcpy(statusStr, "Hold for Reset ...");
      showStatus();

      int pressTime = millis();

      while (_switchPressed) {
        //dwell until released, counting to see how long held and, if held more than 5 seconds, reset the inclinometer

        if ((millis() - pressTime) > _swResetPeriod) {
          strcpy(statusStr, "... Now Release");
          showStatus();
          while (_switchPressed) {
            // wait until released
            }
          resetInclinometer();
          }

        }
      //createConfig();
      doubleBeep();
      _gLedOff;

      }

    }

  void displayPitchRoll() {         // displays sensed attitude on data portion of display

    // uses globals pitch, roll

    u8g2.setFont(_ui_DataFont);   // 10w x 20h A=13

    // draw black box vs. clearing the whole display - helps to prevent flickering of Status Window Contents
    u8g2.setDrawColor(_uiColor_BlkOnWhi);
    u8g2.drawBox(_ui_DataWinX,_ui_DataWinY,_ui_DataWinW,_ui_DataWinH);
    u8g2.setDrawColor(_uiColor_WhtOnBlk);

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

  void showStatus() {               // displays hearbeat and status on status portion of display

    if (!heartBeat[hbIdx]) {  // check hbIdx to see if we're at the end of heartbeat string
      hbIdx = 0;              // and reset to beginning of string if so
      }

    msgStr[0] = heartBeat[hbIdx++];
    msgStr[1] = 0;

    // draw black box vs. clearing the whole display - helps to prevent flickering of Data Window Contents
    u8g2.setDrawColor(_uiColor_BlkOnWhi);
    u8g2.drawBox(_ui_StatWinX,_ui_StatWinY,_ui_StatWinW,_ui_StatWinH);
    u8g2.setDrawColor(_uiColor_WhtOnBlk);

    // draw the heart beat and status text
    u8g2.setFont(_ui_StatFont);
    u8g2.drawStr(_ui_StatWinX+_ui_HBStatTxtOffsetX,_ui_StatWinY+_ui_HBStatTxtOffsetY,msgStr);     // heartbeat
    u8g2.drawStr(_ui_StatWinX+_ui_PromptTxtOffsetX,_ui_StatWinY+_ui_PromptTxtOffsetY,statusStr);  // status texp

    // transfer display memory to display
    u8g2.sendBuffer();
    
    }
    
  void updateOrientation() {	      // sense accelerations and do math to derive pitch/roll attitudes

    #ifdef _debug
      Serial.print("\nupdateOrientation(): ");
      #endif

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
    lastPitch = (RAD_TO_DEG * (atan2(-xAng, -zAng) + PI)) * pDir;
    lastRoll = (RAD_TO_DEG * (atan2(-yAng, -zAng) + PI)) * rDir;

    // correct per orientation flags
    if (pDir < 0) {
      lastPitch = lastPitch + 360;
      }

    if (rDir < 0) {
      lastRoll = lastRoll + 360;
      }

    // convert left lastRoll and forward lastPitch to negative degrees
    if (lastPitch > 180) {
      lastPitch = lastPitch - 360;
      }

    if (lastRoll > 180) {
      lastRoll = lastRoll - 360;
      }

    #ifdef _debug
      sprintf(tmpStr,"lP: %d   lR: %d", lastPitch, lastRoll);
      Serial.println(tmpStr);
      #endif

    smoothAttitude();

    }

  void assertOrientationStates() {  // test sensed pitch and roll and set flags

    // uses globals pitch, roll

    // clear flags on each iteration
    // _Warn higher priority than _Caut
    // instantiated separate flags for future annunciation of separate pitch/roll states

    int pCaut = 0;
    int pWarn = 0;
    int rCaut = 0;
    int rWarn = 0;

    // check pitch attitude and set caution/warning flags
    // adapted with cross-axial modifier
    if (pitch >= pTipFwdAngWarn + _r2p_WarnMod || pitch <= pTipAftAngWarn + _r2p_WarnMod) {
        pWarn = 1;
      } else if (pitch >= pTipFwdAngCaut + _r2p_CautMod || pitch <= pTipAftAngCaut + _r2p_CautMod) {
        pCaut = 1;
      } 

    // check roll attitude and set caution/warning flags
    // adapted with cross-axial modifier
    if (roll >= rTipRtAngWarn + _p2r_WarnMod || roll <= rTipLtAngWarn + _p2r_WarnMod) {
        rWarn = 1;
      } else if (roll >= rTipRtAngCaut + _p2r_CautMod || roll <= rTipLtAngCaut + _p2r_CautMod ) {
        rCaut = 1;
      } 

    // set led and horn based on sensed and set warning/caution flags
    if (!pWarn && !pCaut && !rWarn && !rCaut) {  // Good attitude...
      // annunciate non-Warning/non-Caution
      //_gLedOn;
      _gLedOff;
      _oLedOff;
      _rLedOff;
      deactivateWarnHorn();

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
        activateWarnHorn();
        strcpy(statusStr, "W: Pitch/Roll");
      } else if (pWarn) {     // pWarn, see if rCaut exists
          if (rCaut) {        // yes, pWarn,rCaut
            _gLedOff;
            _oLedOn;
            _rLedOn;
            activateWarnHorn();
            strcpy(statusStr, "W: Pitch C: Roll");
          } else {            // no, pWarn only
            _gLedOff;
            _oLedOff;
            _rLedOn;
            activateWarnHorn();
            strcpy(statusStr, "W: Pitch");
            }
      } else if (rWarn) {     // rWarn, see if pCaut exists
          if (pCaut) {        // yes, pCaut,rWarn
            _gLedOff;
            _oLedOn;
            _rLedOn;
            activateWarnHorn();
            strcpy(statusStr, "W: Roll C: Pitch");
          } else {            // no, rWarn only
            _gLedOff;
            _oLedOff;
            _rLedOn;
            activateWarnHorn();
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
        deactivateWarnHorn();
        strcpy(statusStr, "C: Pitch/Roll");
      } else if (pCaut) {     // pCaut, see if rWarn exists
          if (rWarn) {        // yes, rWarn,pCaut
            _gLedOff;
            _oLedOn;
            _rLedOn;
            activateWarnHorn();
            strcpy(statusStr, "W: Roll C: Pitch");
          } else {            // no, pCaut only
            _gLedOff;
            _oLedOn;
            _rLedOff;
            deactivateWarnHorn();
            strcpy(statusStr, "C: Pitch");
            }
      } else if (rCaut) {     // rCaut, see if pWarn exists
          if (pWarn) {        // yes, pWarn,rCaut
            _gLedOff;
            _oLedOn;
            _rLedOn;
            activateWarnHorn();
            strcpy(statusStr, "W: Pitch C: Roll");
          } else {            // no, rCaut only
            _gLedOff;
            _oLedOn;
            _rLedOff;
            deactivateWarnHorn();
            strcpy(statusStr, "C: Roll");
            }
        }
      }

    }

  void activateWarnHorn() {         // serves to also squelch the horn when timeout attained, if no attitude changes

    if (wHornStart) {                                         // if we already started the horn, then
      if ((millis() - wHornStart) >= _wHornActivePeriod) {    // test to see if we are timed out on the horn
        if ((wHornStartPitch == pitch) and (wHornStartRoll == roll)) {  // test to see if attitude hasn't changed
          _wHornOff;                                            // if attitude is static, cease blaring
         } else {                                             // so, attitude did change since it was when the horn started, so need to record new attitude and restart the timeout testing
            wHornStartPitch = pitch;                            // record new pitch attitude for future conditionals
            wHornStartRoll = roll;                              // record new roll attitude for future conditionals
            _wHornOn;                                           // reassert warning
            wHornStart = millis();                              // reset horn timeout start
          }
       } else {                                               // if we're not timed out on the horn, but it was started, then 
                      // do nothing
        }
     } else {                                                  // so this is a new start of the horn, so 
      wHornStart = millis();                                    // record the start time
      wHornStartPitch = pitch;                                  // record the entry pitch angle
      wHornStartRoll = roll;                                    // record the entry roll angle 
      _wHornOn;                                                 // and sing loudly
      }

    }

  void deactivateWarnHorn() {       // mute the horn and null related variables

      wHornStart = 0;         // null the timer
      wHornStartPitch = 0;    // null attitude                                         
      wHornStartRoll = 0;     // null attitude   
      _wHornOff;              // gag the noise maker

    }

  void smoothAttitude() {           // smooth by averaging over last _attAvgIterations

    // zero out the accumulators on each run instance
    pitchAccum = 0;
    rollAccum = 0;

    #ifdef _debug_smooth
      Serial.println("\nSmoothing Attitude:");
      #endif

    // reindex historized values to bump down the stacks
    for (int i = _attAvgIterations; i > 0; i--) {

      // reindex each val to next position
      pitchArray[i]=pitchArray[i-1];
      rollArray[i]=rollArray[i-1];

      // add values to accumulators for KISS averaging purposes
      pitchAccum += pitchArray[i];
      rollAccum += rollArray[i];
      
      #ifdef _debug_smooth
        // display reindexed values
        sprintf(tmpStr,"P(%d): %d [%d]  R(%d): %d [%d]", i, pitchArray[i], pitchAccum, i, rollArray[i], rollAccum);
        Serial.println(tmpStr);
        #endif
      
      }

    // push latest pitch and roll values onto top of stacks
    pitchArray[0]=lastPitch;
    rollArray[0]=lastRoll;

    // add latest values to accumulators
    pitchAccum += pitchArray[0];
    rollAccum += rollArray[0];

    // then average the accumulator values (integer math is fine...), 
    // assigning results to the global attitude variables
    pitch = pitchAccum/(_attAvgIterations+1);
    roll = rollAccum/(_attAvgIterations+1);

    #ifdef _debug_smooth
      // print the latest values to include [accumulator intervals]
      sprintf(tmpStr,"P(%d): %d [%d]  R(%d): %d [%d]", 0, pitchArray[0], pitchAccum, 0, rollArray[0], rollAccum);
      Serial.println(tmpStr);

      Serial.println("-----------------------------------------------");

      // display the accumulated values which were used to derive averages
      sprintf(tmpStr,"pA: %d   rA: %d", pitchAccum, rollAccum);
      Serial.println(tmpStr);

      // and display the averaged pitch and roll 
      sprintf(tmpStr,"\nSP: %d   SR: %d", pitch, roll);
      Serial.println(tmpStr);

      Serial.println("===============================================\n");
      #endif

    }

  // ########################################################################################

// XIAO nRF52840 BLE SENSE INCLINOMETER APP #################################################
  void setup() {

    #ifdef _debug
      timeStart = millis(); // catch init start time - ~49 day rollover
     #endif

    startDisplay();

    #ifdef _debug
      initComms();
     #endif

    setupIO();
    _gLedOn;
    startIMU();
    showSplash();

    #ifdef _debug
      timeEnd = millis() - timeStart; // catch init end time
      Serial.print("\nInit complete in (ms) ");
      sprintf(tmpStr,"%d", timeEnd);
      Serial.println(tmpStr);
      Serial.println("\n... Inclinometer Startup Complete - Entering Main Loop");
     #endif

    // Initialize attitude smoothing array
    for (int i = 0; i < _attAvgIterations; i++) { pitchArray[i]=0; rollArray[i]=0; }

    tripleBeep();

    //delay(_initDly);

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

    //delay(63);   // imputes a ~10hz rate  (non-delayed loop running at ~27hz/37mS)

    }

  // ########################################################################################
