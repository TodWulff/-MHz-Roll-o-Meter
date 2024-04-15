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
//  Possible enhancements to consider:
//    - Add cute startup splash screen
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

#include "LSM6DS3.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); // this is the 0.91" 128x32 small oled display
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // this is the larger more square-ish 128x64 oled display

// max/min 'normalized' values - from -1.0|1.0 to 0|2047 - a -1G-1G range - maybe expand, as sensor is 8G/16G, I perceive
// but it works as is, so will likely not muck with what isn't broken...
const int MIN = 0;      // -1g
const int MAX = 2047;   // 1g

char msgString[20];     // for use w/ u8g2 lib var string

const int labelX = 1;
const int labelY = 26;

const int pitchX = 15;
const int pitchY = 26;

const int rollX = 75;
const int rollY = 26;

const int degSymOffset = -7;

// installed orientation correction flags
const int bInv = -1;    // board inversion flag - 0 for non-inverted or -1 to invert
const int pDir = 1;     // pitch inversion flag - 1 for non-inverted or -1 to invert
const int rDir = -1;    // roll inversion flag - 1 for non-inverted or -1 to invert

const int gLed = 10;
const int oLed = 9;
const int rLed = 8;

const int wHorn = 7;

const int led_On = 0;
const int led_Off = 1;
const int wHorn_On = 0;
const int wHorn_Off = 1;

int cWidth = 9;

// set these after the sensor is installed in the vehicle and tested for pitch and roll stability
const int pTipFwdAngCaut = 30;
const int pTipFwdAngWarn = 45;

const int pTipAftAngCaut = -30;
const int pTipAftAngWarn = -45;

const int rTipRtAngCaut = 30;
const int rTipRtAngWarn = 45;

const int rTipLtAngCaut = -30;
const int rTipLtAngWarn = -45;

//setup app
void setup(void) {

    // start oled driver
    u8g2.begin();
    showSplash(); // has own delay

    // setup IO states, enabling each after being set
    digitalWrite(gLed,led_Off);
    pinMode(gLed , OUTPUT);

    digitalWrite(oLed,led_Off);
    pinMode(oLed , OUTPUT);

    digitalWrite(rLed,led_Off);
    pinMode(rLed , OUTPUT);

    digitalWrite(wHorn,wHorn_Off);
    pinMode(wHorn , OUTPUT);

    startIMU();

}

void loop() {

    // init vars for angle calculations
    int xAng = 0;
    int yAng = 0;
    int zAng = 0;

    // get current accelerometer values on each axis
    double xAccRaw = myIMU.readFloatAccelX();
    double yAccRaw = myIMU.readFloatAccelY();
    double zAccRaw = myIMU.readFloatAccelZ();

    // normalizing accelerometer values per guidance: 
    // https://adam-meyer.com/arduino/sensing-orientation-with-the-adxl335-arduino
    // given raw is -1.x|1.x, 'normalized' to 0|2047
    int xAccNrm = 1024 + (xAccRaw*1023);
    int yAccNrm = 1024 + (yAccRaw*1023);
    int zAccNrm = 1024 + (zAccRaw*1023);

    // convert to range of -90 to +90 degrees, or +90 to -90 degrees if inverted orientation
    if (bInv < 0) {
      xAng = map(xAccNrm, MIN, MAX, 90, -90);
      yAng = map(yAccNrm, MIN, MAX, 90, -90);
      zAng = map(zAccNrm, MIN, MAX, 90, -90);
    } else {
      xAng = map(xAccNrm, MIN, MAX, -90, 90);
      yAng = map(yAccNrm, MIN, MAX, -90, 90);
      zAng = map(zAccNrm, MIN, MAX, -90, 90);
    }

    // convert radians to degrees, inverting if so needed
    int pitch = (RAD_TO_DEG * (atan2(-xAng, -zAng) + PI)) * pDir;
    int roll = (RAD_TO_DEG * (atan2(-yAng, -zAng) + PI)) * rDir;

    // correct for axis orientation flags
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

    showAttitude(pitch, roll);

    delay(100);   // 10hz rate
}

void showSplash() {

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_busdisplay11x5_te);
  u8g2.drawStr(1,15,"     ~MHz");
  u8g2.drawStr(1,30,"  Roll-o-Meter");
  u8g2.sendBuffer();
  delay(1500);

}

void startIMU() {

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_10x20_te);

  //Call .begin() to configure the IMUs & show status on oled
  if (myIMU.begin() != 0) {

    u8g2.drawStr(labelX,labelY,"IMU BORKED");
    u8g2.sendBuffer();

    digitalWrite(gLed,led_Off);
    digitalWrite(oLed,led_On);
    digitalWrite(rLed,led_On);
    digitalWrite(wHorn,wHorn_On);

    delay(15000);  // delay an annoingly long time

  } else {

    u8g2.drawStr(labelX,labelY,"IMU OK");
    u8g2.sendBuffer();

    digitalWrite(gLed,led_On);
    digitalWrite(oLed,led_Off);
    digitalWrite(rLed,led_Off);
    digitalWrite(wHorn,wHorn_Off);

    delay(3000);

  }

}

void showAttitude(int pitch, int roll) {

  // clear attitude caution and warning flags on each iteration

  int pCaut = 0;
  int pWarn = 0;
  int rCaut = 0;
  int rWarn = 0;

  // check pitch attitude and set caution and warning flags
  if (pitch >= pTipFwdAngWarn || pitch <= pTipAftAngWarn) {
    pWarn = 1;
  } else if (pitch >= pTipFwdAngCaut || pitch <= pTipAftAngCaut) {
    pCaut = 1;
  } 

  // check roll attitude and set caution and warning flags 
  if (roll >= rTipRtAngWarn || roll <= rTipLtAngWarn) {
    rWarn = 1;
  } else if (roll >= rTipRtAngCaut || roll <= rTipLtAngCaut) {
    rCaut = 1;
  } 

  // set led and horn based on sensed and set warning/caution flags
  if (pWarn == 1 || rWarn == 1) {

    digitalWrite(gLed,led_Off);
    digitalWrite(oLed,led_Off);
    digitalWrite(rLed,led_On);
    digitalWrite(wHorn,wHorn_On);

  } else if (pCaut == 1 || rCaut == 1) {

    digitalWrite(gLed,led_Off);
    digitalWrite(oLed,led_On);
    digitalWrite(rLed,led_Off);
    digitalWrite(wHorn,wHorn_Off);

  } else {  // (pWarn == 0 && rWarn == 0)

    digitalWrite(gLed,led_On);
    digitalWrite(oLed,led_Off);
    digitalWrite(rLed,led_Off);
    digitalWrite(wHorn,wHorn_Off);

  }

  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_10x20_te);   // 10w x 20h A=13

  //draw labels and populate data passed
  u8g2.drawStr(labelX,labelY,"P     R");

  // draw values
  itoa(pitch,msgString,10);
  u8g2.drawStr(pitchX,pitchY,msgString);

  itoa(roll,msgString,10);
  u8g2.drawStr(rollX,rollY,msgString);

  // draw degree symbol (pos dependent on signs/values of pitch and roll values)
  String temp1 = String(pitch);
  String temp2 = String(roll);
  u8g2.drawUTF8(pitchX+((temp1.length()+1)*cWidth)+degSymOffset,labelY,"°");
  u8g2.drawUTF8(rollX+((temp2.length()+1)*cWidth)+degSymOffset,labelY,"°");

  // and push ther buffer out to the display
  u8g2.sendBuffer();

}
