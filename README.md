# -MHz-Roll-o-Meter
Based on/Inspired by Scale Built RC's Roll-o-Meter of years past.

Hardware:
- Seeed Studio Xiao nRF52840 BLE Sense SBC
- 128*32 OLED Display with SSD1306 driver
- Active buzzer with active low digital input to trigger 
- 3ea Discrete LEDs with 220 ohm current-limiting resistors
- 1ea Discrete SPST Momentary PB switch for user interaction

Arduino IDE:    **Arduino-2.3.x**

Author:         **Tod Wulff (a.k.a. MegaHurtz/~MHz)**

Date:           **23 April 2024**

Version:        **v1.0**

Description:   
- See video for early implementation: [~MHz Roll-o-Meter](https://youtu.be/2NaQaHJ9XEI)

Inspiration/References:
- Scale Built RC's Roll-o-Meter - Thanks, brother! - https://www.youtube.com/watch?v=PfeZpKERkO8
- https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
- https://github.com/camerontech/inclinometer
- https://forum.seeedstudio.com/t/xiao-sense-accelerometer-examples-and-low-power/270801
- https://forum.seeedstudio.com/t/seeed-xiao-ble-nrf52840-sense-giving-bad-gyroscope-data/274134/11
- https://adam-meyer.com/arduino/sensing-orientation-with-the-adxl335-arduino  <-- this
- plus a bunch of others for the OLED display (using u8g2 lib)

Adopted code to be easily navigable using Arduino IDE's code-folding: ![XiaoInclinometer Code Folding](https://i.imgur.com/TLFXAx1.png)

I'm not a dev, but rather an old-arse script kiddie hack, so my Arduino/C++ coding style is not mature and, as evidenced in the code, malleable.  It is what it is.

During the initial development efforts last week, I implemented LittleFS, imputing a temporary adoption of Mbed core use.  I wanted to have a config file and then code a run-time config evolution, but it proved to be too little ROI for my simple use of the app.  I didn't get any responses to a [help request on the Arduino Discord](https://discord.com/channels/420594746990526466/1230943312718991410).  In the end, it proved easier to 'hard code' operational parameters as depicted in the preprocessor directives.  I maintained the use of proper variables for these parameters in case I decide to revisit doing so.

Also, I had considered using a Neopixel LED or a DotStar LED, but the implications of doing so were distasteful ('cause I am lazy and the multiple libraries/config made me cringe), so I abandoned considering same.

If anyone wants to submit a PR that implements a config file, using LittleFS, or implements the use of Neopixel or DotStar LEDs, I'd welcome same.

If this ends up being used by anyone besides myself, I would like to be made aware of same:  [MegaHurtz \<at\> engineer \<dot\> com](mailto://MegaHurtz\<at\>engineer\<dot\>com) or one can join and hang out on our AMA Club's Discord: [GLSA Discord Invite Link](https://discord.gg/EfWhUDrXxR)

Thanks.  Enjoy!

~MHz
