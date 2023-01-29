
| Supported Targets        | ESP32 |
| ------------------------ | ----- |

| Based on ESP-IDF example |
| ------------------------ |

This repostory contains a simple driver for a L298N DC motor driver for an ESP32 using the esp-idf toolchain (NOT arduino). This code will not run on an ESP32-S2 as it does not have any motor control PWM drivers.

It is coded specifically for the [dfrobot breakout board](https://www.dfrobot.com/product-66.html), with accompanying [drobot L298N wiki](https://wiki.dfrobot.com/MD1.3_2A_Dual_Motor_Controller_SKU_DRI0002).

This driver assume a setup is used with two motors that fully control the direction of a robot, such as the [dfrobot devastator tracked robot](https://www.dfrobot.com/product-1477.html). It takes care of sending the correct signals to the motors given a direction and magnitude (think throttle) that the robot should go in. This is specifically done such that it allows a robot built using this driver to be controlled with a joystick.
