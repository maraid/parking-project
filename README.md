This project provides a control interface for Koala 2.5 differential drive mobile robot.

There is a RPi3 mounted on top of the robot which connects through serial interface to the robot, and through WiFi to the local area network.
You can send commands using MQTT topics. The robot expect a JSON file with two parameters: Circle radius, arch length. Negative radius means turning to the left, negative length means going backwards.
Example JSON:
{ "R": 0.5, "L": 1.57 }
meaning: go forward and right around a half circle with the radius 0.5 meter.

Licenses:

General:
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>

IMU folder:
Code taken BerryIMU GitHub page, see licensing there:
https://github.com/mwilliams03/BerryIMU
