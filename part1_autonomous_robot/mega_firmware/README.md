# Arduino Mega firmware

The Mega does the low level work on the robot: it drives the two rear motors through the L298N, counts both wheel encoders, reads the BNO055, passes on the NEO-M8N GPS and switches the sprayer pump relay. The Jetson Nano talks to it over USB with the `mega_bridge` node in agridoc_bringup, which turns this into the usual ROS topics (`cmd_vel` in, `odom`, `joint_states`, `imu/data` and `gps/fix` out, `pump` to switch the sprayer).

The wheel speed loop runs on the Mega at 50 Hz (feed forward from the top speed and the PWM the motors need to start, plus a PI correction from the encoders), so the Jetson only sends the speed each wheel should turn at. If the Jetson stops sending for 0.5 s the Mega stops both wheels and the pump.

## Wiring

| Part | Pin | Mega |
|---|---|---|
| L298N | ENA (jumper removed) | 9 |
| | IN1 | 8 |
| | IN2 | 7 |
| | ENB (jumper removed) | 10 |
| | IN3 | 11 |
| | IN4 | 12 |
| | GND | GND |
| Left encoder | A | 2 |
| | B | 4 |
| | VCC / GND | 5V / GND |
| Right encoder | A | 3 |
| | B | 5 |
| | VCC / GND | 5V / GND |
| BNO055 | SDA | 20 |
| | SCL | 21 |
| | VIN / GND | 5V / GND |
| Relay module | IN | 22 |
| | VCC / GND | 5V / GND |
| NEO-M8N GPS | TX | 19 (RX1) |
| | RX | not connected |
| | VCC / GND | 5V / GND |

The left motor goes on OUT1/OUT2 of the L298N and the right motor on OUT3/OUT4. The 12 V battery feeds the L298N motor input and the pump through the relay (COM and NO). The relay module is active low. The BNO055 is the Adafruit board, which has its own regulator and level shifting, so 5 V on VIN and the 5 V I2C pullups of the Mega are fine. The GPS board has its own 3.3 V regulator. Its TX (3.3 V) is high enough for the Mega's RX1, and its RX is left open so the Mega's 5 V TX1 never reaches it. The module keeps its factory settings: 9600 baud and one fix a second. It sits on the lid at the back of the robot with the antenna facing up, away from the lidar motor. The Mega is powered from the Jetson over USB and its GND is shared with the L298N and the battery.

## Libraries

- Arduino IDE 1.8.19 with Arduino AVR Boards 1.8.5
- Adafruit BNO055 1.6.0
- Adafruit Unified Sensor 1.1.6
- Adafruit BusIO 1.13.2

All of them install from the Library Manager. Open `mega_firmware.ino`, pick Tools > Board > Arduino Mega or Mega 2560 and upload.

## Serial protocol

115200 baud, one text line per message.

Jetson to Mega, sent 20 times a second by `mega_bridge`:

    C <left wheel rad/s> <right wheel rad/s> <pump 0 or 1>

Mega to Jetson, every 20 ms:

    E <millis> <left ticks> <right ticks>
    I <qw> <qx> <qy> <qz> <gx> <gy> <gz> <ax> <ay> <az>

and once a second the GGA sentence from the GPS exactly as the module sent it, for example:

    $GNGGA,101112.00,2010.13800,N,07359.22000,E,1,11,0.80,571.2,M,-66.1,M,,*64

Ticks count up when the wheel turns forward. The gyro is in rad/s and the acceleration in m/s^2. `mega_bridge` checks the NMEA checksum and publishes the position as `gps/fix`; the covariance comes from the HDOP times the module's 2.5 m accuracy. After a reset the Mega prints `READY` (or `READY NO_IMU` if it can't find the BNO055).

## Checks with the wheels off the ground

Open the Serial Monitor at 115200 baud with "Newline" line ending.

1. Turn each wheel forward by hand one full turn. Its count in the `E` lines should go up by about 1393 (7 pulses per motor turn, both edges, 99.5:1). If it goes down, flip `LEFT_ENC_INVERT` or `RIGHT_ENC_INVERT`. If it is far from 1393, put the measured number in `TICKS_PER_REV` here and in the `ticks_per_rev` parameter of `mega_bridge`.
2. Send `C 1 1 0`. Both wheels should turn forward for half a second. A wheel that turns backwards needs `LEFT_MOTOR_INVERT` or `RIGHT_MOTOR_INVERT` flipped.
3. Send `C 4 4 0` a few times and look at how many ticks pass in half a second (about 443 at 4 rad/s). `MAX_SPEED` (here) and `max_wheel_speed` (in `mega_bridge`) are the top wheel speed through the L298N. If the wheels reach a different speed at full PWM, set both to it.
4. Send `C 0 0 1`. The relay should click on for half a second and then off.
5. Take the robot outside with a clear view of the sky. After a minute or so (longer on the very first start) the `$GNGGA` lines get a position, a fix quality of 1 or 2 (the field after `E`) and 5 or more satellites. Without a fix the position fields are empty and the quality is 0.

## Running it from the Jetson

The Mega is a CH340 clone, so it shows up as `/dev/ttyUSB*` just like the RPLidar and the two can swap numbers between boots. `mega_bridge` uses the fixed name Linux makes under `/dev/serial/by-id/` instead. Check it once with:

    ls /dev/serial/by-id/

The default is `usb-1a86_USB2.0-Serial-if00-port0`. If yours differs, pass it with `-p port:=`.

    ros2 run agridoc_bringup mega_bridge

Watch the GPS (outside, once it has a fix):

    ros2 topic echo /gps/fix

Switch the pump by hand (it turns off 1 s after the messages stop):

    ros2 topic pub -r 5 /pump std_msgs/msg/Bool "{data: true}"
