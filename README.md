# prosthetic_arm_control

## Overview
This project is a continution of an ongoing project to develop a transhumeral (above elbow) prosthetic arm that swings in time with a user's stride. 

This repository is a guide to using the zephyr based software onto the teensy 4.1 this project uses.

## Project Setup
### Zephyr
Use the zephyr [getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to set up your zephyr environment on your desktop. This project was developed with Zephyr 3.3.0 and Zephyr SDK 0.16.1 and will not work with other versions.

If you do not have experience with zephyr, as I did at the start of this project, I found the [nRF Connect SDK tutorial](https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/) helpful. Although it centers arround the nRF Connect SDK, the Zephyr fundamentals that this tutorial covers are widely applicable.

Since teensy's are not fully supported by zephyr, you might run into unique problems. This [tutorial](https://github.com/ufechner7/zephyr_teensy4_test) and the github [issue thread](https://github.com/zephyrproject-rtos/zephyr/issues/30204) it originiated from are helpful resources.
### Repository
Next, clone this repository into your `zephyrproject/zephyr` folder.
```
cd zephyrproject/zephyr
git clone https://github.com/Mjenz/prosthetic_arm_control.git
``` 
Inside this `prosthetic_arm_control` folder that you will build and flash the application.

### Building
Zephyr has a powerful command line tool known as west. Use the code below to build your application.
```
west build -p always -b teensy41 
``
Typically, west would also be used to flash the application with `west flash`, however, since the teensy series of microcontrollers is not fully supported on zephyr you cannot use this method.

### Flashing
To flash your application onto the teensy 4.1 you will use the [teensy command line tool](https://www.pjrc.com/teensy/loader_cli.html). You can install the tool using the following command for macOS or similar for other systems. (Requires Homebrew)
```
brew install teensy_loader_cli
```

To flash the application via usb use the following command.
```
teensy_loader_cli --mcu=TEENSY41 -w build/zephyr/zephyr.hex
```

Now your teensy has the code onboard! Something that I have found in testing is that when flashing the teensy while using the prosthetic arm, it is helpful to disconnect the teensy from power entirely before flashing. I have found if I do not do this the teensy cannot connect to the IMU or current sensor.

## Interfacing with hardware
### Motor
The [hiwonder servo motor](https://www.hiwonder.com/products/hps-3518sg) has been hijacked by soldering wires directly to the m+ and m- motor inputs. These wires, in addition to the motor's existing 3 wires are what interface the motor to the teensy. 

### Battery
The current setup uses a 11V 2200mAh lithium polymer battery.

## Lets use it
### Directions
The software should work as is on the github. Once your teensy is flashed and the sensor unit is connected to the motor, the prosthesis should stay still until swinging or walking is detected. Since the imu cannot determine the cadance of the user for roughly 3 seconds, the prosthesis will run the 'base swing' thread until a pace has been determined. Then it will switch into 'controlled swing' mode where it will calibrate its swing with the pace of the user.

### Plotting
Oftentimes it can be difficult to asses the accuracy of the prosthetis by eye. To create plots of the prosthetics trajectory or the imu magnitude, you can use the `read_zephyr_serial.py` scipt included in the github. This python script reads from the serial port that the teensy is connected to and creates the corresponding plot. Make sure the comment in the correct data output that you want in the `main.c` file, and the correct serial read function in the python file.

### Warnings
1. Since the motor is controlled by PWM, the signal can be distorted if wires of excessive length are used. Be sure to keep your wires short to prevent unpredictable behavior from the prosthetic arm.