# prosthetic_arm_control

## Overview
This project is a continution of an ongoing project to develop a transhumeral (above elbow) prosthetic arm that swings in time with a user's stride. 

This repository is a guide to using the zephyr based software onto the teensy 4.1 this project uses.

## Project Setup
### Zephyr
Use the zephyr getting started guide to set up your zephyr environment on your desktop. This project was developed with Zephyr 3.3.0 and Zephyr SDK 0.16.1 and will not work with other versions.
```
https://docs.zephyrproject.org/latest/develop/getting_started/index.html
```

If you do not have experience with zephyr, as I did at the start of this project, I found the following Nordic Semiconductor tutorial helpful. Although it centers arround the nRF Connect SDK, the Zephyr fundamentals that this tutorial covers are widely applicable.
```
https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/
```

### Repository
Next, clone this repository into your `zephyrproject/zephyr` folder.
```
cd zephyrproject/zephyr
git clone https://github.com/Mjenz/prosthetic_arm_control.git
``` 
Inside this `prosthetic_arm_control` folder that you will build and flash the application
