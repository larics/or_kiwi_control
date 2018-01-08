# Simulink control of the Kiwi SCARA robot

This repo contains the Simulink model and additional code for controlling the Kiwi SCARA robot using the ERT Linux real-time target. It provides the infrastructure for the Fundamentals of robotics course laboratory at FER.

## System overview

## Installation

The software has been tested with Matlab R2014a on Ubuntu 14.04 (and Ubuntu 16.04?). This repo provides the client side software. In order to control the robot, the Real-time Linux target machine must be set up as described in (TODO: Mirkovic rad za rektorovu, link na image).

### gcc 4.7

Matlab R2014a requires gcc-4.7, so this version must be installed and set as default system-wide before starting Matlab, in order for Matlab to find it. The `update-alternatives` tool offers a nice way of choosing between gcc versions. In this example (applicable to Ubuntu 16.04) we'll enable choosing between gcc 5 and gcc 4.7.

```
sudo apt install gcc-4.7 g++-4.7
sudo update alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-4.7
sudo update alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 100 --slave /usr/bin/g++ g++ /usr/bin/g++-5
```

This gives priority to gcc 5 (the default gcc version on Ubuntu 16.04), but lets us switch to gcc 4.7 using
```
sudo update-alternatives --config gcc
```

**Note:** If you are also developing other software on the same machine, keep in mind that you have made this change and that you might need to switch back to the default gcc version for other development.

### ERT Linux

Clone the [ERT Linux repo](https://github.com/larics/ert_linux.git) to a location of your choice (e.g. `~/Documents/MATLAB`) and make sure to check out the [Kiwi branch](https://github.com/larics/ert_linux/tree/kiwi). Then open Matlab, navigate to the `ert_linux` folder and run the installation script
```
>> install
```

which should add to path...

[BCI can driver](https://github.com/larics/bci)