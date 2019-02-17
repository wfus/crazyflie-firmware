# TF MicroFlie

This is a fork of the crazyflie 2.0 firmware, which is available
[here](https://www.github.com/bitcraze/crazyflie-firmware). We will mainly
be using custom firmware to run small ML models locally on the STM32F4
microcontroller on the actual Crazyflie.

## Installation

### Install a toolchain

#### OS X
```bash
brew tap PX4/homebrew-px4
brew install gcc-arm-none-eabi
```
#### Debian/Ubuntu

Tested on Ubuntu 14.04 64b and Ubuntu 16.04 64b:

For ubuntu 14.04 :

```bash
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
```

For ubuntu 16.04 :

```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
```

After adding the corresponding repositories, execute the following commands

```bash
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi
```

### Cloning

This repository uses git submodules. Clone with the --recursive flag, or make
sure to update the submodules manually.

```bash
cd crazyflie-firmware
git submodule init
git submodule update
```

## Compiling

### Crazyflie 2.0

To make the project, you can just type in `make`. Then, we will want to flash
the Crazyflie. To do this, you will need to following the installation
instructions for the radio bootloader and install the python libraries, 
available [here](https://www.github.com/bitcraze/crazyflie-client-python) and
[here](https://www.github.com/bitcraze/crazyflie-lib-python).

Once all of those steps are done and the python libraries for bootloading are
installed, you can now compile and flash the image onto the crazyflie. Make
sure that the crazyflie is in bootloader mode (usually by turning it off and
then holding the power button for 3 seconds).

```bash
make clean
make -j7
make cload
```

### config.mk
To create custom build options create a file called config.mk in the tools/make/
folder and fill it with options. This is the main file that you will edit in
order to add your code to be run on the crazyflie.

## Useful Links

* https://github.com/bitcraze/crazyflie-firmware-experimental/blob/icra-2017/src/modules/src/retrace.c#L157

* https://forum.bitcraze.io/viewtopic.php?t=2723

* https://forum.bitcraze.io/viewtopic.php?f=6&t=2648&p=13352&hilit=demo#p13352

