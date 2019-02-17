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


## Integrating TFMicro and ML

### Controlling Crazyflie Onboard

We will be integrating TFMicro into this system by creating our own deck
boot sequence and forcing the crazyflie to run that on startup. If you want
to run your own code on startup, make a `.c` file in `src/deck/drivers/src/`.
My example was the file `src/deck/drivers/src/sequence.c`. Then, to make sure
that these files are compiled, make sure to add in the Makefile a line
`PROJ_OBJ += sequence.o`. 

Then, to make the crazyflie load this deck code in forcibly when it boots up,
make sure to either add a line `CFLAGS += -DDECK_FORCE=<sequence name>` to
the Makefile or the file `tools/make/config.mk` before compiling. Note that
`<sequence name>` is set in the `sequence.c` file - look at all of the deck
driver code examples in the same directory to see how this variable is set. 

### Integrating TF-Micro

All of the code for the crazyflie is in C, while we require C++ (and a few
C++11 extensions) for compiling TF Micro. The easiest way to deal with this
is to compile TF Micro, and create a wrapper class to compile with C symbols
and call it from your `sequence.c` file. More information about how to
convert ML models to the correct format can be found in the folder `tfmicro/`,
and [here](tfmicro/README.md).

## Useful Links

* https://github.com/bitcraze/crazyflie-firmware-experimental/blob/icra-2017/src/modules/src/retrace.c#L157

* https://forum.bitcraze.io/viewtopic.php?t=2723

* https://forum.bitcraze.io/viewtopic.php?f=6&t=2648&p=13352&hilit=demo#p13352

