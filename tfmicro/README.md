# Tensorflow Micro on the Crazyflie 2.0

We will try to compile TF Micro on the Crazyflie 2.0, which definitely should
be able to handle the library size. The specs on the default Crazyflie:

    * 1 MB flash memory
    * Cortex M4 ARM chip

However, all of the default firmware is written in C. Therefore, in this segment
we will compile TF Micro for the microcontoller using C++ (`arm-none-eabi-g++`),
expose some libraries to C, and compile everything in the end in C so the
default firmware works (`arm-non-eabi-gcc`).
