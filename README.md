# FreeRTOS cell for the jailhouse hypervisor on X86(QEMU)
This repo is fork from https://github.com/siemens/freertos-cell

This project aims at run FreeRTOS on x86 QEMU enviroment with LINUX.

# FreeRTOS cell for the jailhouse hypervisor

[FreeRTOS](http://www.freertos.org/) is a real time operation system
for embedded systems. It is widely used on ARM based microprocessor boards.
[Jailhouse](https://github.com/siemens/jailhouse) is a partitioning Hypervisor based on Linux.
For more information on both systems refer to the corresponding web pages.

This project aims at getting both systems run together on a multicore ARM processor system.
It allows to combine the general purpose OS Linux with a hard real time OS. Both systems are
almost isolated from each other by the underlying hypervisor.

# Prerequisites

- At the moment the system runs only on a [Banana Pi board](https://www.google.de/#q=banana+pi)
- Make sure Jailhouse is running correctly on this board. Installation instructions can
  be found [here](https://github.com/siemens/jailhouse#setup-on-banana-pi-arm-board).
- A working cross compiler toolchain for your target platform

# Setup

First of all clone this repository

    FREERTOS_CELL_DIR=/home/user/freertos-cell
    git clone https://github.com/siemens/freertos-cell.git $FREERTOS_CELL_DIR

## Jailhouse configuration

Let's assume your Jailhouse sources reside in the directory

    JAILHOUSE_DIR=/home/user/jailhouse
    
then do the following to build new cell description files for the hypervisor

1. Root cell description
    
          cp $FREERTOS_CELL_DIR/jailhouse-configs/bananapi.c $JAILHOUSE_DIR/configs/

2. RTOS cell description

          cp $FREERTOS_CELL_DIR/jailhouse-configs/bananapi-freertos-demo.c $JAILHOUSE_DIR/configs/

3. Rebuild your jailhouse subsystem. In the directory "$JAILHOUSE_DIR/configs" you will get
   two new files which are used in the next step.

