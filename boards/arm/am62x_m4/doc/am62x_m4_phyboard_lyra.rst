.. _am62x_m4_phyboard_lyra:

AM62x phyBOARD-Lyra M4F Core
############################

Overview
********

The AM62x phyBOARD-Lyra board configuration is used by Zephyr applications
that run on the TI AM62x platform. The board configuration provides support
for the ARM Cortex-M4F MCU core and the following features:

- Nested Vector Interrupt Controller (NVIC)
- System Tick System Clock (SYSTICK)

The board configuration also enables support for the semihosting debugging console.

See the `PHYTEC AM62x Product Page`_ for details.

.. figure:: img/phyCORE-AM62x_Lyra_frontside.webp
   :align: center
   :alt: AM62x phyBOARD-Lyra

   PHYTEC phyBOARD-Lyra with the phyCORE-AM62x SoM

Hardware
********
The AM62x phyBOARD-Lyra kit features the AM62x SoC, which is composed of a
quad Cortex-A53 cluster and a single Cortex-M4 core in the MCU domain. Zephyr
is ported to run on the M4F core and the following listed hardware
specifications are used:

- Low-power ARM Cortex-M4F
- Memory

   - 256KB of SRAM
   - 2GB of DDR4

- Debug

   - XDS110 based JTAG

Supported Features
==================

The am62x_m4_phyboard_lyra configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| SYSTICK   | on-chip    | systick                             |
+-----------+------------+-------------------------------------+
| PINCTRL   | on-chip    | pinctrl                             |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial                              |
+-----------+------------+-------------------------------------+

Other hardware features are not currently supported by the port.

Devices
========
System Clock
------------

This board configuration uses a system clock frequency of 400 MHz.

DDR RAM
-------

The board has 2GB of DDR RAM available. This board configuration
allocates Zephyr 4kB of RAM (only for resource table: 0x9CC00000 to 0x9CC00400).

Serial Port
-----------

This board configuration uses a single serial communication channel with the
MCU domain UART (MCU_UART0).

SD Card
*******

Download PHYTEC's official `WIC`_ as well as `BMAP`_ and flash the WIC file with
an etching software onto an SD-card. This will boot Linux on the A53 application
cores of the SoM. These cores will then load the zephyr binary on the M4 core
using remoteproc.

The default configuration can be found in the defconfig file:

.. code-block:: console

   boards/arm/am62x_m4/am62x_m4_phyboard_lyra_defconfig

Flashing
********

The Linux running on the A53 uses the remoteproc framework to manage the M4F co-processor.
Therefore, the testing requires the binary to be copied to the SD card to allow the A53 cores to
load it while booting using remoteproc.

To test the M4F core, we build the `hello_world` sample with the following command.

.. code-block:: console

   # From the root of the Zephyr repository
   west build -p -b am62x_m4_phyboard_lyra samples/hello_world

This builds the program and the binary is present in the `build/zephyr` directory as `zephyr.elf`.

We now copy this binary onto the SD card in the `/lib/firmware` directory and name it as `am62-mcu-m4f0_0-fw`.

.. code-block:: console

   # Mount the SD card at sdcard for example
   sudo mount /dev/sdX sdcard
   # copy the elf to the /lib/firmware directory
   sudo cp --remove-destination zephyr.elf sdcard/lib/firmware/am62-mcu-m4f0_0-fw

The SD card can now be used for booting. The binary will now be loaded onto the M4F core on boot.

To allow the board to boot using the SD card, set the boot pins to the SD Card boot mode. Refer to `phyBOARD SD Card Booting Essentials`_.

After changing the boot mode, stop in U-Boot to enable the M4F co-processor.

.. code-block:: console

   setenv overlays k3-am62-phyboard-lyra-rpmsg.dtbo
   # Save the overlays variable permanently
   saveenv
   boot

The board should boot into Linux and the binary will run and print Hello world to the MCU_UART0
port.



.. _PHYTEC AM62x Product Page:
   https://www.phytec.com/product/phycore-am62x/

.. _WIC:
   https://download.phytec.de/Software/Linux/BSP-Yocto-AM62x/BSP-Yocto-AM62x-PD23.1.0/images/yogurt/phyboard-lyra-am62xx-2/phytec-qt5demo-image-phyboard-lyra-am62xx-2.wic.xz

.. _BMAP:
   https://download.phytec.de/Software/Linux/BSP-Yocto-AM62x/BSP-Yocto-AM62x-PD23.1.0/images/yogurt/phyboard-lyra-am62xx-2/phytec-qt5demo-image-phyboard-lyra-am62xx-2.wic.bmap

.. _phyBOARD SD Card Booting Essentials:
   https://docs.phytec.com/latest/phycore-am62x/bootingessentials/sdcard.html
