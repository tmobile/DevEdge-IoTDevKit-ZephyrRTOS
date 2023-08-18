.. _intel_adl_crb:

Alder Lake CRB
##############

Overview
********
Alder Lake Reference Board (INTEL_ADL CRB) is an example implementation of a
compact single board computer with high performance for IoT edge devices.

This board configuration enables kernel support for the `INTEL_ADL`_ board.

Hardware
********

General information about the board can be found at the `INTEL_ADL`_ website.

Connections and IOs
===================

Refer to the `INTEL_ADL`_ website for more information.

Programming and Debugging
*************************
Use the following procedures for booting an image on a RPL CRB board.

.. contents::
   :depth: 1
   :local:
   :backlinks: top

Build Zephyr application
========================

#. Build a Zephyr application; for instance, to build the ``hello_world``
   application on Alder Lake CRB:

   .. zephyr-app-commands::
      :zephyr-app: samples/hello_world
      :board: intel_adl_crb
      :goals: build

   .. note::

      A Zephyr EFI image file named :file:`zephyr.efi` is automatically
      created in the build directory after the application is built.

Preparing the Boot Device
=========================

Prepare a USB flash drive to boot the Zephyr application image on
an Alder Lake CRB board.

#. Format the USB flash drive as FAT32.

   On Windows, open ``File Explorer``, and right-click on the USB flash drive.
   Select ``Format...``. Make sure in ``File System``, ``FAT32`` is selected.
   Click on the ``Format`` button and wait for it to finish.

   On Linux, graphical utilities such as ``gparted`` can be used to format
   the USB flash drive as FAT32. Alternatively, under terminal, find out
   the corresponding device node for the USB flash drive (for example,
   ``/dev/sdd``). Execute the following command:

   .. code-block:: console

      $ mkfs.vfat -F 32 <device-node>

   .. important::
      Make sure the device node is the actual device node for
      the USB flash drive. Or else you may erase other storage devices
      on your system, and will render the system unusable afterwards.

#. Copy the Zephyr EFI image file :file:`zephyr/zephyr.efi` to the USB drive.

Booting the Alder Lake CRB Board
================================

Boot the Alder Lake CRB board to the EFI shell with USB flash drive connected.

#. Insert the prepared boot device (USB flash drive) into the Alder Lake CRB board.

#. Connect the board to the host system using the serial cable and
   configure your host system to watch for serial data.  See
   `INTEL_ADL`_ website for more information.

   .. note::
      On Windows, PuTTY has an option to set up configuration for
      serial data.  Use a baud rate of 115200.

#. Power on the Alder Lake CRB board.

#. When the following output appears, press :kbd:`F7`:

   .. code-block:: console

      Press <DEL> or <ESC> to enter setup.

#. From the menu that appears, select the menu entry that describes
   that particular EFI shell.

#. From the EFI shell select Zephyr EFI image to boot.

   .. code-block:: console

      Shell> fs0:zephyr.efi

.. _INTEL_ADL: https://ark.intel.com/content/www/us/en/ark/products/codename/232598/products-formerly-alder-laken.html
