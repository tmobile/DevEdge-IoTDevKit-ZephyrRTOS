GNSS firmware hot start test
############################

Overview
********

This is an example of how you implement hot/warm starts for the CXD5605
chip.  It should help you take advantage of this feature in your code.

Building and Running
********************

This has been built for the dev edge board.  It takes the normal build command as
in the following:

west build -b tmo_dev_edge zephyr/samples/sensor/cxd5605_hot -DBOARD_ROOT=tmo-zephyr-sdk/

This sample is a standalone application.  You build it using the above
command and flash it.  The GNSS chip firmware verification test will start.
You should see the output below.  If there are errors they will be printed out.
