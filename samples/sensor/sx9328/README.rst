.. _sx9328_sample:

SX9328 SAR proximity sensor.
############################

Overview
********

This sample reads the proximity flags from the SX9328 proximity sensor and
displays the results.

The SX9328 is a capacitive SAR sensor capable of
human sensing, as well as capacitive sensing of object. By default the sensor
has three sensing classes: "table", "body", and "proximity". Proximity simply
indicates whether any object is detected close to the sensor, "table"
indicates proximity to a low permitivity object, and "body" indicates proximity
to a high permitivity objects.

By default, this sample prints flags on triggers. To poll, simply disable
CONFIG_SX9328_TRIGGER.

Note: this sample does not use the Smart SAR mode.

Sample Output
=============

.. code-block:: console

   prox is 1
   bdy_prox is 0
   tbl_prox is 1
