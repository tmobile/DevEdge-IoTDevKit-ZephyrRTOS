.. _bluetooth_mesh_shell:

Bluetooth Mesh Shell
####################

The Bluetooth mesh shell subsystem provides a set of Bluetooth mesh shell commands for the :ref:`shell_api` module.
It allows for testing and exploring the Bluetooth mesh API through an interactive interface, without having to write an application.

The Bluetooth mesh shell interface provides access to most Bluetooth mesh features, including provisioning, configuration, and message sending.

Prerequisites
*************

The Bluetooth mesh shell subsystem depends on the application to create the composition data and do the mesh initialization.

Application
***********

The Bluetooth mesh shell subsystem is most easily used through the Bluetooth mesh shell application under ``tests/bluetooth/mesh_shell``.
See :ref:`shell_api` for information on how to connect and interact with the Bluetooth mesh shell application.

Basic usage
***********

The Bluetooth mesh shell subsystem adds a single ``mesh`` command, which holds a set of sub-commands. Every time the device boots up, make sure to call ``mesh init`` before any of the other Bluetooth mesh shell commands can be called::

	uart:~$ mesh init

This is done to ensure that all available log will be printed to the shell output.

Provisioning
============

The mesh node must be provisioned to become part of the network. This is only necessary the first time the device boots up, as the device will remember its provisioning data between reboots.

The simplest way to provision the device is through self-provisioning. To do this the user must provision the device with the default network key and address ``0x0001``, execute::

	uart:~$ mesh prov local 0 0x0001

Since all mesh nodes use the same values for the default network key, this can be done on multiple devices, as long as they're assigned non-overlapping unicast addresses. Alternatively, to provision the device into an existing network, the unprovisioned beacon can be enabled with ``mesh prov pb-adv on`` or ``mesh prov pb-gatt on``. The beacons can be picked up by an external provisioner, which can provision the node into its network.

Once the mesh node is part of a network, its transmission parameters can be controlled by the general configuration commands:

* To set the destination address, call ``mesh target dst <addr>``.
* To set the network key index, call ``mesh target net <NetIdx>``.
* To set the application key index, call ``mesh target app <AppIdx>``.

By default, the transmission parameters are set to send messages to the provisioned address and network key.

Configuration
=============

By setting the destination address to the local unicast address (``0x0001`` in the ``mesh prov local`` command above), we can perform self-configuration through any of the :ref:`bluetooth_mesh_shell_cfg_cli` commands.

A good first step is to read out the node's own composition data::

	uart:~$ mesh models cfg get-comp

This prints a list of the composition data of the node, including a list of its model IDs.

Next, since the device has no application keys by default, it's a good idea to add one::

	uart:~$ mesh models cfg appkey add 0 0

Message sending
===============

With an application key added (see above), the mesh node's transition parameters are all valid, and the Bluetooth mesh shell can send raw mesh messages through the network.

For example, to send a Generic OnOff Set message, call::

	uart:~$ mesh test net-send 82020100

.. note::
	All multibyte fields model messages are in little endian, except the opcode.

The message will be sent to the current destination address, using the current network and application key indexes. As the destination address points to the local unicast address by default, the device will only send packets to itself. To change the destination address to the All Nodes broadcast address, call::

	uart:~$ mesh target dst 0xffff

With the destination address set to ``0xffff``, any other mesh nodes in the network with the configured network and application keys will receive and process the messages we send.

.. note::
	To change the configuration of the device, the destination address must be set back to the local unicast address before issuing any configuration commands.

Sending raw mesh packets is a good way to test model message handler implementations during development, as it can be done without having to implement the sending model. By default, only the reception of the model messages can be tested this way, as the Bluetooth mesh shell only includes the foundation models. To receive a packet in the mesh node, you have to add a model with a valid opcode handler list to the composition data in ``subsys/bluetooth/mesh/shell.c``, and print the incoming message to the shell in the handler callback.

Parameter formats
*****************

The Bluetooth mesh shell commands are parsed with a variety of formats:

.. list-table:: Parameter formats
	:widths: 1 4 2
	:header-rows: 1

	* - Type
	  - Description
	  - Example
	* - Integers
	  - The default format unless something else is specified. Can be either decimal or hexadecimal.
	  - ``1234``, ``0xabcd01234``
	* - Hexstrings
	  - For raw byte arrays, like UUIDs, key values and message payloads, the parameters should be formatted as an unbroken string of hexadecimal values without any prefix.
	  - ``deadbeef01234``
	* - Booleans
	  - Boolean values are denoted in the API documentation as ``<val: on, off>``.
	  - ``on``, ``off``, ``enabled``, ``disabled``, ``1``, ``0``

Commands
********

The Bluetooth mesh shell implements a large set of commands. Some of the commands accept parameters, which are mentioned in brackets after the command name. For example, ``mesh lpn set <value: off, on>``. Mandatory parameters are marked with angle brackets (e.g. ``<NetKeyIndex>``), and optional parameters are marked with square brackets (e.g. ``[destination address]``).

The Bluetooth mesh shell commands are divided into the following groups:

.. contents::
	:depth: 1
	:local:

.. note::
	Some commands depend on specific features being enabled in the compile time configuration of the application. Not all features are enabled by default. The list of available Bluetooth mesh shell commands can be shown in the shell by calling ``mesh`` without any arguments.

General configuration
=====================

``mesh init``
-------------

	Initialize the mesh shell. This command must be run before any other mesh command.

``mesh reset-local``
--------------------

	Reset the local mesh node to its initial unprovisioned state. This command will also clear the Configuration Database (CDB) if present.

Target
======

The target commands enables the user to monitor and set the target destination address, network index and application index for the shell. These parameters are used by several commands, like provisioning, Configuration Client, etc.

``mesh target dst [destination address]``
-----------------------------------------

	Get or set the message destination address. The destination address determines where mesh packets are sent with the shell, but has no effect on modules outside the shell's control.

	* ``destination address``: If present, sets the new 16-bit mesh destination address. If omitted, the current destination address is printed.


``mesh target net [NetIdx]``
----------------------------

	Get or set the message network index. The network index determines which network key is used to encrypt mesh packets that are sent with the shell, but has no effect on modules outside the shell's control. The network key must already be added to the device, either through provisioning or by a Configuration Client.

	* ``NetIdx``: If present, sets the new network index. If omitted, the current network index is printed.


``mesh target app [AppIdx]``
----------------------------

	Get or set the message application index. The application index determines which application key is used to encrypt mesh packets that are sent with the shell, but has no effect on modules outside the shell's control. The application key must already be added to the device by a Configuration Client, and must be bound to the current network index.

	* ``AppIdx``: If present, sets the new application index. If omitted, the current application index is printed.

Low Power Node
==============

``mesh lpn set <value: off, on>``
---------------------------------

	Enable or disable Low Power operation. Once enabled, the device will turn off its radio and start polling for friend nodes.

	* ``value``: Sets whether Low Power operation is enabled.

``mesh lpn poll``
-----------------

	Perform a poll to the friend node, to receive any pending messages. Only available when LPN is enabled.

Testing
=======

``mesh test net-send <Hex string>``
-----------------------------------

	Send a raw mesh message with the current destination address, network and application index. The message opcode must be encoded manually.

	* ``hex string`` Raw hexadecimal representation of the message to send.

``mesh test iv-update``
-----------------------

	Force an IV update.


``mesh test iv-update-test <value: off, on>``
---------------------------------------------

	Set the IV update test mode. In test mode, the IV update timing requirements are bypassed.

	* ``value``: Enable or disable the IV update test mode.


``mesh test rpl-clear``
-----------------------

	Clear the replay protection list, forcing the node to forget all received messages.

.. warning::

	Clearing the replay protection list breaks the security mechanisms of the mesh node, making it susceptible to message replay attacks. This should never be performed in a real deployment.

Health Server Test
------------------

``mesh test health-srv add-fault <Fault ID>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Register a new Health Server Fault for the Linux Foundation Company ID.

	* ``Fault ID``: ID of the fault to register (``0x0001`` to ``0xFFFF``)


``mesh test health-srv del-fault [Fault ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Remove registered Health Server faults for the Linux Foundation Company ID.

	* ``Fault ID``: If present, the given fault ID will be deleted. If omitted, all registered faults will be cleared.

Provisioning
============

To allow a device to broadcast connectable unprovisioned beacons, the :kconfig:option:`CONFIG_BT_MESH_PROV_DEVICE` configuration option must be enabled, along with the :kconfig:option:`CONFIG_BT_MESH_PB_GATT` option.

``mesh prov pb-gatt <val: off, on>``
------------------------------------

	Start or stop advertising a connectable unprovisioned beacon. The connectable unprovisioned beacon allows the mesh node to be discovered by nearby GATT based provisioners, and provisioned through the GATT bearer.

	* ``val``: Enable or disable provisioning with GATT

To allow a device to broadcast unprovisioned beacons, the :kconfig:option:`CONFIG_BT_MESH_PROV_DEVICE` configuration option must be enabled, along with the :kconfig:option:`CONFIG_BT_MESH_PB_ADV` option.

``mesh prov pb-adv <val: off, on>``
-----------------------------------

	Start or stop advertising the unprovisioned beacon. The unprovisioned beacon allows the mesh node to be discovered by nearby advertising-based provisioners, and provisioned through the advertising bearer.

	* ``val``: Enable or disable provisioning with advertiser

To allow a device to provision devices, the :kconfig:option:`CONFIG_BT_MESH_PROVISIONER` and :kconfig:option:`CONFIG_BT_MESH_PB_ADV` configuration options must be enabled.

``mesh prov remote-adv <UUID> <NetKeyIndex> <addr> <AttentionDuration> [AuthType]``
-----------------------------------------------------------------------------------

	Provision a nearby device into the mesh. The mesh node starts scanning for unprovisioned beacons with the given UUID. Once found, the unprovisioned device will be added to the mesh network with the given unicast address, and given the network key indicated by ``NetKeyIndex``.

	* ``UUID``: UUID of the unprovisioned device.
	* ``NetKeyIndex``: Index of the network key to pass to the device.
	* ``addr``: First unicast address to assign to the unprovisioned device. The device will occupy as many addresses as it has elements, and all must be available.
	* ``AttentionDuration``: The duration in seconds the unprovisioned device will identify itself for, if supported. See :ref:`bluetooth_mesh_models_health_srv_attention` for details.
	* ``AuthType``: If present, the OOB authentication type used for provisioning.

		* ``no``: No OOB (default).
		* ``static``: Static OOB.
		* ``output``: Output OOB.
		* ``input``: Input OOB.

To allow a device to provision devices over GATT, the :kconfig:option:`CONFIG_BT_MESH_PROVISIONER` and :kconfig:option:`CONFIG_BT_MESH_PB_GATT_CLIENT` configuration options must be enabled.

``mesh prov remote-gatt <UUID> <NetKeyIndex> <addr> <AttentionDuration>``
-------------------------------------------------------------------------

	Provision a nearby device into the mesh. The mesh node starts scanning for connectable advertising for PB-GATT with the given UUID. Once found, the unprovisioned device will be added to the mesh network with the given unicast address, and given the network key indicated by ``NetKeyIndex``.

	* ``UUID``: UUID of the unprovisioned device.
	* ``NetKeyIndex``: Index of the network key to pass to the device.
	* ``addr``: First unicast address to assign to the unprovisioned device. The device will occupy as many addresses as it has elements, and all must be available.
	* ``AttentionDuration``: The duration in seconds the unprovisioned device will identify itself for, if supported. See :ref:`bluetooth_mesh_models_health_srv_attention` for details.

``mesh prov uuid [UUID: 1-16 hex values]``
------------------------------------------

	Get or set the mesh node's UUID, used in the unprovisioned beacons.

	* ``UUID``: If present, new 128-bit UUID value. Any missing bytes will be zero. If omitted, the current UUID will be printed. To enable this command, the :kconfig:option:`BT_MESH_SHELL_PROV_CTX_INSTANCE` option must be enabled.


``mesh prov input-num <number>``
--------------------------------

	Input a numeric OOB authentication value. Only valid when prompted by the shell during provisioning. The input number must match the number presented by the other participant in the provisioning.

	* ``number``: Decimal authentication number.


``mesh prov input-str <string>``
--------------------------------

	Input an alphanumeric OOB authentication value. Only valid when prompted by the shell during provisioning. The input string must match the string presented by the other participant in the provisioning.

	* ``string``: Unquoted alphanumeric authentication string.


``mesh prov static-oob [val: 1-16 hex values]``
-----------------------------------------------

	Set or clear the static OOB authentication value. The static OOB authentication value must be set before provisioning starts to have any effect. The static OOB value must be same on both participants in the provisioning. To enable this command, the :kconfig:option:`BT_MESH_SHELL_PROV_CTX_INSTANCE` option must be enabled.

	* ``val``: If present, indicates the new hexadecimal value of the static OOB. If omitted, the static OOB value is cleared.


``mesh prov local <NetKeyIndex> <addr> [IVIndex]``
--------------------------------------------------

	Provision the mesh node itself. If the Configuration database is enabled, the network key must be created. Otherwise, the default key value is used.

	* ``NetKeyIndex``: Index of the network key to provision.
	* ``addr``: First unicast address to assign to the device. The device will occupy as many addresses as it has elements, and all must be available.
	* ``IVindex``: Indicates the current network IV index. Defaults to 0 if omitted.


``mesh prov beacon-listen <val: off, on>``
------------------------------------------

	Enable or disable printing of incoming unprovisioned beacons. Allows a provisioner device to detect nearby unprovisioned devices and provision them. To enable this command, the :kconfig:option:`BT_MESH_SHELL_PROV_CTX_INSTANCE` option must be enabled.

	* ``val``: Whether to enable the unprovisioned beacon printing.

``mesh prov remote-pub-key <PubKey>``
--------------------------------------
	Provide Device public key.

	* ``PubKey`` - Device public key in big-endian.

``mesh prov auth-method input <Action> <Size>``
-----------------------------------------------
	From the provisioner device, instruct the unprovisioned device to use the specified Input OOB authentication action.

	* ``Action`` - Input action. Allowed values:
		* ``0`` - No input action.
		* ``1`` - Push action set.
		* ``2`` - Twist action set.
		* ``4`` - Enter number action set.
		* ``8`` - Enter String action set.
	* ``Size`` - Authentication size.

``mesh prov auth-method output <Action> <Size>``
------------------------------------------------
	From the provisioner device, instruct the unprovisioned device to use the specified Output OOB authentication action.

	* ``Action`` - Output action. Allowed values:
		* ``0`` - No output action.
		* ``1`` - Blink action set.
		* ``2`` - Vibrate action set.
		* ``4`` - Display number action set.
		* ``8`` - Display String action set.
	* ``Size`` - Authentication size.

``mesh prov auth-method static <Value>``
----------------------------------------
	From the provisioner device, instruct the unprovisioned device to use static OOB authentication, and use the given static authentication value when provisioning.

	* ``Value`` - Static OOB value.

``mesh prov auth-method none <Value>``
--------------------------------------
	From the provisioner device, don't use any authentication when provisioning new devices. This is the default behavior.

Proxy
=====

The Proxy Server module is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_GATT_PROXY` configuration option.

``mesh proxy identity-enable``
------------------------------

	Enable the Proxy Node Identity beacon, allowing Proxy devices to connect explicitly to this device. The beacon will run for 60 seconds before the node returns to normal Proxy beacons.

The Proxy Client module is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_PROXY_CLIENT` configuration option.

``mesh proxy connect <NetKeyIndex>``
------------------------------------

	Auto-Connect a nearby proxy server into the mesh.

	* ``NetKeyIndex``: Index of the network key to connect.


``mesh proxy disconnect <NetKeyIndex>``
---------------------------------------

	Disconnect the existing proxy connection.

	* ``NetKeyIndex``: Index of the network key to disconnect.


``mesh proxy solicit <NetKeyIndex>``
------------------------------------

	Begin Proxy Solicitation of a subnet. Support of this feature can be enabled through the :kconfig:option:`CONFIG_BT_MESH_PROXY_SOLICITATION` configuration option.

	* ``NetKeyIndex``: Index of the network key to send Solicitation PDUs to.

.. _bluetooth_mesh_shell_cfg_cli:

Models
======

Configuration Client
--------------------

The Configuration Client model is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_CFG_CLI` configuration option. This is implemented as a separate module (``mesh models cfg``) inside the ``mesh models`` subcommand list. This module will work on any instance of the Configuration Client model if the mentioned shell configuration options is enabled, and as long as the Configuration Client model is present in the model composition of the application. This shell module can be used for configuring itself and other nodes in the mesh network.

The Configuration Client uses general message parameters set by ``mesh target dst`` and ``mesh target net`` to target specific nodes. When the Bluetooth mesh shell node is provisioned, given that the :kconfig:option:`BT_MESH_SHELL_PROV_CTX_INSTANCE` option is enabled with the shell provisioning context initialized, the Configuration Client model targets itself by default. Similarly, when another node has been provisioned by the Bluetooth mesh shell, the Configuration Client model targets the new node. In most common use-cases, the Configuration Client is depending on the provisioning features and the Configuration database to be fully functional. The Configuration Client always sends messages using the Device key bound to the destination address, so it will only be able to configure itself and the mesh nodes it provisioned. The following steps are an example of how you can set up a device to start using the Configuration Client commands:

* Initialize the client node (``mesh init``).
* Create the CDB (``mesh cdb create``).
* Provision the local device (``mesh prov local``).
* The shell module should now target itself.
* Monitor the composition data of the local node (``mesh models cfg get-comp``).
* Configure the local node as desired with the Configuration Client commands.
* Provision other devices (``mesh prov beacon-listen``) (``mesh prov remote-adv``) (``mesh prov remote-gatt``).
* The shell module should now target the newly added node.
* Monitor the newly provisioned nodes and their addresses (``mesh cdb show``).
* Monitor the composition data of the target device (``mesh models cfg get-comp``).
* Configure the node as desired with the Configuration Client commands.

``mesh models cfg target get``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the target Configuration server for the Configuration Client model.

``mesh models cfg help``
^^^^^^^^^^^^^^^^^^^^^^^^

	Print information for the Configuration Client shell module.

``mesh models cfg reset``
^^^^^^^^^^^^^^^^^^^^^^^^^

	Reset the target device.

``mesh models cfg timeout [timeout in seconds]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get and set the Config Client model timeout used during message sending.

	* ``timeout in seconds``: If present, set the Config Client model timeout in seconds. If omitted, the current timeout is printed.


``mesh models cfg get-comp [page]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Read a composition data page. The full composition data page will be printed. If the target does not have the given page, it will return the last page before it.

	* ``page``: The composition data page to request. Defaults to 0 if omitted.


``mesh models cfg beacon [val: off, on]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the network beacon transmission.

	* ``val``: If present, enables or disables sending of the network beacon. If omitted, the current network beacon state is printed.


``mesh models cfg ttl [ttl: 0x00, 0x02-0x7f]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the default TTL value.

	* ``ttl``: If present, sets the new default TTL value. If omitted, the current default TTL value is printed.


``mesh models cfg friend [val: off, on]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the Friend feature.

	* ``val``: If present, enables or disables the Friend feature. If omitted, the current Friend feature state is printed:

		* ``0x00``: The feature is supported, but disabled.
		* ``0x01``: The feature is enabled.
		* ``0x02``: The feature is not supported.


``mesh models cfg gatt-proxy [val: off, on]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the GATT Proxy feature.

	* ``val``: If present, enables or disables the GATT Proxy feature. If omitted, the current GATT Proxy feature state is printed:

		* ``0x00``: The feature is supported, but disabled.
		* ``0x01``: The feature is enabled.
		* ``0x02``: The feature is not supported.


``mesh models cfg relay [<val: off, on> [<count: 0-7> [interval: 10-320]]]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the Relay feature and its parameters.

	* ``val``: If present, enables or disables the Relay feature. If omitted, the current Relay feature state is printed:

		* ``0x00``: The feature is supported, but disabled.
		* ``0x01``: The feature is enabled.
		* ``0x02``: The feature is not supported.

	* ``count``: Sets the new relay retransmit count if ``val`` is ``on``. Ignored if ``val`` is ``off``. Defaults to ``2`` if omitted.
	* ``interval``: Sets the new relay retransmit interval in milliseconds if ``val`` is ``on``. Ignored if ``val`` is ``off``. Defaults to ``20`` if omitted.

``mesh models cfg node-id <NetKeyIndex> [Identity]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or Set of current Node Identity state of a subnet.

	* ``NetKeyIndex``: The network key index to Get/Set.
	* ``Identity``: If present, sets the identity of Node Identity state.

``mesh models cfg polltimeout-get <LPN Address>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get current value of the PollTimeout timer of the LPN within a Friend node.

	* ``addr`` Address of Low Power node.

``mesh models cfg net-transmit-param [<count: 0-7> <interval: 10-320>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the network transmit parameters.

	* ``count``: Sets the number of additional network transmits for every sent message.
	* ``interval``: Sets the new network retransmit interval in milliseconds.


``mesh models cfg netkey add <NetKeyIndex> [val]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Add a network key to the target node. Adds the key to the Configuration Database if enabled.

	* ``NetKeyIndex``: The network key index to add.
	* ``val``: If present, sets the key value as a 128-bit hexadecimal value. Any missing bytes will be zero. Only valid if the key does not already exist in the Configuration Database. If omitted, the default key value is used.


``mesh models cfg netkey upd <NetKeyIndex> [val]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Update a network key to the target node.

	* ``NetKeyIndex``: The network key index to updated.
	* ``val``: If present, sets the key value as a 128-bit hexadecimal value. Any missing bytes will be zero. If omitted, the default key value is used.

``mesh models cfg netkey get``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of known network key indexes.


``mesh models cfg netkey del <NetKeyIndex>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Delete a network key from the target node.

	* ``NetKeyIndex``: The network key index to delete.


``mesh models cfg appkey add <NetKeyIndex> <AppKeyIndex> [val]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Add an application key to the target node. Adds the key to the Configuration Database if enabled.

	* ``NetKeyIndex``: The network key index the application key is bound to.
	* ``AppKeyIndex``: The application key index to add.
	* ``val``: If present, sets the key value as a 128-bit hexadecimal value. Any missing bytes will be zero. Only valid if the key does not already exist in the Configuration Database. If omitted, the default key value is used.

``mesh models cfg appkey upd <NetKeyIndex> <AppKeyIndex> [val]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Update an application key to the target node.

	* ``NetKeyIndex``: The network key index the application key is bound to.
	* ``AppKeyIndex``: The application key index to update.
	* ``val``: If present, sets the key value as a 128-bit hexadecimal value. Any missing bytes will be zero. If omitted, the default key value is used.

``mesh models cfg appkey get <NetKeyIndex>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of known application key indexes bound to the given network key index.

	* ``NetKeyIndex``: Network key indexes to get a list of application key indexes from.


``mesh models cfg appkey del <NetKeyIndex> <AppKeyIndex>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Delete an application key from the target node.

	* ``NetKeyIndex``: The network key index the application key is bound to.
	* ``AppKeyIndex``: The application key index to delete.


``mesh models cfg model app-bind <addr> <AppIndex> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Bind an application key to a model. Models can only encrypt and decrypt messages sent with application keys they are bound to.

	* ``addr``: Address of the element the model is on.
	* ``AppIndex``: The application key to bind to the model.
	* ``Model ID``: The model ID of the model to bind the key to.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.



``mesh models cfg model app-unbind <addr> <AppIndex> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Unbind an application key from a model.

	* ``addr``: Address of the element the model is on.
	* ``AppIndex``: The application key to unbind from the model.
	* ``Model ID``: The model ID of the model to unbind the key from.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.


``mesh models cfg model app-get <elem addr> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of application keys bound to a model.

	* ``elem addr``: Address of the element the model is on.
	* ``Model ID``: The model ID of the model to get the bound keys of.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.


``mesh models cfg model pub <addr> <mod id> [cid] [<PubAddr> <AppKeyIndex> <cred: off, on> <ttl> <period> <count> <interval>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the publication parameters of a model. If all publication parameters are included, they become the new publication parameters of the model. If all publication parameters are omitted, print the current publication parameters of the model.

	* ``addr``: Address of the element the model is on.
	* ``Model ID``: The model ID of the model to get the bound keys of.
	* ``cid``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.

	Publication parameters:

		* ``PubAddr``: The destination address to publish to.
		* ``AppKeyIndex``: The application key index to publish with.
		* ``cred``: Whether to publish with Friendship credentials when acting as a Low Power Node.
		* ``ttl``: TTL value to publish with (``0x00`` to ``0x07f``).
		* ``period``: Encoded publication period, or 0 to disable periodic publication.
		* ``count``: Number of retransmission for each published message (``0`` to ``7``).
		* ``interval`` The interval between each retransmission, in milliseconds. Must be a multiple of 50.

``mesh models cfg model pub-va <addr> <UUID> <AppKeyIndex> <cred: off, on> <ttl> <period> <count> <interval> <mod id> [cid]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Set the publication parameters of a model.

	* ``addr``: Address of the element the model is on.
	* ``Model ID``: The model ID of the model to get the bound keys of.
	* ``cid``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.

	Publication parameters:

		* ``UUID``: The destination virtual address to publish to.
		* ``AppKeyIndex``: The application key index to publish with.
		* ``cred``: Whether to publish with Friendship credentials when acting as a Low Power Node.
		* ``ttl``: TTL value to publish with (``0x00`` to ``0x07f``).
		* ``period``: Encoded publication period, or 0 to disable periodic publication.
		* ``count``: Number of retransmission for each published message (``0`` to ``7``).
		* ``interval`` The interval between each retransmission, in milliseconds. Must be a multiple of 50.


``mesh models cfg model sub-add <elem addr> <sub addr> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Subscription the model to a group address. Models only receive messages sent to their unicast address or a group or virtual address they subscribe to. Models may subscribe to multiple group and virtual addresses.

	* ``elem addr``: Address of the element the model is on.
	* ``sub addr``: 16-bit group address the model should subscribe to (``0xc000`` to ``0xFEFF``).
	* ``Model ID``: The model ID of the model to add the subscription to.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.


``mesh models cfg model sub-del <elem addr> <sub addr> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Unsubscribe a model from a group address.

	* ``elem addr``: Address of the element the model is on.
	* ``sub addr``: 16-bit group address the model should remove from its subscription list (``0xc000`` to ``0xFEFF``).
	* ``Model ID``: The model ID of the model to add the subscription to.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.


``mesh models cfg model sub-add-va <elem addr> <Label UUID> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Subscribe the model to a virtual address. Models only receive messages sent to their unicast address or a group or virtual address they subscribe to. Models may subscribe to multiple group and virtual addresses.

	* ``elem addr``: Address of the element the model is on.
	* ``Label UUID``: 128-bit label UUID of the virtual address to subscribe to. Any omitted bytes will be zero.
	* ``Model ID``: The model ID of the model to add the subscription to.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.


``mesh models cfg model sub-del-va <elem addr> <Label UUID> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Unsubscribe a model from a virtual address.

	* ``elem addr``: Address of the element the model is on.
	* ``Label UUID``: 128-bit label UUID of the virtual address to remove the subscription of. Any omitted bytes will be zero.
	* ``Model ID``: The model ID of the model to add the subscription to.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.

``mesh models cfg model sub-ow <elem addr> <sub addr> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Overwrite all model subscriptions with a single new group address.

	* ``elem addr``: Address of the element the model is on.
	* ``sub addr``: 16-bit group address the model should added to the subscription list (``0xc000`` to ``0xFEFF``).
	* ``Model ID``: The model ID of the model to add the subscription to.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.

``mesh models cfg model sub-ow-va <elem addr> <Label UUID> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Overwrite all model subscriptions with a single new virtual address. Models only receive messages sent to their unicast address or a group or virtual address they subscribe to. Models may subscribe to multiple group and virtual addresses.

	* ``elem addr``: Address of the element the model is on.
	* ``Label UUID``: 128-bit label UUID of the virtual address as the new Address to be added to the subscription list. Any omitted bytes will be zero.
	* ``Model ID``: The model ID of the model to add the subscription to.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.

``mesh models cfg model sub-del-all <elem addr> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Remove all group and virtual address subscriptions from of a model.

	* ``elem addr``: Address of the element the model is on.
	* ``Model ID``: The model ID of the model to Unsubscribe all.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.

``mesh models cfg model sub-get <elem addr> <Model ID> [Company ID]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of addresses the model subscribes to.

	* ``elem addr``: Address of the element the model is on.
	* ``Model ID``: The model ID of the model to get the subscription list of.
	* ``Company ID``: If present, determines the Company ID of the model. If omitted, the model is a Bluetooth SIG defined model.


``mesh models cfg krp <NetKeyIdx> [Phase]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the key refresh phase of a subnet.

	* ``NetKeyIdx``: The identified network key used to Get/Set the current Key Refresh Phase state.
	* ``Phase``: New Key Refresh Phase. Valid phases are 0, 1 or 2.

``mesh models cfg hb-sub [<src> <dst> <period>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the Heartbeat subscription parameters. A node only receives Heartbeat messages matching the Heartbeat subscription parameters. Sets the Heartbeat subscription parameters if present, or prints the current Heartbeat subscription parameters if called with no parameters.

	* ``src``: Unicast source address to receive Heartbeat messages from.
	* ``dst``: Destination address to receive Heartbeat messages on.
	* ``period``: Logarithmic representation of the Heartbeat subscription period:

		* ``0``: Heartbeat subscription will be disabled.
		* ``1`` to ``17``: The node will subscribe to Heartbeat messages for 2\ :sup:`(period - 1)` seconds.


``mesh models cfg hb-pub [<dst> <count> <period> <ttl> <features> <NetKeyIndex>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get or set the Heartbeat publication parameters. Sets the Heartbeat publication parameters if present, or prints the current Heartbeat publication parameters if called with no parameters.

	* ``dst``: Destination address to publish Heartbeat messages to.
	* ``count``: Logarithmic representation of the number of Heartbeat messages to publish periodically:

		* ``0``: Heartbeat messages are not published periodically.
		* ``1`` to ``17``: The node will periodically publish 2\ :sup:`(count - 1)` Heartbeat messages.
		* ``255``: Heartbeat messages will be published periodically indefinitely.

	* ``period``: Logarithmic representation of the Heartbeat publication period:

		* ``0``: Heartbeat messages are not published periodically.
		* ``1`` to ``17``: The node will publish Heartbeat messages every 2\ :sup:`(period - 1)` seconds.

	* ``ttl``: The TTL value to publish Heartbeat messages with (``0x00`` to ``0x7f``).
	* ``features``: Bitfield of features that should trigger a Heartbeat publication when changed:

		* ``Bit 0``: Relay feature.
		* ``Bit 1``: Proxy feature.
		* ``Bit 2``: Friend feature.
		* ``Bit 3``: Low Power feature.

	* ``NetKeyIndex``: Index of the network key to publish Heartbeat messages with.


Health Client
-------------

The Health Client model is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_HEALTH_CLI` configuration option. This is implemented as a separate module (``mesh models health``) inside the ``mesh models`` subcommand list. This module will work on any instance of the Health Client model if the mentioned shell configuration options is enabled, and as long as one or more Health Client model(s) is present in the model composition of the application. This shell module can be used to trigger interaction between Health Clients and Servers on devices in a Mesh network.

By default, the module will choose the first Health Client instance in the model composition when using the Health Client commands. To choose a spesific Health Client instance the user can utilize the commands ``mesh models health instance set`` and ``mesh models health instance get-all``.

The Health Client may use the general messages parameters set by ``mesh target dst``, ``mesh target net`` and ``mesh target app`` to target specific nodes. If the shell target destination address is set to zero, the targeted Health Client will attempt to publish messages using its configured publication parameters.

``mesh models health instance set <Elem_idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Set the Health Client model instance to use.

	* ``Elem_idx``: Element index of Health Client model.

``mesh models health instance get-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Prints all available Health Client model instances on the device.

``mesh models health fault-get <Company ID>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of registered faults for a Company ID.

	* ``Company ID``: Company ID to get faults for.


``mesh models health fault-clear <Company ID>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Clear the list of faults for a Company ID.

	* ``Company ID``: Company ID to clear the faults for.


``mesh models health fault-clear-unack <Company ID>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Clear the list of faults for a Company ID without requesting a response.

	* ``Company ID``: Company ID to clear the faults for.


``mesh models health fault-test <Company ID> <Test ID>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Invoke a self-test procedure, and show a list of triggered faults.

	* ``Company ID``: Company ID to perform self-tests for.
	* ``Test ID``: Test to perform.


``mesh models health fault-test-unack <Company ID> <Test ID>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Invoke a self-test procedure without requesting a response.

	* ``Company ID``: Company ID to perform self-tests for.
	* ``Test ID``: Test to perform.


``mesh models health period-get``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the current Health Server publish period divisor.


``mesh models health period-set <divisor>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Set the current Health Server publish period divisor. When a fault is detected, the Health Server will start publishing is fault status with a reduced interval. The reduced interval is determined by the Health Server publish period divisor: Fault publish period = Publish period / 2\ :sup:`divisor`.

	* ``divisor``: The new Health Server publish period divisor.


``mesh models health period-set-unack <divisor>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Set the current Health Server publish period divisor. When a fault is detected, the Health Server will start publishing is fault status with a reduced interval. The reduced interval is determined by the Health Server publish period divisor: Fault publish period = Publish period / 2\ :sup:`divisor`.

	* ``divisor``: The new Health Server publish period divisor.


``mesh models health attention-get``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the current Health Server attention state.


``mesh models health attention-set <timer>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Enable the Health Server attention state for some time.

	* ``timer``: Duration of the attention state, in seconds (``0`` to ``255``)


``mesh models health attention-set-unack <timer>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Enable the Health Server attention state for some time without requesting a response.

	* ``timer``: Duration of the attention state, in seconds (``0`` to ``255``)


Binary Large Object (BLOB) Transfer Client model
------------------------------------------------

The :ref:`bluetooth_mesh_blob_cli` can be added to the mesh shell by enabling the :kconfig:option:`CONFIG_BT_MESH_BLOB_CLI` option, and disabling the :kconfig:option:`CONFIG_BT_MESH_DFU_CLI` option.

``mesh models blob cli target <addr>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Add a Target node for the next BLOB transfer.

	* ``addr``: Unicast address of the Target node's BLOB Transfer Server model.


``mesh models blob cli bounds [<group>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the total boundary parameters of all Target nodes.

	* ``group``: Optional group address to use when communicating with Target nodes. If omitted, the BLOB Transfer Client will address each Target node individually.


``mesh models blob cli tx <id> <size> <block size log> <chunk size> [<group> [<mode: push, pull>]]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Perform a BLOB transfer to Target nodes. The BLOB Transfer Client will send a dummy BLOB to all Target nodes, then post a message when the transfer is completed. Note that all Target nodes must first be configured to receive the transfer using the ``mesh models blob srv rx`` command.

	* ``id``: 64-bit BLOB transfer ID.
	* ``size``: Size of the BLOB in bytes.
	* ``block size log`` Logarithmic representation of the BLOB's block size. The final block size will be ``1 << block size log`` bytes.
	* ``chunk size``: Chunk size in bytes.
	* ``group``: Optional group address to use when communicating with Target nodes. If omitted or set to 0, the BLOB Transfer Client will address each Target node individually.
	* ``mode``: BLOB transfer mode to use. Must be either ``push`` (Push BLOB Transfer Mode) or ``pull`` (Pull BLOB Transfer Mode). If omitted, ``push`` will be used by default.


``mesh models blob cli tx-cancel``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Cancel an ongoing BLOB transfer.

``mesh models blob cli tx-get [group]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Determine the progress of a previously running BLOB transfer. Can be used when not performing a BLOB transfer.

	* ``group``: Optional group address to use when communicating with Target nodes. If omitted or set to 0, the BLOB Transfer Client will address each Target node individually.


``mesh models blob cli tx-suspend``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Suspend the ongoing BLOB transfer.


``mesh models blob cli tx-resume``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Resume the suspended BLOB transfer.

``mesh models blob cli instance-set <elem_idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Use the BLOB Transfer Client model instance on the specified element when using the other BLOB Transfer Client model commands.

	* ``elem_idx``: The element on which to find the BLOB Transfer Client model instance to use.

``mesh models blob cli instance-get-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of all BLOB Transfer Client model instances on the node.


BLOB Transfer Server model
--------------------------

The :ref:`bluetooth_mesh_blob_srv` can be added to the mesh shell by enabling the :kconfig:option:`CONFIG_BT_MESH_BLOB_SRV` option. The BLOB Transfer Server model is capable of receiving any BLOB data, but the implementation in the mesh shell will discard the incoming data.


``mesh models blob srv rx <id> [<timeout base>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Prepare to receive a BLOB transfer.

	* ``id``: 64-bit BLOB transfer ID to receive.
	* ``timeout base``: Optional additional time to wait for client messages, in 10-second increments.


``mesh models blob srv rx-cancel``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Cancel an ongoing BLOB transfer.

``mesh models blob srv instance-set <elem_idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Use the BLOB Transfer Server model instance on the specified element when using the other BLOB Transfer Server model commands.

	* ``elem_idx``: The element on which to find the BLOB Transfer Server model instance to use.

``mesh models blob srv instance-get-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of all BLOB Transfer Server model instances on the node.


Firmware Update Client model
----------------------------

The Firmware Update Client model can be added to the mesh shell by enabling configuration options :kconfig:option:`CONFIG_BT_MESH_BLOB_CLI` and :kconfig:option:`CONFIG_BT_MESH_DFU_CLI`. The Firmware Update Client demonstrates the firmware update Distributor role by transferring a dummy firmware update to a set of Target nodes.


``mesh models dfu slot add <size> [<fwid> [<metadata> [<uri>]]]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Add a virtual DFU image slot that can be transferred as a DFU image. The image slot will be assigned an image slot index, which is printed as a response, and can be used to reference the slot in other commands. To update the image slot, remove it using the ``mesh models dfu slot del`` shell command and then add it again.

	* ``size``: DFU image slot size in bytes.
	* ``fwid``: Optional firmware ID, formatted as a hexstring.
	* ``metadata``: Optional firmware metadata, formatted as a hexstring.
	* ``uri``: Optional URI for the firmware.


``mesh models dfu slot del <slot idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Delete the DFU image slot at the given index.

	* ``slot idx``: Index of the slot to delete.


``mesh models dfu slot get <slot-idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get all available information about a DFU image slot.

	* ``slot idx``: Index of the slot to get.


``mesh models dfu cli target <addr> <img idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Add a Target node.

	* ``addr``: Unicast address of the Target node.
	* ``img idx``: Image index to address on the Target node.


``mesh models dfu cli target-state``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Check the DFU Target state of the device at the configured destination address.


``mesh models dfu cli target-imgs [<max count>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of DFU images on the device at the configured destination address.

	* ``max count``: Optional maximum number of images to return. If omitted, there's no limit on the number of returned images.


``mesh models dfu cli target-check <slot idx> <target img idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Check whether the device at the configured destination address will accept a DFU transfer from the given DFU image slot to the Target node's DFU image at the given index, and what the effect would be.

	* ``slot idx``: Index of the local DFU image slot to check.
	* ``target img idx``: Index of the Target node's DFU image to check.


``mesh models dfu cli send <slot idx> [<group>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Start a DFU transfer to all added Target nodes.

	* ``slot idx``: Index of the local DFU image slot to send.
	* ``group``: Optional group address to use when communicating with the Target nodes. If omitted, the Firmware Update Client will address each Target node individually.


``mesh models dfu cli apply``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Apply the most recent DFU transfer on all Target nodes. Can only be called after a DFU transfer is completed.


``mesh models dfu cli confirm``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Confirm that the most recent DFU transfer was successfully applied on all Target nodes. Can only be called after a DFU transfer is completed and applied.


``mesh models dfu cli progress``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Check the progress of the current transfer.


``mesh models dfu cli suspend``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Suspend the ongoing DFU transfer.


``mesh models dfu cli resume``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Resume the suspended DFU transfer.

``mesh models dfu srv progress``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Check the progress of the current transfer.

``mesh models dfu cli instance-set <elem_idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Use the Firmware Update Client model instance on the specified element when using the other Firmware Update Client model commands.

	* ``elem_idx``: The element on which to find the Firmware Update Client model instance to use.

``mesh models dfu cli instance-get-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of all Firmware Update Client model instances on the node.


Firmware Update Server model
----------------------------

The Firmware Update Server model can be added to the mesh shell by enabling configuration options :kconfig:option:`CONFIG_BT_MESH_BLOB_SRV` and :kconfig:option:`CONFIG_BT_MESH_DFU_SRV`. The Firmware Update Server demonstrates the firmware update Target role by accepting any firmware update. The mesh shell Firmware Update Server will discard the incoming firmware data, but otherwise behave as a proper firmware update Target node.


``mesh models dfu srv applied``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Mark the most recent DFU transfer as applied. Can only be called after a DFU transfer is completed, and the Distributor has requested that the transfer is applied.

	As the mesh shell Firmware Update Server doesn't actually apply the incoming firmware image, this command can be used to emulate an applied status, to notify the Distributor that the transfer was successful.


``mesh models dfu srv progress``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Check the progress of the current transfer.

``mesh models dfu srv rx-cancel``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Cancel incoming DFU transfer.

``mesh models dfu srv instance-set <elem_idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Use the Firmware Update Server model instance on the specified element when using the other Firmware Update Server model commands.

	* ``elem_idx``: The element on which to find the Firmware Update Server model instance to use.

``mesh models dfu srv instance-get-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of all Firmware Update Server model instances on the node.


.. _bluetooth_mesh_shell_dfd_server:

Firmware Distribution Server model
----------------------------------

The Firmware Distribution Server model commands can be added to the mesh shell by enabling the :kconfig:option:`CONFIG_BT_MESH_DFD_SRV` configuration option.
The shell commands for this model mirror the messages sent to the server by a Firmware Distribution Client model.
To use these commands, a Firmware Distribution Server must be instantiated by the application.

``mesh models dfd receivers-add <addr>,<fw_idx>[;<addr>,<fw_idx>]...``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Add receivers to the Firmware Distribution Server.
	Supply receivers as a list of comma-separated addr,fw_idx pairs, separated by semicolons, for example, ``0x0001,0;0x0002,0;0x0004,1``.
	Do not use spaces in the receiver list.
	Repeated calls to this command will continue populating the receivers list until ``mesh models dfd receivers-delete-all`` is called.

	* ``addr``: Address of the receiving node(s).
	* ``fw_idx``: Index of the firmware slot to send to ``addr``.

``mesh models dfd receivers-delete-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Delete all receivers from the server.

``mesh models dfd receivers-get <first> <count>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of info about firmware receivers.

	* ``first``: Index of the first receiver to get from the receiver list.
	* ``count``: The number of recievers for which to get info.

``mesh models dfd capabilities-get``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the capabilities of the server.

``mesh models dfd get``
^^^^^^^^^^^^^^^^^^^^^^^

	Get information about the current distribution state, phase and the transfer parameters.

``mesh models dfd start <app_idx> <slot_idx> [<group> [<policy_apply> [<ttl> [<timeout_base> [<xfer_mode>]]]]]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Start the firmware distribution.

	* ``app_idx``: Application index to use for sending. The common application key should be bound to the Firmware Update and BLOB Transfer models on the Distributor and Target nodes.
	* ``slot_idx``: Index of the local image slot to send.
	* ``group``: Optional group address to use when communicating with the Target nodes. If omitted, the Firmware Distribution Server will address each Target node individually. To keep addressing each Target node individually while changing other arguments, set this argument value to 0.
	* ``policy_apply``: Optional field that corresponds to the update policy. Setting this to ``true`` will make the Firmware Distribution Server apply the image immediately after the transfer is completed.
	* ``ttl``: Optional. TTL value to use when sending. Defaults to configured default TTL.
	* ``timeout_base``: Optional additional value used to calculate timeout values in the firmware distribution process. See :ref:`bluetooth_mesh_blob_timeout` for information about how ``timeout_base`` is used to calculate the transfer timeout. Defaults to 0.
	* ``xfer_mode``: Optional BLOB transfer mode. 1 = Push mode (Push BLOB Transfer Mode), 2 = Pull mode (Pull BLOB Transfer Mode). Defaults to Push mode.

``mesh models dfd suspend``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Suspends the ongoing distribution.

``mesh models dfd cancel``
^^^^^^^^^^^^^^^^^^^^^^^^^^

	Cancel the ongoing distribution.

``mesh models dfd apply``
^^^^^^^^^^^^^^^^^^^^^^^^^

	Apply the distributed firmware.

``mesh models dfd fw-get <fwid>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get information about the firmware image uploaded to the server.

	* ``fwid``: Firmware ID of the image to get.

``mesh models dfd fw-get-by-idx <idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get information about the firmware image uploaded to the server in a specific slot.

	* ``idx``: Index of the slot to get the image from.

``mesh models dfd fw-delete <fwid>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Delete a firmware image from the server.

	* ``fwid``: Firmware ID of the image to delete.

``mesh models dfd fw-delete-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Delete all firmware images from the server.

``mesh models dfd instance-set <elem_idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Use the Firmware Distribution Server model instance on the specified element when using the other Firmware Distribution Server model commands.

	* ``elem_idx``: The element on which to find the Firmware Distribution Server model instance to use.

``mesh models dfd instance-get-all``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get a list of all Firmware Distribution Server model instances on the node.


.. _bluetooth_mesh_shell_dfu_metadata:

DFU metadata
------------

The DFU metadata commands allow generating metadata that can be used by a Target node to check the firmware before accepting it. The commands are enabled through the :kconfig:option:`CONFIG_BT_MESH_DFU_METADATA` configuration option.

``mesh models dfu metadata comp-clear``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Clear the stored composition data to be used for the Target node.

``mesh models dfu metadata comp-add <cid> <pid> <vid> <crpl> <features>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Create a header of the Composition Data Page 0.

	* ``cid``: Company identifier assigned by Bluetooth SIG.
	* ``pid``: Vendor-assigned product identifier.
	* ``vid``: Vendor-assigned version identifier.
	* ``crpl``: The size of the replay protection list.
	* ``features``: Features supported by the node in bit field format:
		* ``0``: Relay.
		* ``1``: Proxy.
		* ``2``: Friend.
		* ``3``: Low Power.

``mesh models dfu metadata comp-elem-add <loc> <nums> <numv> {<sig model id>|<vnd company id> <vnd model id>}...``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Add element description of the Target node.

	* ``loc``: Element location.
	* ``nums``: Number of SIG models instantiated on the element.
	* ``numv``: Number of vendor models instantiated on the element.
	* ``sig model id``: SIG Model ID.
	* ``vnd company id``: Vendor model company identifier.
	* ``vnd model id``: Vendor model identifier.

``mesh models dfu metadata comp-hash-get [<128-bit key>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Generate a hash of the stored Composition Data to be used in metadata.

	* ``128-bit key``: Optional 128-bit key to be used to generate the hash.

``mesh models dfu metadata encode <major> <minor> <rev> <build_num> <size> <core type> <hash> <elems> [<user data>]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Encode metadata for the DFU.

	* ``major``: Major version of the firmware.
	* ``minor``: Minor version of the firmware.
	* ``rev``: Revision number of the firmware.
	* ``build_num``: Build number.
	* ``size``: Size of the signed bin file.
	* ``core type``: New firmware core type in bit field format:
		* ``0``: Application core.
		* ``1``: Network core.
		* ``2``: Applications specific BLOB.
	* ``hash``: Hash of the composition data generated using ``mesh models dfu metadata comp-hash-get`` command.
	* ``elems``: Number of elements on the new firmware.
	* ``user data``: User data supplied with the metadata.


Segmentation and Reassembly (SAR) Configuration Client
------------------------------------------------------

The SAR Configuration client is an optional mesh model that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_SAR_CFG_CLI` configuration option. The SAR Configuration Client model is used to support the functionality of configuring the behavior of the lower transport layer of a node that supports the SAR Configuration Server model.


``mesh models sar tx-get``
^^^^^^^^^^^^^^^^^^^^^^^^^^

	Send SAR Configuration Transmitter Get message.

``mesh models sar tx-set <7 configuration values>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Send SAR Configuration Transmitter Set message.

	* ``seg_int_step``: SAR Segment Interval Step state.
	* ``unicast_retrans_count``: SAR Unicast Retransmissions Count state.
	* ``unicast_retrans_without_prog_count``: SAR Unicast Retransmissions Without Progress Count state.
	* ``unicast_retrans_int_step``: SAR Unicast Retransmissions Interval Step state.
	* ``unicast_retrans_int_inc``: SAR Unicast Retransmissions Interval Increment state.
	* ``multicast_retrans_count``: SAR Multicast Retransmissions Count state.
	* ``multicast_retrans_int``: SAR Multicast Retransmissions Interval state.

``mesh models sar rx-get``
^^^^^^^^^^^^^^^^^^^^^^^^^^

	Send SAR Configuration Receiver Get message.

``mesh models sar rx-set <5 configuration values>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Send SAR Configuration Receiver Set message.

	* ``seg_thresh``: SAR Segments Threshold state.
	* ``ack_delay_inc``: SAR Acknowledgment Delay Increment state.
	* ``discard_timeout``: SAR Discard Timeout state.
	* ``rx_seg_int_step``: SAR Receiver Segment Interval Step state.
	* ``ack_retrans_count``: SAR Acknowledgment Retransmissions Count state.


Private Beacon Client
---------------------

The Private Beacon Client model is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_PRIV_BEACON_CLI` configuration option.

``mesh models prb priv-beacon-get``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the target's Private Beacon state. Possible values:

		* ``0x00``: The node doesn't broadcast Private beacons.
		* ``0x01``: The node broadcasts Private beacons.

``mesh models prb priv-beacon-set <enable> <rand_interval>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Set the target's Private Beacon state.

	* ``enable``: Control Private Beacon state.
	* ``rand_interval``: Random refresh interval (in 10-second steps), or 0 to keep current value.

``mesh models prb priv-gatt-proxy-get``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the target's Private GATT Proxy state. Possible values:

		* ``0x00``: The Private Proxy functionality is supported, but disabled.
		* ``0x01``: The Private Proxy functionality is enabled.
		* ``0x02``: The Private Proxy functionality is not supported.

``mesh models prb priv-gatt-proxy-set <state>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Set the target's Private GATT Proxy state.

	* ``state``: New Private GATT Proxy value:

		* ``0x00``: Disable the Private Proxy functionality.
		* ``0x01``: Enable the Private Proxy functionality.

``mesh models prb priv-node-id-get <net_idx>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Get the target's Private Node Identity state. Possible values:

		* ``0x00``: The node does not adverstise with the Private Node Identity.
		* ``0x01``: The node advertises with the Private Node Identity.
		* ``0x02``: The node doesn't support advertising with the Private Node Identity.

	* ``net_idx``: Network index to get the Private Node Identity state of.

``mesh models prb priv-node-id-set <net_idx> <state>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Set the target's Private Node Identity state.

	* ``net_idx``: Network index to set the Private Node Identity state of.
	* ``state``: New Private Node Identity value:
		* ``0x00``: Stop advertising with the Private Node Identity.
		* ``0x01``: Start advertising with the Private Node Identity.


Opcodes Aggregator Client
-------------------------

The Opcodes Aggregator client is an optional Bluetooth mesh model that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_OP_AGG_CLI` configuration option. The Opcodes Aggregator Client model is used to support the functionality of dispatching a sequence of access layer messages to nodes supporting the Opcodes Aggregator Server model.

``mesh models opagg seq-start <elem_addr>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Start the Opcodes Aggregator Sequence message. This command initiates the context for aggregating messages and sets the destination address for next shell commands to ``elem_addr``.

	* ``elem_addr``: Element address that will process the aggregated opcodes.

``mesh models opagg seq-send``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Send the Opcodes Aggregator Sequence message. This command completes the procedure, sends the aggregated sequence message to the target node and clears the context.

``mesh models opagg seq-abort``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	Abort the Opcodes Aggregator Sequence message. This command clears the Opcodes Aggregator Client context.


Configuration database
======================

The Configuration database is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_CDB` configuration option. The Configuration database is only available on provisioner devices, and allows them to store all information about the mesh network. To avoid conflicts, there should only be one mesh node in the network with the Configuration database enabled. This node is the Configurator, and is responsible for adding new nodes to the network and configuring them.

``mesh cdb create [NetKey]``
----------------------------

	Create a Configuration database.

	* ``NetKey``: Optional network key value of the primary network key (NetKeyIndex=0). Defaults to the default key value if omitted.


``mesh cdb clear``
------------------

	Clear all data from the Configuration database.


``mesh cdb show``
-----------------

	Show all data in the Configuration database.


``mesh cdb node-add <UUID> <addr> <num-elem> <NetKeyIdx> [DevKey]``
-------------------------------------------------------------------

	Manually add a mesh node to the configuration database. Note that devices provisioned with ``mesh provision`` and ``mesh provision-adv`` will be added automatically if the Configuration Database is enabled and created.

	* ``UUID``: 128-bit hexadecimal UUID of the node. Any omitted bytes will be zero.
	* ``addr``: Unicast address of the node, or 0 to automatically choose the lowest available address.
	* ``num-elem``: Number of elements on the node.
	* ``NetKeyIdx``: The network key the node was provisioned with.
	* ``DevKey``: Optional 128-bit device key value for the device. If omitted, a random value will be generated.


``mesh cdb node-del <addr>``
----------------------------

	Delete a mesh node from the Configuration database. If possible, the node should be reset with ``mesh reset`` before it is deleted from the Configuration database, to avoid unexpected behavior and uncontrolled access to the network.

	* ``addr`` Address of the node to delete.


``mesh cdb subnet-add <NeyKeyIdx> [<NetKey>]``
----------------------------------------------

	Add a network key to the Configuration database. The network key can later be passed to mesh nodes in the network. Note that adding a key to the Configuration database does not automatically add it to the local node's list of known network keys.

	* ``NetKeyIdx``: Key index of the network key to add.
	* ``NetKey``: Optional 128-bit network key value. Any missing bytes will be zero. If omitted, a random value will be generated.


``mesh cdb subnet-del <NetKeyIdx>``
-----------------------------------

	Delete a network key from the Configuration database.

	* ``NetKeyIdx``: Key index of the network key to delete.


``mesh cdb app-key-add <NetKeyIdx> <AppKeyIdx> [<AppKey>]``
-----------------------------------------------------------

	Add an application key to the Configuration database. The application key can later be passed to mesh nodes in the network. Note that adding a key to the Configuration database does not automatically add it to the local node's list of known application keys.

	* ``NetKeyIdx``: Network key index the application key is bound to.
	* ``AppKeyIdx``: Key index of the application key to add.
	* ``AppKey``: Optional 128-bit application key value. Any missing bytes will be zero. If omitted, a random value will be generated.


``mesh cdb app-key-del <AppKeyIdx>``
------------------------------------

	Delete an application key from the Configuration database.

	* ``AppKeyIdx``: Key index of the application key to delete.


On-Demand Private GATT Proxy Client
-----------------------------------

The On-Demand Private GATT Proxy Client model is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_OD_PRIV_PROXY_CLI` configuration option.

``mesh models od_priv_proxy od-priv-gatt-proxy [duration]``
-----------------------------------------------------------

	Set the On-Demand Private GATT Proxy state on active target, or fetch the value of this state from it. This feature can be enabled through the :kconfig:option:`CONFIG_BT_MESH_OD_PRIV_PROXY_CLI` configuration option.

	* ``duration``: If given, set the state of On-Demand Private GATT Proxy to this value. Fetch this value otherwise.


Solicitation PDU RPL Client
---------------------------

The Solicitation PDU RPL Client model is an optional mesh subsystem that can be enabled through the :kconfig:option:`CONFIG_BT_MESH_SOL_PDU_RPL_CLI` configuration option.

``mesh models sol_pdu_rpl sol-pdu-rpl-clear <range_start> <acked> [range_len]``
-------------------------------------------------------------------------------

	Clear active target's solicitation replay protection list (SRPL) in given range of solicitation source (SSRC) addresses. This feature can be enabled through the :kconfig:option:`CONFIG_BT_MESH_SOL_PDU_RPL_CLI` configuration option.

	* ``range_start``: Start address of the SSRC range.
	* ``acked``: This argument decides on whether an acknowledged or unacknowledged message will be sent.
	* ``range_len``: Range length for the SSRC addresses to be cleared from the solicitiation RPL list. This parameter is optional; if absent, only a single SSRC address will be cleared.
