#!/usr/bin/env bash
# Copyright 2021 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# Note:
# Tests must be added in pairs and in sequence.
# First test: saves data; second test: verifies it.

# Provision, configure and reset a device
overlay=overlay_pst_conf
RunTest mesh_pst_repr persistence_reprovisioning_device \
	persistence_reprovisioning_provisioner -- -argstest clear-settings=1

# Repeat the test
overlay=overlay_pst_conf
RunTest mesh_pst_repr persistence_reprovisioning_device \
	persistence_reprovisioning_provisioner

conf=prj_mesh1d1_conf
overlay=overlay_pst_conf
RunTest mesh_pst_repr_1d1 persistence_reprovisioning_device \
	persistence_reprovisioning_provisioner -- -argstest clear-settings=1

conf=prj_mesh1d1_conf
overlay=overlay_pst_conf
RunTest mesh_pst_repr_1d1 persistence_reprovisioning_device \
	persistence_reprovisioning_provisioner
