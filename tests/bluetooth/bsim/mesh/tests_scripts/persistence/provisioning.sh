#!/usr/bin/env bash
# Copyright 2021 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# Note:
# Tests must be added in pairs and in sequence.
# First test: saves data; second test: verifies it.

# SKIP=(persistence_provisioning_data_save)
overlay=overlay_pst_conf
RunTest mesh_pst_prov_data_check persistence_provisioning_data_save

# SKIP=(persistence_provisioning_data_load)
overlay=overlay_pst_conf
RunTest mesh_pst_prov_data_check persistence_provisioning_data_load

conf=prj_mesh1d1_conf
overlay=overlay_pst_conf
RunTest mesh_pst_prov_data_check_1d1 persistence_provisioning_data_save

conf=prj_mesh1d1_conf
overlay=overlay_pst_conf
RunTest mesh_pst_prov_data_check_1d1 persistence_provisioning_data_load
