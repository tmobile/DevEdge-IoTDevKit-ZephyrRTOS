#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# Test Private Beacon advertising during IV Update procedure
conf=prj_mesh1d1_conf
RunTest mesh_priv_beacon_on_iv_update \
  beacon_rx_on_iv_update \
  beacon_tx_priv_on_iv_update \
  -- -argstest rand-int=1
