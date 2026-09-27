#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="12.8"
fi

export STAGING_ROOT="/data/safe_staging"

# BYD Song Plus DM-i: the DiPilot camera (MPC) faults with 'check multifunction
# video controller' when it sees the UDS/isotp firmware-query frames that card
# broadcasts on the powertrain bus during startup (relayed to bus 2 by the
# panda while still in elm327 mode). The platform is fixed via the vehicle
# selection UI (fixed_fingerprint) and CAN auto-match, so the query is
# unnecessary - skip it entirely.
export SKIP_FW_QUERY=1
