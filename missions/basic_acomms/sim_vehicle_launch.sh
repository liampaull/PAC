#! /bin/bash

RUNTYPE=simulation
LOCATION=MIT
MISSION_ROOT=`pwd`

SCRIPT_PATH=`readlink -f $0`
SCRIPT_DIRNAME=`dirname ${SCRIPT_PATH}`
MOOS_IVP_LIAM_DIR=`readlink -f ${SCRIPT_DIRNAME}/..`

echo "//---------------------------------------------------------------------"
echo "// Building & Launching the AUV Simulation"

pushd ${SCRIPT_DIRNAME} &>/dev/null
source scripts/vehicles_launch.sh
popd &>/dev/null

echo -e
echo "// End of line."
echo "//---------------------------------------------------------------------"


