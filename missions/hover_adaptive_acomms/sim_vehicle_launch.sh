#!/bin/bash

TIME_WARP=1
JUST_BUILD="no"
VNAME="anonymous"
RUNTYPE=simulation
MISSION_ROOT=`pwd`


for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        printf "%s [SWITCHES] [time_warp]   \n" $0
        printf "  --just_make, -j    \n"
        printf "  --vname=VNAME      \n"
        printf "  --help, -h         \n"
        exit 0;
    elif [ "${ARGI:0:8}" = "--vname=" ] ; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ] ; then
        JUST_BUILD="yes"
    else
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done


SCRIPT_PATH=`readlink -f $0`
SCRIPT_DIRNAME=`dirname ${SCRIPT_PATH}`

echo "//---------------------------------------------------------------------"
echo "// Building & Launching the AUV Simulation"

pushd ${SCRIPT_DIRNAME} &>/dev/null
source scripts/vehicles_launch.sh
popd &>/dev/null

echo -e
echo "// End of line."
echo "//---------------------------------------------------------------------"
