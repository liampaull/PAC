#! /bin/bash

##############################################################################
# This script is called by 'runtime_launch.sh' and 'simulation_launch' in the
# ../missions directory, so please be mindful of your edits.
##############################################################################

# Pass command line flags to nsplug.
FLAGS="${RUNTYPE} ${LOCATION} $@ "


rm -f AUV_1.bhv
echo "      * removed the existing AUV_1 BHV file"

nsplug vehicle_bhv.meta AUV_1.bhv -f ${FLAGS} VEHICLE_NAME=AUV_1 MISSION_ROOT=${MISSION_ROOT}
echo "      * built new vehicle BHV file from plugs"

rm -f AUV_1.moos
echo "      * removed the existing AUV_1 MOOS file"

nsplug vehicle_moos.meta AUV_1.moos -f ${FLAGS} VEHICLE_NAME=AUV_1 MISSION_ROOT=${MISSION_ROOT}
echo "      * built new vehicle MOOS file from plugs"



# Launch the MOOS files.
pAntler AUV_1.moos >& /dev/null &
echo "      * launched the new AUV_1 MOOS file"
sleep 1
echo " * launching collections viewers"
collections_viewer -l udpm://239.255.76.56:7661 &
echo "launching loggers and spies" 
lcm-logger --lcm-url=udpm://239.255.76.56:7661 >& /dev/null &
lcm-spy --lcm-url=udpm://239.255.76.56:7661 &
sleep 1
 
