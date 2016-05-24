#! /bin/bash

##############################################################################
# This script is called by 'runtime_launch.sh' and 'simulation_launch' in the
# ../missions directory, so please be mindful of your edits.
##############################################################################

# Pass command line flags to nsplug.
FLAGS="${RUNTYPE} ${LOCATION} $@ "



rm -f shoreside.moos
#echo "      * removed the existing shoreside MOOS file"

nsplug shoreside.meta shoreside.moos -f ${FLAGS} MISSION_ROOT=${MISSION_ROOT}
#echo "      * built new shoreside MOOS file from plugs"


pAntler shoreside.moos >& /dev/null &
echo "      * launched the new shoreside MOOS file"

