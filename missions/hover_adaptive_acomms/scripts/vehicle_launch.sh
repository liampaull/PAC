#! /bin/bash

# Pass command line flags to nsplug.
FLAGS="${RUNTYPE} $@ "


rm -f ${VNAME}.bhv
echo "      * removed the existing ${VNAME} BHV file"

nsplug vehicle_bhv.meta ${VNAME}.bhv -f ${FLAGS} VEHICLE_NAME=${VNAME} MISSION_ROOT=${MISSION_ROOT}
echo "      * built new vehicle BHV file from plugs"


rm -f ${VNAME}.moos
echo "      * removed the existing ${VNAME} MOOS file"

nsplug vehicle_moos.meta ${VNAME}.moos -f ${FLAGS} VEHICLE_NAME=${VNAME} MISSION_ROOT=${MISSION_ROOT} RUNTYPE=${RUNTYPE}
echo "      * built new vehicle MOOS file from plugs"

if [ ${JUST_BUILD} = "yes" ] ; then
    exit 0
fi

# Launch the MOOS file.
pAntler ${VNAME}.moos >& /dev/null &
echo "      * launched the new ${VNAME} MOOS file"

 
