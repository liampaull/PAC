#! /bin/bash

echo " * launching collections viewers"
collections_viewer -l udpm://239.255.76.56:7661 &
collections_viewer -l udpm://239.255.76.56:7662 &
collections_viewer -l udpm://239.255.76.56:7663 &
sleep 1
echo "launching loggers and spies" 
lcm-logger --lcm-url=udpm://239.255.76.56:7661 >& /dev/null &
lcm-logger --lcm-url=udpm://239.255.76.56:7662 >& /dev/null &
lcm-logger --lcm-url=udpm://239.255.76.56:7663 >& /dev/null &
lcm-spy --lcm-url=udpm://239.255.76.56:7661 &
lcm-spy --lcm-url=udpm://239.255.76.56:7662 &
lcm-spy --lcm-url=udpm://239.255.76.56:7663 &
