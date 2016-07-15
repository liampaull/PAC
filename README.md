# PAC

Steps to get code up and running

1) Download and build MOOS from private fork 
 - git clone 
 - install dependencies (look in local readme)
 - ./build-moos.sh
 - ./build-ivp.sh

2) Download and build goby2
 - install libdccl http://libdccl.org/
 - don't forget to sudo apt-get install libproj-dev
 - 

3) hovergroup code
 - git clone 
 - follow instructions here: https://wikis.mit.edu/confluence/display/hovergroup/Software+Installation but *do not* install MOOS, ivp or Goby

4) dependencies for iSAM
 - http://people.csail.mit.edu/kaess/isam/doc/index.html

5a) LCM 1.0.0
 - https://github.com/lcm-proj/lcm/releases/tag/v1.0.0

5) dependencies for libbot2
 - https://github.com/libbot2/libbot2

6) ```cd make checkout
  make ```
  everything should build successfully

7) necessary paths:
 - 

8) ```cd missions
  ./launch_all.sh ```
