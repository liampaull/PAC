# Basic collection makefile distributed with pods version: 12.11.14

default_target: all

# get a list of subdirs to build by reading tobuild.txt
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to three parent directories for a 'build' directory.
#   Otherwise, use ./build.
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX=$(shell for pfx in ./ .. ../.. ../../..; do d=`pwd`/$$pfx/build; \
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
endif

# build quietly by default.  For a verbose build, run "make VERBOSE=1"
$(VERBOSE).SILENT:

all: 
	@[ -d $(BUILD_PREFIX) ] || mkdir -p $(BUILD_PREFIX) || exit 1
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done
	@# Place additional commands here if you have any

clean:
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
	@# Place additional commands here if you have any

update:
	svn up isam_pod
	svn up visualization
	svn up libbot2
	svn up MOOSApps
	svn up isam_coop
	cd missions
	svn up common_plugs

checkout:
	svn co https://svn.csail.mit.edu/marine/pods/isam_pod 
	svn co https://svn.csail.mit.edu/marine/pods/visualization
	svn co http://libbot.googlecode.com/svn/trunk libbot2
	svn co https://svn.csail.mit.edu/marine/projects/auvcslam/MOOSApps
	svn co https://svn.csail.mit.edu/marine/projects/auvcslam/isam_coop
	mkdir -p missions
	cd missions
	svn co https://svn.csail.mit.edu/marine/projects/auvcslam/missions/common_plugs