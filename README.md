mindandiron
===========

FIRST, install all of the things contained in setup/setup.sh (some of them are tools you can ignore, like Chrome and Sublime)

SECOND, clone the decawave decoder
git clone git@github.com:makmanalp/decawave-decoder.git
cd decawave-decoder; make
to run: ./kratos_decawave.sh

THIRD, clone the mindandiron repo
git clone git@github.com:Nican/mindandiron.git
cd mindandiron
cd gazebo
mkdir build
cd build
cmake ..
make

To get the AprilTags running, we compiled the code as described here: http://people.csail.mit.edu/kaess/apriltags/
We added ADD_DEFINITIONS(-fPIC) to the beginning of the AprilTags CMakeLists.txt file before 'make'ing
We moved the apriltags/build/lib/libapriltags.a file to mindandiron/gazebo/lib/, so we shouldn't need to get april tag stuff in the future, but this is in the record if we have to go through the process again
