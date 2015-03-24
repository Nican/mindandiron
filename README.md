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
