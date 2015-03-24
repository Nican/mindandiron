mindandiron
===========

-y automake
-y libzmqpp-dev
-y libv4l-dev
-y libopencv-dev
-y libeigen3-dev
-y libbox2d-dev
-y libvtk5-qt4-dev
-y openssh-server
-y subversion
-y htop

sudo add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all-dev

Follow instructions here: https://github.com/OpenKinect/libfreenect2#ubuntu-1404-perhaps-earlier
-y libturbojpeg libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev

How to set up VPN
sudo add-apt-repository -y ppa:pritunl
sudo apt-get update
sudo apt-get install pritunl-client-gtk
	Get server key somehow? We used Henrique's
Open Pritunl program
Import profile, connect over Main

Installing Gazebo: 
http://gazebosim.org/tutorials?tut=install
wget -O /tmp/gazebo4_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo4_install.sh; sudo sh /tmp/gazebo4_install.sh


To install MsgPack:
git clone https://github.com/msgpack/msgpack-c.git
cd msgpack-c
./bootstrap
./configure CXXFLAGS="-std=c++11"
make
sudo make install
