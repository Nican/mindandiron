mindandiron
===========

-y libtoolize aclocal autoheader automake 
-y automake autoconf 
-y libtool gcc
libzmqpp-dev libv4l-dev
  Follow instructions here, but get libpcl-dev, not libpcl-all: http://www.pointclouds.org/downloads/linux.html
libpcl-dev
  Follow instructions here: https://github.com/OpenKinect/libfreenect2#ubuntu-1404-perhaps-earlier
-y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev automake
cmake libopencv-dev libeigen3-dev libv4l-dev
subversion
htop
libeigen3-dev
libbox2d-dev
libopencv-dev
libvtk5-qt4-dev
openssh-server

How to set up VPN
sudo add-apt-repository -y ppa:pritunl
sudo apt-get update
sudo apt-get install pritunl-client-gtk
	Get server key somehow? We used Henrique's
Open Pritunl program
Import profile, connect over Main


Mind &amp; Iron

http://gazebosim.org/tutorials?tut=install

Installing Gazebo: 
wget -O /tmp/gazebo4_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo4_install.sh; sudo sh /tmp/gazebo4_install.sh


To install MsgPack:
$ git clone https://github.com/msgpack/msgpack-c.git
$ cd msgpack-c
$ ./bootstrap
$ ./configure CXXFLAGS="-std=c++11"
$ make
$ sudo make install


sudo apt-get install libzmqpp-dev libv4l-dev
http://people.csail.mit.edu/kaess/apriltags/
