#!/usr/bin/env bash
touch ~/.bash_aliases
echo -e "# ============================================================
# Added by Eric
alias getin='sudo apt-get install'
alias getreps='apt-cache search'
alias getrem='sudo apt-get autoremove'
alias getlocs='dpkg -l'
alias getupd='sudo apt-get update'
alias getupg='sudo apt-get upgrade'
alias getdupg='sudo apt-get dist-upgrade'
alias gits='git branch; echo "==================="; git status -s'
alias gitac='git add --all :/; git commit'
export PS1='\[\e[1;96m\]\W\[\e[m\] \[\e[1;92m\]\$\[\e[m\] \[\e[0m\]'

cdls() { cd "$@" && ls; }

cpplint() {
	current_dir=$(pwd)
	cd ~/Downloads
	./cpplint.py "$current_dir/$1"
	cd $current_dir
}
# ============================================================" > ~/.bash_aliases


# To begin the chrome install
# http://ubuntuportal.com/2014/04/how-to-install-google-chrome-web-browser-in-ubuntu-14-04-lts-trusty-tahr.html
cd ~/Downloads
if [ ! -f ~/Downloads/google-chrome-stable_current_amd64.deb ]; then
	wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
fi
sudo dpkg -i google-chrome-stable_current_amd64.deb
sudo apt-get update
sudo apt-get upgrade
sudo apt-get -f install
sudo dpkg -i google-chrome-stable_current_amd64.deb


# To get sublime
sudo add-apt-repository -y ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get install -y sublime-text-installer


# To get cpplint
# http://google-styleguide.googlecode.com/svn/trunk/cpplint/cpplint.py
cd ~/Downloads
if [ ! -f ~/Downloads/cpplint.py ]; then
	wget http://google-styleguide.googlecode.com/svn/trunk/cpplint/cpplint.py
fi


sudo apt-get install -y build-essential
sudo apt-get install -y automake
sudo apt-get install -y git
sudo apt-get install -y git-core
sudo apt-get install -y vim
sudo apt-get install -y screen
sudo apt-get install -y subversion
sudo apt-get install -y htop
sudo apt-get install -y openssh-server
sudo apt-get install -y libzmqpp-dev
sudo  apt-get install -y libv4l-dev
sudo apt-get install -y libopencv-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libbox2d-dev
sudo apt-get install -y libvtk5-qt4-dev


# To get PCL
# Instructions from here
sudo add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all-dev


# To get libfreenect - see installation instruction here, might go wrong in bash script
# https://github.com/OpenKinect/libfreenect2#ubuntu-1404-perhaps-earlier
sudo apt-get install -y libturbojpeg libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev
cd ~/Downloads
if [ ! -d "libfreenect2" ]; then
	git clone https://github.com/OpenKinect/libfreenect2.git
	cd libfreenect2/depends
	sh install_ubuntu.sh
	# Not sure if this line is necessary, it was in one of my installs
	sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0.0.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so
	cd ../examples/protonect
	cmake CMakeLists.txt
	make
	sudo make install
fi


# Installing Gazebo in 14.04 only
# http://gazebosim.org/tutorials?tut=install
if [ ! -f ~/Downloads/cpplint.py ]; then
	wget -O ~/Downloads/gazebo4_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo4_install.sh
	sudo sh ~/Downloads/gazebo4_install.sh
fi


# Install msgpack
cd ~/Downloads
if [ ! -d "msgpack-c" ]; then
	git clone https://github.com/msgpack/msgpack-c.git
	cd msgpack-c
	./bootstrap
	./configure CXXFLAGS="-std=c++11"
	make
	sudo make install
fi


# How to set up VPN
sudo add-apt-repository -y ppa:pritunl
sudo apt-get update
sudo apt-get install pritunl-client-gtk
#   Get server key somehow? We used Henrique's
#   Open Pritunl program
#   Import profile, connect over Main


# For communicating with USB
sudo adduser $USER dialout

echo "RESTART THE COMPUTER TO FINISH UPDATES"
