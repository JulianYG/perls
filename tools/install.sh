#!/bin/bash

# This script is meant to document the installation of 
# all packages and libraries that we install on this machine.

### IMPORTANT: All libraries and packages installed on this machine must 
###            have a corresponding entry here. The ones that are cloned from
###            a repo should go in ~/installed_libraries/

### Install essentials.
sudo apt-get install build-essential
sudo apt-get install compizconfig-settings-manager compiz-plugins-extra

### Install NVIDIA drivers.
sudo apt-get install nvidia-375

### Install emacs
sudo apt-get install emacs

### Install yum
sudo apt-get install yum

### Install virtualenv
sudo apt-get install python-virtualenv
### Install Sublime Text 3
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt update && sudo apt install sublime-text

# Note: You might want to edit preferences to convert all tabs to spaces too.


### Install Google Chrome
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'
sudo apt-get update 
sudo apt-get install google-chrome-stable

### Install git
sudo apt install git


### Install ssh.
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install openssh-server

sudo nano /etc/ssh/sshd_config
# NOTE: add MaxAuthTries 3 under Port, last line AllowUsers cvgl_ros
sudo service ssh restart

### Install cs225a repository.

# Note: The installation expects RBDL, Chai, and, SAI2Simulation to be unzipped and 
#       located at ~/installed_libraries/rbdl, ~/installed_libraries/chai3d, and
#       ~/installed_libraries/sai2-simulation
cd ~/installed_libraries
sudo apt-get install cmake # cmake
sudo apt-get install libeigen3-dev 
sudo apt-get install libtinyxml2-dev
sudo apt-get install libjsoncpp-dev
sudo apt-get install libhiredis-dev
sudo apt-get install libglfw3-dev
sudo apt-get install xorg-dev
sudo apt-get install freeglut3-dev
sudo apt-get install libasound2-dev
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install redis-server
sudo apt-get install libyaml-cpp-dev
sudo apt-get install libv4l-dev

# update cmake
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:george-edison55/cmake-3.x
sudo apt-get update
sudo apt-get upgrade

# copy eigen cmake over so it can be found...
sudo cp /usr/share/cmake-2.8/Modules/FindEigen3.cmake /usr/share/cmake-3.2/Modules/

# install libglfw3 for realsies
echo "deb http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
echo "deb-src http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
sudo apt-get update
sudo apt-get install libglfw3 libglfw3-dev
sudo apt-get install libopenscenegraph-dev

sudo apt-get purge libglfw3-dev
git clone https://github.com/glfw/glfw.git
cd glfw/
mkdir build
cd build
cmake -D BUILD_SHARED_LIBS=ON ..
sudo make install
sudo ldconfig

cd rbdl
mkdir build 
cd build
cmake -DRBDL_BUILD_ADDON_URDFREADER=On -DRBDL_USE_ROS_URDF_LIBRARY=OFF ..
make -j4
sudo make install
cd ../..

cd chai3d
mkdir build
cd build
cmake ..
make -j4
cd ../..

cd sai2-simulation
mkdir build
cd build
cmake ..
make -j4
cd ../..

git clone https://github.com/manips-sai/sai2-common.git
cd sai2-common
mkdir build
cd build
cmake ..
make -j4
cd ../..

git clone https://github.com/tmigimatsu/cs225a-dist.git
cd cs225a-dist
rm -rf sai2-common
cp -rf ../sai2-common .
sh make.sh 

git checkout devel

# install boost
sudo apt-get install libboost-all-dev
sudo apt-get install cmake libboost-dev libboost-program-options-dev libboost-system-dev libboost-thread-dev

# install doxygen
sudo apt-get install git doxygen

# install NatNetLinux
cd external
cd NatNetLinux.git
git submodule update --init
cd ../..

# cd ..
# git clone https://github.com/rocketman768/NatNetLinux.git
# cd NatNetLinux
# mkdir build
# cd build
# cmake ..
# make
# sudo make install
# cd ../../cs225a-dist

# update gcc
sudo apt-get install python-software-properties
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 50
sudo apt-get install g++-4.9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 50

### NOTE: In perls/redis/RedisClient.cpp comment out line 39, "tcp" is a problem

sh make.sh 

### Install pybullet
cd ~/
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
cd build_cmake
nano CMakeCache.txt
# Note: Set the following line:
# PYTHON_LIBRARY:FILEPATH=/usr/lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so
cd ..
./build_cmake_pybullet_double.sh

# Important: Transfer over rethink_ee_description, table_square, sawyer_arm, cube_green

### Important: Do this.
# Also, open ~/.bashrc
# Go to bottom, and change the lines to:
# PYTHON_LIBRARY=/usr/lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so
# # add pybullet to python path
# export PYTHONPATH="${PYTHONPATH}:/home/cvgl_ros/bullet3/build_cmake/examples/pybullet"$

### Install perls library.
cd ~/installed_libraries
git clone https://github.com/JulianYG/perls.git
cd perls
git checkout deploy
cd ..
ln -s ~/bullet3/data perls/data # soft link files from bullet into perls
sudo pip install gym 
sudo ln -s ~/installed_libraries/perls /usr/local/lib/python2.7/dist-packages/perls

# Install pynput, a dependency for perls.
sudo pip install pynput

# Install six, a dependency for pynput
sudo pip install --upgrade six

# Install openvr, a dependency for perls.
sudo apt-get install libsdl2-dev
sudo pip install openvr

### Important: open /usr/local/lib/python2.7/dist-packages/openvr/__init__.py
#   Delete lines 37-38, replace with the following:
# # Load library
# if platform.system() == 'Windows':
#     # Add current directory to PATH, so we can load the DLL from right here.
#     os.environ['PATH'] += os.pathsep + os.path.dirname(__file__)
# else:
#     _openvr_lib_name = os.path.join(os.path.dirname(__file__), _openvr_lib_name)

# Install redis.
sudo pip install redis

# Install IPython
sudo pip install IPython

### Install all robot stuff.

# Consult the following document and follow the instructions there.
# https://docs.google.com/document/d/1TFPzeHz8cX4zU0iIxAR3wJV1JN4H-T5fpzOayP-qTu0/edit

sudo apt-get install ros-indigo-octomap-mapping ros-indigo-octomap-ros ros-indigo-octomap-rviz-plugins ros-indigo-octomap-server

### Install all kinect stuff
cd ~/installed_libraries
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

# If we are running 14.04 
cd depends; ./download_debs_trusty.sh
sudo dpkg -i debs/libusb*deb
sudo apt-get install libjpeg-turbo8-dev

# Install OpenCL
sudo apt-add-repository ppa:floe/beignet; sudo apt-get update; sudo apt-get install beignet-dev; sudo dpkg -i debs/ocl-icd*deb

# Install OpenNI2
sudo apt-add-repository ppa:deb-rob/ros-trusty && sudo apt-get update
sudo apt-get install libopenni2-dev

# build libfreenect2
cd ..
mkdir build && cd build
cmake .. -DENABLE_CXX11=ON
make -j8
sudo make install

## Remember to run this command for CMake based third-party application to find libfreenect2:
# cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2

# Setup udev rules:
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
# Test:
./bin/Protonect

## Install IAI Kinect2 
cd ~/ros_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2

rosdep install --from-paths ~/ros_ws/src/iai_kinect2 --ignore-src -r
cd ~/ros_ws
catkin_make -DCMAKE_BUILD_TYPE="Release" -Dfreenect2_DIR=/usr/local/lib/cmake/freenect2

## Install pylibfreenect2

# Install cython
sudo pip install cython
sudo pip install pylibfreenect2

# Make sure /usr/local/lib is added to LD_LIBRARY_PATH
# Also add these into PATH and LD_LIBRARY_PATH:
# /usr/local/cuda/bin
# /usr/local/cuda/lib64


# Test:
roslaunch kinect2_bridge kinect2_bridge.launch

### Install libraries for imitation learning.
sudo apt-get install python-pip python-dev python-virtualenv
cd ~/Desktop
mkdir imitation
cd imitation
virtualenv .env
source .env/bin/activate
git clone https://github.com/YunzhuLi/InfoGAIL.git
export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.12.1-cp27-none-linux_x86_64.whl
pip install --upgrade $TF_BINARY_URL
sudo apt-get install libblas-dev liblapack-dev
sudo apt-get install gfortran
pip install keras==1.2.2
sudo apt-get update
sudo apt-get install xautomation
pip install gym
# install opencv system-wide
sudo apt-get install python-opencv
# for some reason, virtualenv has trouble finding it, so copy over what we need
cp /usr/lib/python2.7/dist-packages/cv2.so ~/Desktop/imitation/.env/lib/python2.7/site-packages
# also make sure we can link to perls from virtualenv
ln -s ~/installed_libraries/perls ~/Desktop/imitation/.env/lib/python2.7/site-packages/perls
pip install -U pip
pip install IPython
pip install h5py
pip install matplotlib

pip install openvr

##  Install glfw for python
pip install glfw
pip install redis

### Important: open ~/Desktop/imitation/.env/lib/python2.7/site-packages/openvr/__init__.py
#   Delete lines 37-38, replace with the following:
# # Load library
# if platform.system() == 'Windows':
#     # Add current directory to PATH, so we can load the DLL from right here.
#     os.environ['PATH'] += os.pathsep + os.path.dirname(__file__)
# else:
#     _openvr_lib_name = os.path.join(os.path.dirname(__file__), _openvr_lib_name)

# used to start processes in new shells
sudo apt-get install screen 
cd ../..

### TODO: put instructions for modified InfoGAIL here... ###
### Works with Keras 1.2.2, and Tensorflow 1.3 ###

### Install TORCS and vision stuff for InfoGAIL. ###
sudo apt-get install libvorbis-dev
# then, follow instructions here:
# https://github.com/YunzhuLi/InfoGAIL

### Instructions for installing tensorflow with GPU support. ###

# Go to the following website, and download the deb file
# https://developer.nvidia.com/cuda-downloads
#

# run this with file name
sudo dpkg -i cuda-repo-<distro>_<version>_<architecture>.deb
sudo apt-get update
sudo apt-get install cuda

### Do the same with the patch! ###
sudo dpkg -i cuda-repo-<distro>_<version>_<architecture>.deb
sudo apt-get update
sudo apt-get upgrade

# add these lines to bashrc
export PATH=/usr/local/cuda-8.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64\
                         ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}


# download cuDNN here: (INSTALL v6)
# https://developer.nvidia.com/cudnn

# download and install the debian packages 
sudo dpkg -i libcudnn6_6.0.21-1+cuda8.0_amd64.deb 
sudo dpkg -i libcudnn6-dev_6.0.21-1+cuda8.0_amd64.deb
sudo dpkg -i libcudnn6-doc_6.0.21-1+cuda8.0_amd64.deb 

# finally, can create a virtualenv and install tensorflow gpu version via:
pip install --upgrade tensorflow-gpu

# now, install all previous libraries needed for imitation (see above)


### SteamVR installation. ###
sudo add-apt-repository ppa:mamarley/nvidia-dev
sudo apt-get update
sudo apt-get install nvidia-381

sudo apt-get install steam
sudo reboot 

sudo apt-get install libudev-dev
sudo apt-get install libsdl2-dev
sudo apt-get install 

# Go to Library->Tools in Steam and download SteamVR
# Go to properties, and select Beta
# Good to go!!!

### OpenRAVE installation ###
sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev  libgsm1-dev liblapack-dev libmpfr-dev libode-dev libogg-dev libopenscenegraph-dev libpcrecpp0 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev
sudo add-apt-repository ppa:openrave/release
sudo sh -c 'echo "deb-src http://ppa.launchpad.net/openrave/release/ubuntu `lsb_release -cs` main" >> /etc/apt/sources.list.d/openrave-release-`lsb_release -cs`.list'
sudo apt-get update
sudo apt-get install collada-dom-dev
sudo apt-get install libccd-dev

cd ~/installed_libraries
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
cd openrave
mkdir build
cd ~/installed_libraries

# Install m4
sudo apt-get install m4

# Install libccd
git clone https://github.com/danfis/libccd.git
mkdir build && cd build 
cmake -G "Unix Makefiles" ..
make -j4
sudo make install

# Install FCL
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
mkdir build 
cd build 
make -j8
sudo make install

cd ~/installed_libraries/openrave/build
cmake ..
# Before make, modify ln 1556, 1557 in build/CMakeCache.txt:
# FCL_fcl_LIBDIR:INTERNAL=/opt/ros/indigo/lib
# FCL_fcl_INCLUDEDIR:INTERNAL=/opt/ros/indigo/include/fcl
make -j8
sudo make install

# Install trajopt
git clone https://github.com/joschu/trajopt.git
mkdir build
cd build
cmake ..

# In CMakeCache.txt:
# //build point cloud processing stuff
# BUILD_CLOUDPROC:BOOL=ON
make -j8

# Set PYTHONPATH to point to trajopt

# Catkin build
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get install python-catkin-tools
# Track ik
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin

"""
source devel/setup.bash 

roscore   

roslaunch sawyer_description test_sawyer_description.launch 
rosrun safeNet test_collision_checking 
rosrun rviz rviz

"""

