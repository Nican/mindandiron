# Setting up Gazebo

### Setup

Add to your .baserc/.zshrc file (make sure to change the paths as necessary:
```
export GAZEBO_MODEL_PATH=~/git/mindandiron:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=~/git/mindandiron/gazebo/build:$GAZEBO_PLUGIN_PATH
```

### Compile

Once inside of the `mindandiron/gazebo` directory, run:
```
mkdir build
cd build
cmake ..
make
```
From now on, it is only necessary to run `make` to run to re-compile.