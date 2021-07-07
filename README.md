# PGO-INSAM

### Overview

A Pose Graph Optimization version for Integrated Navitation System(GNSS/INS) based on GTSAM

### Dependency

- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)
  ```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.2/
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
  sudo make install -j8
  ```
### Usage
```
mkdir INSAM
cd INSAM
mkdir src
cd src
catkin_init_workspace
git clone https://github.com/BohemianRhapsodyz/PGO-INSAM.git
cd ..
catkin_make
source devel/setup.bash
roslaunch stim300 od_sins_realtime.launch
rosbag play example.bag
```
check rostopic /ground_truth and /nav to compare the result.
![Alt text](https://github.com/BohemianRhapsodyz/PGO-INSAM/blob/main/pic/screenshot.png)
