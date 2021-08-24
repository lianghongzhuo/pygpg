# Python binding for Grasp Pose Generator (pyGPG)
[![DOI](https://zenodo.org/badge/399418729.svg)](https://zenodo.org/badge/latestdoi/399418729)

[Grasp Pose Generator](https://github.com/atenpas/gpg.git) is a cpp package that creat grasp candidates for 3D point clouds.
This package binding it with python.

## Install
- Install gpg
```
git clone https://github.com/atenpas/gpg.git
cd gpg && mkdir build && cd build && cmake .. && make
sudo make install
```
- Install Python 3.7(optional):
`sudo apt install python3.7-dev`

- Install pygpg
```
git clone https://github.com/lianghongzhuo/pygpg.git
cd pygpg
git clone https://github.com/pybind/pybind11
python setup.py develop
```

## Example:
```
import pygpg
points = [put your point cloud here, should be a nX3 numpy array]
grasps = pygpg.generate_grasps(points)
```

## Known issues:
- pcl default will add python2.7 path in to the system https://github.com/pybind/pybind11/issues/1637#issuecomment-557609822
  So we need to hard code witch python to use in cmake file to avoid the pybind binding python2.7 with gpg.
