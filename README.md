# Python binding for Grasp Pose Generator (pyGPG)
[![DOI](https://zenodo.org/badge/399418729.svg)](https://zenodo.org/badge/latestdoi/399418729)

[Grasp Pose Generator](https://github.com/atenpas/gpg.git) is a cpp package that creat grasp candidates for 3D point clouds.
This package binding it with python.

## Install
- Install gpg
```bash
git clone https://github.com/atenpas/gpg.git
cd gpg && mkdir build && cd build && cmake .. && make
sudo make install
```

- Install pygpg
```bash
git clone https://github.com/lianghongzhuo/pygpg.git
cd pygpg
git clone https://github.com/pybind/pybind11
python setup.py develop
```

## Example:
```python
import numpy as np
import pygpg

points = np.random.rand(3000,3)  # put your point cloud here, should be a nX3 numpy array, here is an example random array
grasps = pygpg.generate_grasps(points)
```

## Known issues:
- pcl default will add python2.7 path in to the system: [issues](https://github.com/pybind/pybind11/issues/1637#issuecomment-557609822).
  So we need to hard code witch python to use in cmake file to avoid the pybind binding python2.7 with gpg.

## Citation
If you found pyGPG useful in your research, please consider citing:
```
@software{pygpg,
  author       = {Hongzhuo Liang},
  title        = {Python binding for Grasp Pose Generator (pyGPG)},
  month        = Aug,
  year         = 2021,
  doi          = {10.5281/zenodo.5247189},
  url          = {https://doi.org/10.5281/zenodo.5247189}
}
```
