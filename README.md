# Python binding for Grasp Pose Generator (pyGPG)
[![DOI](https://zenodo.org/badge/399418729.svg)](https://zenodo.org/badge/latestdoi/399418729)

[Grasp Pose Generator](https://github.com/atenpas/gpg.git) is a cpp package that creat grasp candidates for 3D point clouds.
This package binding it with python.

## Install

```bash
./build_pygpg.sh
```

## Example:
```python
import numpy as np
import pygpg

points = np.random.rand(3000,3)  # put your point cloud here, should be a nX3 numpy array, here is an example random array
grasps = pygpg.generate_grasps(points)
```

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
