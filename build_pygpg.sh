#!/usr/bin/env bash
rm -rf dependencies build *.egg-info *.so && mkdir dependencies && cd dependencies
git clone https://github.com/lianghongzhuo/gpg.git --depth=1  # some cmake change to let it work as subproject
git clone https://github.com/pybind/pybind11 --depth=1
cd ..
pip install -e .
