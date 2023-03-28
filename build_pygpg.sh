#!/usr/bin/env bash
mkdir dependencies && cd dependencies
git clone https://github.com/lianghongzhuo/gpg.git  # some cmake change to let it work as subproject
git clone https://github.com/pybind/pybind11
cd ..
pip install -e .
