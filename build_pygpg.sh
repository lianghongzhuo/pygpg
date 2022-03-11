#!/usr/bin/env bash
rm -rf build pygpg.egg-info
rm *gnu.so
python setup.py develop
