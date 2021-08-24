# Install pygpg
- Install gpg
```
git clone https://github.com/atenpas/gpg.git
cd gpg && mkdir build && cd build && cmake .. && make
sudo make install
```
- Install Python 3.7:
`sudo apt install python3.7-dev`

- Then run command (in Python 3.7 environment):
```
git clone https://github.com/lianghongzhuo/pygpg.git
cd pygpg
git clone https://github.com/pybind/pybind11
python setup.py develop
```
