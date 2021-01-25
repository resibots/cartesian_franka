# cartesian_franka
A simple library for moving the Franka robot in cartesian space

## Install
- libfranka:
```
cd libfranka
mkdir build; cd build; cmake .. -DCMAKE_INSTALL_PREFIX=$HOME -DCMAKE_BUILD_TYPE=Release
make -j9
make install
```
- pybind:
```
pip install pybind11
```
- cartesian_franka
```
cd cartesian_franak
./waf configure --libfranka=$HOME --python
./waf
```
