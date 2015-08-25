Configure:
```
mkdir build
mkdir install
cd build && cmake -DCMAKE_INSTALL_PREFIX=../install .. && cd ..
```

Build:
```
cmake --build build
```

![Travis build status](https://travis-ci.org/tkoolen/crtpDemo.svg?branch=master)
