# traffic-sign-detection
Real-time traffic sign detection using Gielis curves

## System & dependencies

System requirements:

* Ubuntu 14.04
* CMake 2.8.12
* Eigen 3.2.0
* OpenCV 2.4.8
* GTest 1.6.1

### CMake Library

`sudo apt-get install cmake`

The version available in Ubuntu 14.04 repositories is CMake 2.8.12

### Eigen Library

`sudo apt-get install libeigen3-dev`

The version available in Ubuntu 14.04 repositories is Eigen 3.2.0

### OpenCV Library

`sudo apt-get install libopencv-dev`

The version available in Ubuntu 14.04 repositories is OpenCV 2.4.8

### Google tests

`sudo apt-get install libgtest-dev`

However, this command only download the source that you need to compile.

```
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
```

The version available in Ubuntu 14.04 repositories is GTest 1.6.1

## Compilation

* Create a build folder:

`mkdir build`

* Move to the created folder:

`cd build`

* Create the `MakeFile` via cmake:

`cmake ../src` 

* Compile the code to generate the executable:

`make` or `make -j n` where `n` is the number of cores to use for the compilation

* A folder bin will be created at the same level as the build directory.

* In order to run the code:

`../bin/main ../test-images/different0035.jpg`
