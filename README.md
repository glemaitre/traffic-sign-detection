# traffic-sign-detection
Real-time traffic sign detection using Gielis curves

## System & dependencies

System requirements:

* Ubuntu 14.04
* Qt 4.8.x
* Eigen 3.x
* OpenCV 2.x

### Qt library

To install the full Qt SDK

`sudo apt-get install qt-sdk`

The version available in Ubuntu 14.04 repositories is Qt 4.8.5

### Eigen Library

`sudo apt-get install libeigen3-dev`

The version available in Ubuntu 14.04 repositories is Eigen 3.2.0

### OpenCV Library

`sudo apt-get install libopencv-dev`

### Google tests
Recommended to install from source
https://code.google.com/p/googletest/

Google tests depends on pthread

The version available in Ubuntu 14.04 repositories is OpenCV 2.4.8

## Compilation

* Create a binary folder:

`mkdir bin`

* Move to the created folder:

`cd bin`

* Create the `MakeFile` via qmake:

`qmake-qt4 ../`

* Compile the code to generate the executable:

`make`

* In order to run the code:

`./traffic-sign-detection ../test-images/different0035.jpg`
