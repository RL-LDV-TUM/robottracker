robottracker
============

Tracking E-Puck Robots using AR(uco)-markers
[ArUco Library](http://www.uco.es/investiga/grupos/ava/node/26)

# Installation
1. Install OpenCV (~2.4.8)
2. Install Aruco (~1.2.4)
3. Build 
```bash
mkdir build
cd build
cmake ..
cd ..
make
```
# Run
```bash
bin/tracker_main [CameraId] [Camera.yml] [markerlength(cm)] [serverport]
```
