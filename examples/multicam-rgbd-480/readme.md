## Frame drops test summary

### Hardware/ OS
1. D455 sensor
- firmware 5.16.0.1

2. Jetson Orin 32GB Developer Kit
- JP6.0
- librealsense-2.56.3 (native install using kernel patches)

### Environment:
2 environments are setup for the testing:
1. Static - RS_calibration_target (A3 size) at a distance of 1.2m from the D455 sensor
2. Dynamic - RS_claibration_target (A3 size) is moved in 6DOF space in front of the D455 sensor

* RS_calibration_target is attached.

### Test configuration:
4 tests, each ~5 minutes long, are performed for each of the 2 environments defined above with following configurations:
1. HDRD_720:
- Infrared left (1280x720, 30Hz)
- Depth (1280x720, 30Hz)
- A `rs2::frameset` is aquired using RS API which consists of 2 frames - infrared left frame and depth frame

2. HDRD_480:
- Infrared left (848x480, 60Hz)
- Depth (848x480, 60Hz)
- A `rs2::frameset` is aquired using RS API which consists of 2 frames - infrared left frame and depth frame

3. RGBD_720:
- Color (1280x720, 15Hz)
- Depth (1280x720, 15Hz)
- A `rs2::frameset` is aquired using RS API which consists of 2 frames - color frame and depth frame

4. RGBD_480:
- Color (640x480, 30Hz)
- Depth (848x480, 30Hz)
- A `rs2::frameset` is aquired using RS API which consists of 2 frames - color frame and depth frame

* A total of 8 tests are performed.
* An example command to run a 5 minute test is as follows:
`timeout 5m rs-multicam-hdrd-720 [D455_SERIAL_NO]`

### Code:
4 `.cpp` files, each for the defined configurations, are attached herewith.
* The code is adapted from the `rs-multicam.cpp` example code from RS API. 

### Results:
* Frame drops are printed to the console in the following format and logged to a `.txt` file for each of the 8 tests. Two examples of frame drop logs are shown below:

E.g. 1:
`RS frame dropped: 9606 ! (2)`
* In the above example, 9606 represents the frame_count right after the frame drop. (2) represents the difference between the current frame_count and the previous frame_count. (e.g. 9606-9604 = 2). This corresponds to 1 frame drop (9605 frame_count is missing).

E.g. 2:
`RS frame dropped: 1384 ! (3)`
* In the above example, 1384 represents the frame_count right after the frame drop. (3) represents the difference between the current frame_count and the previous frame_count. (e.g. 1384-1381 = 3). This corresponds to 2 consecutive frame drops (1382 and 1383 frame_counts are missing).

* The log files for all 8 tests are attached:
1. hdrd_720_static.txt
2. hdrd_480_static.txt
3. rgbd_720_static.txt
4. rgbd_480_static.txt
5. hdrd_720_dynamic.txt
6. hdrd_480_dynamic.txt
7. rgbd_720_dynamic.txt
8. rgbd_480_dynamic.txt