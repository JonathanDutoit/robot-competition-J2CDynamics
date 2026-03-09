# Camera 

The testing script that can be found in this folder creates a live video of the camera output in format (640, 480) using `Picamera2` and `Flask` packages and can be accessed by: `http://<ip_of_raspy>:5000/video_feed`. In the initial test, it only consumes around 12-13% of the CPU and negligible memory. 


# Dual Stream

The `dual_stream_yolo.py` script servers a dual-stream (2 threads) for simultaneous live camera feed and duplo detection! By testing on the Raspy 5, it uses around 50% CPU usage. The video feed is served in `http://172.21.64.110:7123/video_feed`
