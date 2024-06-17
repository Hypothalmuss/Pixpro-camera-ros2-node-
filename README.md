***I originally wrote this for ros2 foxy-distro, but I have also tried it on humble distro and it works perfectly***
This package allowed me to connect my pixpro sp360 camera using usb cable.
You can find the calibration functions aswell; one function to take calibrations` pictures and another one to generate the .yaml file containing the camera`s matrix and coefficients.
I used the 6x9 checkboard calibration for my case, If you are using another one make sure to update that on the calibrate_camera.py file.
The camera is supposed to detect aruco markers 6x6-250. If you are using different markers, change that in subscriberImage.py file.

