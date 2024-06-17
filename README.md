***I originally wrote this for ros2 foxy-distro, but I have also tried it on humble distro and it works perfectly***

-This package allowed me to connect my pixpro sp360 camera using usb cable.

-You can find the calibration functions aswell; one function to take calibrations' pictures and another one to generate the .yaml file containing the camera's matrix and coefficients.

-I used the 6x9 checkboard calibration for my case, If you are using another one make sure to update that on the calibrate_camera.py file.

![image1(1)](https://github.com/Hypothalmuss/Pixpro-camera-ros2-node-/assets/99557619/2b361667-775c-41db-a584-1d47f2b95bab)

-The camera is supposed to detect aruco markers 6x6-250. If you are using different markers, change that in subscriberImage.py file.

![image1](https://github.com/Hypothalmuss/Pixpro-camera-ros2-node-/assets/99557619/e15e94a5-16a4-47c2-bcbc-119bfb3da620)
