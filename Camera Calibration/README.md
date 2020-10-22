# Camera Calibration
Given two images of a mold: “left.jpg” and “right.jpg” from two cameras, and the coordinates (X, Y, Z) of some corners of the mold in world frame as below:

![Object](https://github.com/Praj390/AuE8240_Autonomous_Driving_Technology/blob/master/Camera%20Calibration/Figure.jpg)

1. Use least square approach to find the 11 parameters for the left camera.
2. Use least square approach to find the 11 parameters for the right camera.
3. Calculate the coordinates (X, Y, Z) of other marked corners of the mold based on the two images. 
4. Calculate the dimensions of the bar (length, width, height) beside the mold.
5. Estimate the volume of the bottle.