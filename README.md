## **Line Laser Hand-Eye Calibration**  

---
[//]: # (Image References)
[image0]: ./images/frame.png "frame"
[image1]: ./images/calib.png "calib"
[image2]: ./images/CalibrationBlock.jpg "block"
[lib]:    ./lib "lib"
[paper]:  ./doc "paper"


 
The MATLAB codes show the implementation of a Hand-Eye calibration for line laser scanners based on Tsai paper ![paper][paper] and some of his toolbox ![lib][lib]. The code computes homogeneous transformation of a laser scanner frame w.r.t a robot tool frame (aka hand-eye calibration).  


The coordinate frames for the calibration problem are shown below.   
![frame][image0]


The calibration has been done on the known size block similar to the one below.  
![block][image2]



The process is to capture the XYZ coordinate of the object points at multiple robot poses (6DoF) and the XYZ coordinates at the camera/measurement frame.
![calib][image1]


See main_uEpsCalib.m for details. It is quite straight forward to follow I hope. 

