% Finding W P R tool orientation of FANUC from the object orientation
% Note: current axis ZYX (Roll-Pitch-Yaw) orientation convention
% 0- Assigned pattern matching for the point pairs
% 1- Using the camera coordinates and object coordinates of the known
% points to find the rotation matrix (Kabsch.m the least square method)
% 2- Finding WPR from the frame transformation elements

% SAMPLE %
 
 % Generate testing data
%{ 
 data = [2 9 8 3 2 5 5 2 9;
         3 8 2 5 2 5 2 0 3;
         2 7 3 6 1 2 1 4 0];
 
  Pw = [3.83 6.08 15.78 18.03;
        4.43 4.43 4.43  4.43 ;
        0    0    0     0   ];
  Pc = [-10.48  -7.85     1.54   4.133;
          0       0        0      0;
          65.18   65.16    65.11  65.08];  
  wHc = solveH(Pc,Pw);
 
  r_pose = [1043.01 
            -38.26    
            574.92    
              5.28   
            -80.34  
             175.43]
  bHg, wHc  
 %}
 
 data =  [-10.48  -7.85    1.54   4.133;
          0       0        0      0;
          65.18   65.16    65.11  65.08];
 rx = -90;
 ry =  0;
 rz =  90;
 dx =  -100;
 dy =  -70;
 dz =  35;
 [Q] = rot_data_zyx_c(data,rx,ry,rz,dx,dy,dz); 

 % Finding the estimation of the Transformation matrix from the known point
 % pairs data (camera) <--> Q (object)
 [U, r, lrms] = Kabsch(data, Q);
 % U = Rotation matrix
 % l = Linear offset
 % lrms = error
 
 % Finding Yaw-Pitch-Roll from the Transformation matrix
 % Solution: Introduction to robotics book p 43
 % a = rot(Z) -> Fanuc R
 % b = rot(Y) -> Fanuc P
 % c = rot(x) -> Fanuc W
 
 % Angle of Orientation (Z Y X) order
 r31 = U(3,1);
 r11 = U(1,1);
 r21 = U(2,1);
 r32 = U(3,2);
 r33 = U(3,3);
 
 b = atan2(-r31,sqrt(r11^2 + r21^2));
 a = atan2(r21/cos(b),r11/cos(b));
 c = atan2(r32/cos(b),r33/cos(b));
 % degree
 R = a*180/pi;
 P = b*180/pi;
 W = c*180/pi;
 % radian
 Rz = a; Ry = b; Rx = c;
 
 % Translation
 tx = r(1);
 ty = r(2);
 tz = r(3);
 
 % Homogeneous matrix
 H  =    [cos(Rz)*cos(Ry)  cos(Rz)*sin(Ry)*sin(Rx)-sin(Rz)*cos(Rx)  sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx) tx;
          sin(Rz)*cos(Ry)  sin(Rz)*sin(Ry)*sin(Rx)+cos(Rz)*cos(Rx)  sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx) ty;
          -sin(Ry)         cos(Ry)*sin(Rx)                          cos(Ry)*cos(Rx)                         tz;
                 0                             0                                       0                     1];
             
 
 