% MATLAB CODE, P.Phairatt 25/Nov/2013 
% uEpsilon line scanner, robotic hand-eye calibration


% Note: - This work utilise a set of libraries to solve 
%         the problem of least sqaure estimation and calibration  
%         through a series of frame transformations
%       - Using a known size calibration specimen 
%       - See individual files for code owners and copyright

% w := world/object frame
% c := camera frame
% g := gripper(robot tool frame)
% b := base (robot base frame)
% H := transformation matrix


% --------------------------------------------- %
% Step 1: Finding wHc from point coordinates
% Kabsch estimation (camera to object)
% [U, r, lrms] = Kabsch(Pc, Pw);
% --------------------------------------------- %

% 5 set of data with 4 point constraints

% 4-Calibration points (X,Y,Z) in the world frame
  Pw = [8.3  16.6  8.3  16.6;
        0     0    -17  -17;
        0     3.7   0    3.7];

% #Pose 1#    
  Pc1 = [3.60     -4.97     3.69     -4.8;
          0       0         -17.29    -17.29;
          67.75   63.93     67.41     63.62];

  wHc1 = solveH(Pc1,Pw);
   
% #Pose 2#    
  Pc2 = [-0.61    -9       -0.86    -9.37;
          0        0       -16.43   -16.43;
          61.47    57.24   65.86    61.59];
       
  wHc2 = solveH(Pc2,Pw);
          
% #Pose 3#        
  Pc3 = [-0.48    -8.83   -0.663   -9.07;
          0       0        -15.46   -15.46 ; 
          72.88   68.48   66.77     62.34];   
    
  wHc3 =  solveH(Pc3,Pw);
% #Pose 4#      
  Pc4 = [2.479   -4.75   2.803     -4.48;
         0       0        -17.11   -17.11 ; 
         61.98    56.23   60.39     54.62];
         
  wHc4 = solveH(Pc4,Pw);
        
% #Pose 5#    
  Pc5 = [3.893   -5.234  3.18      -6.08;
         0       0      -16.587  -16.587   ;
         76.67    74.98   75.18    74.47];      
     
  wHc5 = solveH(Pc5,Pw);
   
% Stacking data      
  wHc(:,:,1) = wHc1;
  wHc(:,:,2) = wHc2;
  wHc(:,:,3) = wHc3;
  wHc(:,:,4) = wHc4;
  wHc(:,:,5) = wHc5;

        
% --------------------------------------------- %
% Step 2: Finding bHg from robot pose
% NOTE : Z Y X ACTIVE ROTATION!
% 
% --------------------------------------------- %     

%{
 % Homogeneous matrix ZYX
 H  =    [cos(Rz)*cos(Ry)  cos(Rz)*sin(Ry)*sin(Rx)-sin(Rz)*cos(Rx)  sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx) tx;
          sin(Rz)*cos(Ry)  sin(Rz)*sin(Ry)*sin(Rx)+cos(Rz)*cos(Rx)  sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx) ty;
          -sin(Ry)         cos(Ry)*sin(Rx)                          cos(Ry)*cos(Rx)                         tz;
                 0                             0                                       0                     1];
%}


% This is a 5 set of robot pose(6DoF= 3 translations, 3 rotations) when the measurement took place %
  r_pose = [ 1167.42     -44.25      525.39  63.55      -88.58  117.74;
             1118.44     -43.69      507.24  169.23     -73.93  11.84;
             1233.9      -41.16      529.76  8.97       -68.62  171.04;
             1180.9      -0.21       532.0   72.24      -73.86  108.8;
             1180.07     -87.0       514.65  -67.57     -76.53  -112.64;
           ];

% Compute transformation matrix (robot tool to robot base)
for i=1:1:5
  Rz = r_pose(i,4)*pi/180; Ry = r_pose(i,5)*pi/180; Rx = r_pose(i,6)*pi/180;
  tx = r_pose(i,1); ty = r_pose(i,2); tz = r_pose(i,3);
  
  bHg(:,:,i) =  [cos(Rz)*cos(Ry)  cos(Rz)*sin(Ry)*sin(Rx)-sin(Rz)*cos(Rx)  sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx) tx;
                 sin(Rz)*cos(Ry)  sin(Rz)*sin(Ry)*sin(Rx)+cos(Rz)*cos(Rx)  sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx) ty;
                -sin(Ry)          cos(Ry)*sin(Rx)                          cos(Ry)*cos(Rx)                         tz;
                 0                             0                                       0                     1];
end


% -------------------------------------------------------- %              
% Step 3: Calibration to find Camera to Robot tool offset              
%      
% -------------------------------------------------------- %
  gHc = handEye(bHg, wHc);






