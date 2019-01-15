function chains = gethuman(root)
%GETHUMAN Summary of this function goes here
%   Detailed explanation goes here

% Right Leg
chains(1).parent = root;
chains(1).pretransform = ...
        [ 0, 0, 1, 0.2;  ...
          0, 1, 0, 0.0;  ...
         -1, 0, 0, 0.0;  ...
          0, 0, 0, 1.0];
chains(1).n = 5;
chains(1).l = [0, 0.5, 0.5, 0, 0.1]; % Link Length
chains(1).a = [-pi/2, pi/2, 0, -pi/2, 0]; % Link Twist
chains(1).d = [0,0,0,0,0]; % Link Offset
chains(1).lb = [-0.5236, -0.7854, -2.2689, 1.2217, -0.34906]; % OP, PQ, QR + 90 degrees 
chains(1).ub = [ 2.2689, 0.1745, 0.01, 2.3558, 0.5236]; % OP, PQ, QR + 90 degrees
chains(1).x0 = [0,0,0,pi/2,0];
chains(1).P = [0.2,0.2,-1,1];               % End Effector Target Position
chains(1).pts = zeros(chains(1).n, 4);     % Joint Points
chains(1).k = 100;

% Left Leg
chains(2).parent = root;
chains(2).pretransform = ...
        [ 0, 0, 1, -0.2;  ...
          0, 1, 0, 0.0;  ...
         -1, 0, 0, 0.0;  ...
          0, 0, 0, 1.0];
chains(2).n = 5;
chains(2).l = [0, 0.5, 0.5, 0, 0.1]; % Link Length
chains(2).a = [-pi/2, pi/2, 0, -pi/2, 0]; % Link Twist
chains(2).d = [0,0,0,0,0]; % Link Offset
chains(2).lb = [-0.5236, -0.1745, -2.2689, 1.2217, -0.5236]; %OP, PQ, QR + 90 degrees
chains(2).ub = [2.2689, 0.7854, 0.01, 2.3558, 0.34906]; %OP, PQ, QR + 90 degrees
chains(2).x0 = [0,0,0,pi/2,0];
chains(2).P = [-0.2,0.2,-1,1];               % End Effector Target Position
chains(2).pts = zeros(chains(1).n, 4);     % Joint Points
chains(2).k = 100;

% Body
chains(3).parent = root;
pretransform1 = ... % -90 in y rotation
        [ 0, 0, -1, 0.0;  ...  % [R11 R12 R13 x-translation]
          0, 1,  0, 0.0;  ...  % [R11 R12 R13 x-translation]
          1, 0,  0, 0.0;  ...  % [R11 R12 R13 x-translation]
          0, 0,  0, 1.0];      % [0 0 0 1] by default
pretransform2 = ... % 180 in x rotation
        [ 1,  0,   0, 0.0;  ...  % [R11 R12 R13 x-translation]
          0, -1,   0, 0.0;  ...  % [R11 R12 R13 x-translation]
          0,  0,  -1, 0.0;  ...  % [R11 R12 R13 x-translation]
          0,  0,   0, 1.0];      % [0 0 0 1] by default
chains(3).pretransform = pretransform1*pretransform2;
chains(3).n = 4;
chains(3).l = [0,0.75,0,0.75]; % Link Length
chains(3).a = [pi/2,-pi/2,pi/2, 0]; % Link Twist
chains(3).d = [0,0,0,0]; % Link Offset
% chains(3).lb = [-0.75899, -0.40086, -0.36359,-0.55899, -0.10086, -0.16359]; %ADx, ADy, ADz, ADDx, ADDy, ADDz (upper torso is ADD)
% chains(3).ub = [0.36359,0.40086,0.36359,0.16359,0.10086,0.16359]; %ADx, ADy, ADz, ADDx, ADDy, ADDz
chains(3).lb = [-1.5708, -0.549066, -0.261799, -0.37453]; %ABx, ABy, BDx, BDy(upper torso is BD
chains(3).ub = [0.436332, 0.549066,  0.174533,  0.37453]; %ABx, ABy, BDx, BDy
chains(3).x0 = [0,0,0,0];
chains(3).P = [0,0,1.5,1];              % End Effector Target Position
chains(3).pts = zeros(chains(3).n, 4);     % Joint Points
chains(3).k = 50;

% Right Arm
chains(4).parent = [0,0,1.5,1];
chains(4).pretransform = ... % 90 in y rotation
    [ 0,  0,   1, 0.3;  ...  % [R11 R12 R13 x-translation]
      0,  1,   0, 0.0;  ...  % [R11 R12 R13 x-translation]
     -1,  0,   0, 0.0;  ...  % [R11 R12 R13 x-translation]
      0,  0,   0, 1.0];      % [0 0 0 1] by default
chains(4).n = 4;
chains(4).l = [0,0.5,0,0.5]; % Link Length
chains(4).a = [-pi/2,pi/2,-pi/2,0]; % Link Twist
chains(4).d = [0,0,0,0]; % Link Offset
chains(4).lb = [-1.04719, -2.26892, 0      , -0.26179]; %HIx, HIy, IJx, IJz
chains(4).ub = [3.14159 , 0       , 2.61799,  1.57079]; %HIx, HIy, IJx, IJz
chains(4).x0 = [0,0,0,0];
chains(4).P  = [0.3,0,0.5,1];              % End Effector Target Position
chains(4).pts = zeros(chains(4).n, 5);     % Joint Points
chains(4).k = 80;

% Left Arm
chains(5).parent = [0,0,1.5,1];
chains(5).pretransform =  ...  % 90 in y rotation
    [ 0,  0,   1, -0.3;   ...  % [R11 R12 R13 x-translation]
      0,  1,   0,  0.0;   ...  % [R11 R12 R13 x-translation]
     -1,  0,   0,  0.0;   ...  % [R11 R12 R13 x-translation]
      0,  0,   0,  1.0];       % [0 0 0 1] by default
chains(5).n = 4;
chains(5).l = [0,0.5,0,0.5]; % Link Length
chains(5).a = [-pi/2,pi/2,-pi/2,0]; % Link Twist
chains(5).d = [0,0,0,0]; % Link Offset
chains(5).lb  = [-1.04719, 0      , 0      , -1.57079]; %EFx, EFy, FGx, FGz
chains(5).ub  = [3.14159 , 2.26892, 2.61799,  0.26179]; %EFx, EFy, FGx, FGz
chains(5).x0  = [0,0,0,0];
chains(5).P   = [-0.3,0,0.5,1];              % End Effector Target Position
chains(5).pts = zeros(chains(5).n, 5);     % Joint Points
chains(5).k = 80;

end

