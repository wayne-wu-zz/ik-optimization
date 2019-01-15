% 3D Inverse Kinematic using Optimization
clear all;
close all;

global root chains pts;

% Main Root of the Body
root = [0, 0, 0, 1];

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
chains(1).P  = [0.2,0.2,-1,1];               % End Effector Target Position
chains(1).pts = zeros(chains(1).n, 4);      % Joint Points

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
chains(3).n = 3;
chains(3).l = [0,0,1.5]; % Link Length
chains(3).a = [-pi/2,pi/2,pi/2]; % Link Twist
chains(3).d = [0,0,0]; % Link Offset
chains(3).lb = [-1.30899, -0.61086, -0.52359]; %ADx, ADy, ADz
chains(3).ub = [0.52359,0.61086,0.52359]; %ADx, ADy, ADz
chains(3).x0 = [0,0,pi/2];
chains(3).P = [0,0,1.5,1];              % End Effector Target Position
chains(3).pts = zeros(chains(3).n, 4);     % Joint Points

% Right Arm
chains(4).parent = [0,0,1.5,1];
chains(4).parentChain = 3;
chains(4).pretransform = ... % 90 in y rotation
    [ 0,  0,   1, 0.3;  ...  % [R11 R12 R13 x-translation]
      0,  1,   0, 0.0;  ...  % [R11 R12 R13 x-translation]
     -1,  0,   0, 0.0;  ...  % [R11 R12 R13 x-translation]
      0,  0,   0, 1.0];      % [0 0 0 1] by default
chains(4).n = 3;
chains(4).l = [0,0.5,0.5]; % Link Length
chains(4).a = [-pi/2,pi/2,0]; % Link Twist
chains(4).d = [0,0,0]; % Link Offset
chains(4).lb = [-1.04719, -0.78539,0]; %HIx, HIy, HIz, IJx
chains(4).ub = [3.14159,2.26892,2.61799]; %ADx, ADy, ADz, IJx
chains(4).x0 = [0,0,0];
chains(4).P = [0.3,0,0.5,1];              % End Effector Target Position
chains(4).pts = zeros(chains(4).n, 5);     % Joint Points

% Left Arm
chains(5).parent = [0,0,1.5,1];
chains(5).parentChain = 3;
chains(5).pretransform =  ...  % 90 in y rotation
    [ 0,  0,   1, -0.3;   ...  % [R11 R12 R13 x-translation]
      0,  1,   0,  0.0;   ...  % [R11 R12 R13 x-translation]
     -1,  0,   0,  0.0;   ...  % [R11 R12 R13 x-translation]
      0,  0,   0,  1.0];       % [0 0 0 1] by default
chains(5).n = 3;
chains(5).l = [0,0.5,0.5]; % Link Length
chains(5).a = [-pi/2,pi/2,0]; % Link Twist
chains(5).d = [0,0,0]; % Link Offset
chains(5).lb = [-1.04719, -0.78539,0];     %HIx, HIy, HIz, IJx
chains(5).ub = [3.14159,2.26892,2.61799];  %ADx, ADy, ADz, IJx
chains(5).x0 = [0,0,0];
chains(5).P = [-0.3,0,0.5,1];              % End Effector Target Position
chains(5).pts = zeros(chains(4).n, 5);     % Joint Points

% Plot root
plot3(root(1), root(2), root(3), 'O', 'MarkerSize', 10);

% Animation Frames
frame = 1;
fps = 30;

fig = figure("Name", "Test");

for t = 1:5/fps:20 % Animation loop
    % Optimize Each Chain
%     chains(4).P = [-0.3, 0.3, sin(t)+1, 1]; % right arm
    chains(5).P = [-0.3, 0.3, (sin(t)+1)/2, 1]; % left arm
    for i = 1:5

        if i == 3 % Skip Body
            continue;
        end

        % Concatenate x0, lb, ub for fmincon
        c = chains(i);
        x0 = c.x0;
        lb = c.lb;
        ub = c.ub;
        while c.parentChain ~= 0
            c = chains(c.parentChain);
            x0 = horzcat(x0,c.x0);
            lb = horzcat(lb,c.lb);
            ub = horzcat(ub,c.ub);
        end

        % Optimize Chain
        objfun = @(x)norm(chains(i).P - fk4(chains(i), x))^2;
        [x0, fval] = fmincon(objfun, x0, [], [], [], [], lb, ub);

        % Update x0 for each chain if concatenated
        chains(i).x0 = x0(1:chains(i).n);
        next = chains(i).n+1;
        parent = chains(i).parentChain;
        while parent ~= 0
            chains(parent).x0 = x0(next:next+chains(parent).n-1);
            next = chains(parent).n + 1;
            parent = chains(parent).parentChain;
        end
    end

    % Draw Position
    for i = 1:5
        [y, points] = fk4(chains(i), chains(i).x0);
        render3d(points);
        p = plot3(chains(i).P(1), chains(i).P(2), chains(i).P(3), 'O', 'MarkerSize', 8);
        p.Color = 'red';
    end
    
    Frames(frame) = getframe(fig);
    frame = frame + 1;

    cla;
end

vidObj = VideoWriter('test');
vidObj.Quality = 70;
vidObj.FrameRate = 30;
open(vidObj);
writeVideo(vidObj, Frames);
close(vidObj); 

% Simulation



