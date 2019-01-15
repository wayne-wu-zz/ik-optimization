% Main script for solving IK

% Rest Position
Root  = [0, 5]; % Root position (x, y);
x0    = [0, 0]; % Angle of each joint vectors
L     = [3, 3]; % Lengths of each segment

A = [1,0;0,1];
b = [pi, pi]; 

[x, fval] = fmincon(@objfunc, x0, A, b);

render(Root, L, x);
