% 3D Inverse Kinematic using Optimization
clear all;
close all;

% global chains;
% 
% root = [0,0,0,1];
% 
% % Snake
% chains(1).parent = root;
% chains(1).n  = 6;
% chains(1).pretransform = eye(4);
% chains(1).l  = [0,1,0,1,0,1]; % Link Length
% chains(1).a  = [pi/2,-pi/2,pi/2,-pi/2,pi/2,0]; % Link Twist
% chains(1).d  = zeros(1,chains(1).n); % Link Offset
% chains(1).lb = ones(1,chains(1).n)*(-pi/2); % lower bound angles 
% chains(1).ub = ones(1,chains(1).n)*(pi/2);  % upper bound angles
% chains(1).x0 = zeros(1,chains(1).n);
% chains(1).P  = [5,1,0,1];                 % End Effector Target Position [x,y,z,1]
% chains(1).pts = zeros(chains(1).n, 4);      % Joint Points
% 
% % Plot root
% plot3(root(1), root(2), root(3), 'O', 'MarkerSize', 10);
% 
% % Optimize Each Chain
% 
% frame = 1;
% fps = 30;
% 
% fig = figure('Name', 'Test');

%c = chains(i);

% c = simplechain(3);
% objfun = @(x)norm(c.P - fk2(c, x))^2;
% [c.x0, fval] = fmincon(objfun, c.x0, [], [], [], [], c.lb, c.ub);
% 

for i = 1:30

    n = i;
    
    %optionsunc = optimoptions(@fminunc,'Algorithm','quasi-newton','MaxFunctionEvaluation',1000*2*n);
    %optionscon = optimoptions(@fmincon,'MaxFunctionEvaluation',1000*2*n);
    
    d(i) = n*2;
    
    c = simplechain(n);
    
    % Interior Point Method
    tic
    
    objfun = @(x)norm(c.P - fk2(c, x))^2;
    [c.x0, fval] = fmincon(objfun, c.x0, [], [], [], [], c.lb, c.ub);
    
    x(i) = toc;
    
    c = simplechain(n);
    
    % Cyclic Coordinate Descent
    tic
    
    c = ccd(c);
    
    y(i) = toc;
    
    % Quasi-Newton
    
    c = simplechain(n);
    
    tic
    
    objfun = @(x)norm(c.P - fk2(c, x))^2 + penaltyfunc_fminuncon(c,x);
    [c.x0, fval] = fminunc(objfun,c.x0);   
    
    z(i) = toc;

end


figure; hold on
a1 = plot(d(2:end),y(2:end)); M1 = 'CCD';
a2 = plot(d(2:end),z(2:end)); M2 = 'Quasi-Newton';
legend([a1;a2],M1,M2);
xlabel('DOF');
ylabel('Time(s)');


