% 3D Inverse Kinematic using Optimization
close all;
clear all;

global pts;

% Main Root of the Body
root = [0, 0, 0, 1];

chains = gethuman(root);

% Animation Frame
frame = 1;
fps = 150;

fig = figure('Name', 'Test');
set(gcf,'position',[10,10,900,600])

% Plot root
plot3(root(1), root(2), root(3), 'O', 'MarkerSize', 10);

view([160 30]); %adjust view angle perspective, [z-axis, top-down axis]

tic % Start Time

t2 = 11;
for t = 1:t2/fps:t2
    
    view([120+10*t 30]); %adjust view angle perspective, [z-axis, top-down axis]
    
%     % Ghetto Gangnam Style
    chains(1).P = [0.3+sin(2*t)/6, 0, (sin(2*t)-3)/3, 1]; % right leg
    chains(2).P = [-0.3+sin(2*t)/6, 0, -(sin(2*t)+3)/3, 1]; % left leg
    chains(4).P = [-0.3, 0.5, (sin(2*t)+3)/3, 1]; % right arm
    chains(5).P = [0.3, 0.5, (sin(2*t)+2)/3, 1]; % left arm
%  
    % Backpack Kid
%     root = [sin(2*t)/5,0,0,1];
%     for i = 1:3
%         chains(i).parent = root;
%     end
%     chains(4).P = [ 0.3-sin(2*t)/3,  sin(2*t)/5, 0.4+sin(4*t)/8, 1]; % right arm
%     chains(5).P = [-0.3-sin(2*t)/3, -sin(2*t)/5, 0.4+sin(4*t)/8, 1]; % left arm

%     chains(4).P = [0,0.4,0.5,1];
%     chains(5).P = [0,0.4,0.5,1];
    
    % Optimize Each Chain
    for i = 1:size(chains,2)
        
        objfun = @(x)norm(chains(i).P - fk2(chains(i), x))^2;
        [chains(i).x0, fval] = fmincon(objfun, chains(i).x0, [], [], [], [], chains(i).lb, chains(i).ub);     
        
        hold on
        
        % Draw Chain
        render3d(pts);
        
        p = plot3(chains(i).P(1), chains(i).P(2), chains(i).P(3), 'O', 'MarkerSize', 8);
        p.Color = 'red';
    end
    
    plot3(0, 0, 1.85, 'O', 'MarkerSize', 20, 'MarkerFaceColor', 'blue'); % head plot
    
    Frames(frame) = getframe(fig);
    frame = frame + 1;

    cla;
end

toc % End time

export('gangnam_std', Frames);
