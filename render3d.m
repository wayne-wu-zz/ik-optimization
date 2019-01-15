function render3d( pts )
%RENDER3D Render the points in 3D space

% hold on

p = plot3(pts(:,1), pts(:,2), pts(:,3), '.', 'MarkerSize', 30);
p.Color = 'blue';
xlabel('X');
ylabel('Y');
zlabel('Z');

size = 2;
axis([-size,size,-size,size,-size,size]);

box on
ax = gca;
% ax.ZGrid = 'on';
% ax.XGrid = 'on';
% ax.YGrid = 'on';

% hold on
p = plot3(pts(:,1), pts(:,2), pts(:,3));
p.Color = 'blue';
p.LineWidth = 2.5;

end

