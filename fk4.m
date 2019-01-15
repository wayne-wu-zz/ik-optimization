function [y, pts] = fk4( c, x )
%FK_TRANSFORM Summary of this function goes here
%   Detailed explanation goes here

global chains;

H = eye(4);
if c.parentChain ~= 0
    if size(x,2) > c.n
        H(:,4) = fk4(chains(c.parentChain), x(c.n+1:end));
    else 
        H(:,4) = fk4(chains(c.parentChain), chains(c.parentChain).x0);
    end
else
    H(:,4) = c.parent;
end

l = c.l;
a = c.a;
d = c.d;

%global pts;

pts = zeros(c.n+1, 4);

H = H * c.pretransform;
pts(1,:) = H(:,4);

% Apply each transformation based on Denavit-Hartenberg
for i = 1:c.n
    % Calculate the DH Matrix
    A = [cos(x(i)), -sin(x(i))*cos(a(i)),  sin(x(i))*sin(a(i)), l(i)*cos(x(i)); ...
         sin(x(i)),  cos(x(i))*cos(a(i)), -cos(x(i))*sin(a(i)), l(i)*sin(x(i)); ...
         0,          sin(a(i)),            cos(a(i)),           d(i); ...
         0,          0,                    0,                   1];
    H = H * A;
    pts(i+1,:) = H(:,4);
    %c.pts = H(:,4);
end

y = pts(end,:);

end

