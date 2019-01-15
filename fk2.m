function y = fk2( c, x )
%FK_TRANSFORM Summary of this function goes here
%   Detailed explanation goes here

l = c.l;
a = c.a;
d = c.d;

global pts;

pts = zeros(c.n+1, 4);
pts(1,:) = c.parent;
t = eye(4);
t(:,4) = c.parent;
H = t * c.pretransform;
pts(2,:) = H(:,4);

% Apply each transformation based on Denavit-Hartenberg
for i = 1:c.n
    % Calculate the DH Matrix
    A = [cos(x(i)), -sin(x(i))*cos(a(i)),  sin(x(i))*sin(a(i)), l(i)*cos(x(i)); ...
         sin(x(i)),  cos(x(i))*cos(a(i)), -cos(x(i))*sin(a(i)), l(i)*sin(x(i)); ...
         0,          sin(a(i)),            cos(a(i)),           d(i); ...
         0,          0,                    0,                   1];
    H = H * A;
    pts(i+2,:) = H(:,4);
    c.pts = H(:,4);
end

y = pts(end,:);

end

