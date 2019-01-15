function y = fk( x )
%FK: Calculate the forward kinematics based on joint variables

global root n l a d pts;

pts(1,:) = root;

H = eye(4);

% Base Transformation
H(:, 4) = root;

% Apply each transformation based on Denavit-Hartenberg
for i = 1:n
    A = [cos(x(i)), -sin(x(i))*cos(a(i)),  sin(x(i))*sin(a(i)), l(i)*cos(x(i)); ...
         sin(x(i)),  cos(x(i))*cos(a(i)), -cos(x(i))*sin(a(i)), l(i)*sin(x(i)); ...
         0,          sin(a(i)),            cos(a(i)),           d(i); ...
         0,          0,                    0,                   1];
    H = H * A;
    pts(i+1,:) = H(:,4);
end

y = pts(n+1, :); % End Effector Position

end

