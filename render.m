function render( root, L, x )
%RENDER Draws the IK system.
%Root: The starting point
%L: lengths of each segment
%x: angle of each joint

n = max(size(L));

X = zeros(n+1, 1);
Y = zeros(n+1, 1);

X(1) = root(1);
Y(1) = root(2);

for i = 1:n
    X(i+1) = X(i) + L(i)*cos(x(i));
    Y(i+1) = Y(i) + L(i)*sin(x(i));
end 

plot(X, Y, 'O', 'MarkerSize', 5);
hold on
plot(X, Y);

end

