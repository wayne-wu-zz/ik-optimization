function c = simplechain( n )
%SIMPLECHAIN returns a simple chain

c.parent = [0,0,0,1];
c.n  = n*2;
c.pretransform = eye(4);
for i = 1:c.n;
    c.l(i) = mod(i,2) == 0;
    if c.l(i) == 0
        c.a(i) = pi/2;
    else
        c.a(i) = -pi/2;
    end
end

c.a(end) = 0;
c.d  = zeros(1,c.n);           % Link Offset
c.lb = ones(1,c.n)*(-pi/2);    % lower bound angles 
c.ub = ones(1,c.n)*(pi/2);     % upper bound angles
c.x0 = zeros(1,c.n);
c.P  = [n/3,n/3,n/3,1];                    % End Effector Target Position [x,y,z,1]
c.pts = zeros(c.n, 4);         % Joint Points
c.k = 100;

end

