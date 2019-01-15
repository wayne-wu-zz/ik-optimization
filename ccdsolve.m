function x = ccdsolve(t, e, z, ub, lb)
%CCDSOLVE Solve analytically for the maximum angle
% t = target position with respect to current joint
% e = end position with respect to current joint
% z = unit vector in the z-direction of the current joint

k1 = dot(t,z)*dot(e,z);
k2 = dot(t,e);
k3 = dot(z,cross(e,t));

% Analytic maximum
if(abs(k3 - 0) < 0.00001)
    x = 0; % Force x to be 0 if k3 == 0
else
    x = atan(k3/(k2-k1));
end

% Check second derivative
sd = (k1-k2)*cos(x) - k3*sin(x);
g_ub = k1*(1-cos(ub)) + k2*cos(ub) + k3*sin(ub);
g_lb = k1*(1-cos(lb)) + k2*cos(lb) + k3*sin(lb);

if sd > 0
    if g_ub > g_lb
        x = ub;
    else
        x = lb;
    end
end 

% Check bounds
if x > ub
    if lb < x - 2*pi < ub
        x = x - 2*pi;
    else
        if g_ub > g_lb
            x = ub;
        else
            x = lb;
        end
    end
elseif x < lb
    if lb < x + 2*pi < ub
        x = x + 2*pi;
    else
        if g_ub > g_lb
            x = ub;
        else
            x = lb;
        end
    end
end

end

