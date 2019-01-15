function c = ccd(c)
%CCD Cyclic Coordinate Descent
%c = IK chain

tol = 0.0001; % Tolerance
max = 100;   % Max Iteration

Pd = c.P;    % Target Position
c = fk3(c, c.x0);
pPh = Pd;

for iter = 1 : max
    
    Ph = c.pts(end, :); % End Effector Position
    
    dP = Pd - Ph;
    E = dot(dP,dP); % Error Function
    
    % Check error tolerance and whether the end effector location is
    % converged.
    if E < tol || norm(pPh - Ph)^2 < tol
        %fprintf('Converged\n');
        break; % Converged
    end

%     MATLAB R2016 and below    
    for i = 1:size(c.pts,1)
        e(i,:) = Ph(1:3) - c.pts(i,1:3); % End Effectos position with respect to the current joint
        t(i,:) = Pd(1:3) - c.pts(i,1:3); % Target position with respect to the current joint
    end
    
%     e = Ph(1:3) - c.pts(:,1:3); % End Effectos position with respect to the current joint
%     t = Pd(1:3) - c.pts(:,1:3); % Target position with respect to the current joint
    
    for i = c.n : -1 : 1     
    
        % Local z basis vector expressed in world space
        z = c.H(1:3,1:3,i)*[0,0,1]';
        
        % Solve for angle analytically
        phi = ccdsolve(t(i,:), e(i,:), z, c.ub(i)-c.x0(i), c.lb(i)-c.x0(i));
        
        % Get the new angle
        c.x0(i) = c.x0(i) + phi;
        
        % Rotate in the local z direction to get the new end effector vector
        ee = e(i,:);
        e(i,:) = (z*dot(ee,z)*(1-cos(phi)))' + ee*cos(phi) + cross(z,ee)*sin(phi);
        
        % Update end effector position with respect to i-1 joint
        if i > 1
            e(i-1,:) = e(i,:) + (c.pts(i,1:3)-c.pts(i-1,1:3)); 
        end
        
    end
    
    % Recalculate positions of all joints based on new x0
    c = fk3(c, c.x0);
    
    % Set update previous end effector position
    pPh = Ph;
    
end 