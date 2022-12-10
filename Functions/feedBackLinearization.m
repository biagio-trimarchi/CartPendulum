function u = feedBackLinearization(t, x, v, params)
    % Compute the feed forward u* from dynamic inversion
    % given desired trajectory
    % _______________
    % t : time instant
    
    % Extract parameters
    M = params.M;
    m = params.m;
    g = params.g;
    L = params.L;
    
    % Compute y
    [y, dy, ddy] = trajectory(t);
    
    % Compute feed forward u
    u = -( L*(M+m*sin(x(3))^2)*ddy - (M+m)*g*sin(x(3)) + m*L*sin(x(3))*cos(x(3))*x(4)^2 + v) ...
        / ...
        ( cos(x(3)) );

end