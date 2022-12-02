function u = feedForwardAction(t, params)
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
    u = -( L*(M+m*sin(y)^2)*ddy - (M+m)*g*sin(y) + m*L*sin(y)*cos(y)*dy^2 ) ...
        / ...
        ( cos(y) );
end