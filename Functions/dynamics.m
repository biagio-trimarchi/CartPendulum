function dx = dynamics(x, u, params)
    % Compute cart with pendulum dynamics
    % _______________
    % x : system state
    % u : control input
    % params : parameter of the simulation
    
    % Extract parameters
    k = params.k;       % Elastic constant
    c = params.c;       % Friction coefficient
    M = params.M;       % Mass of the cart
    m = params.m;       % Mass of the pendulum
    g = params.g;       % Gravity acceleration
    L = params.L;       % Length of the rod
    
    % Auxiliary variables (just to avoid typing errors)
    tau = u - k*x(1) - c*x(2);      % External forces on the cart) 
    den = M + m*sin(x(3))^2;        % Common denominator 
    
    dx = zeros(4, 1);
    % State derivative
    dx(1) = x(2);
    dx(2) = ...
            ( tau - m*g*cos(x(3))*sin(x(3)) + m*L*sin(x(3))*x(4)^2) ...
                / ...
            ( den );
    dx(3) = x(4);
    dx(4) = ...
            ( (M + m)*g*sin(x(3)) - m*L*sin(x(3))*cos(x(3))*x(4)^2 - tau*cos(x(3))) ...
            / ...
            (L * den );
end