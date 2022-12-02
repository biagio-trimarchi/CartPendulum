function A = A(x, u, params)
    % Return the linearize A matrix around (x*, u*) 
    % without the terms due to elastic behaviours or friction forces
    % _______________
    % x : system state
    % u : control input
    % params : parameter of the simulation
    
    % Extract parameters
    M = params.M;   % Cart mass
    m = params.m;   % Pendulum mass
    L = params.L;   % Rod length
    g = params.g;   % Gravity acceleration
    
    % Auxiliary variables (x2 dynamics)
    den_x2 = M + m*sin(x(3))^2;                                     % Denominator of x2 dynamics
    num_x2 = u - m*g*cos(x(3))*sin(x(3)) + m*L*sin(x(3))*x(4)^2;    % Numerator of x2 dynamics
    
    d_den_x2 = 2*m*cos(x(3))*sin(x(3));                             % Derivative of den_x2 wrt x3
    d_num_x2 = m*g*( sin(x(3))^2 - cos(x(3))^2 + ...
                m*L*cos(x(3))*x(4)^2 );                             % Derivative of num_x2 wrt x3 
    
    den_x4 = L*den_x2;                                              % Denominator of x4 dynamics
    num_x4 = (M + m)*g*sin(x(3)) - ... 
                m*L*sin(x(3))*cos(x(3))*x(4)^2 - u*cos(x(3));       % Numerator of x4 dynamics
    
    d_den_x4 = L*d_den_x2;                                          % Derivative of den_x4 wrt x3
    d_num_x4 = (M + m)*g*cos(x(3)) + ...
                m*L*( (sin(x(3))^2 - cos(x(3))^2)*x(4)^2 ) + ...  
                u*sin(x(3));                                        % Derivative of den_x4 wrt x3
    %% Linearized matrix
    A = zeros(4,4);
    
    % df/dx1
    A(1, 1) = 0;
    A(2, 1) = 0;
    A(3, 1) = 0;
    A(4, 1) = 0;
    
    % df/dx2
    A(1, 2) = 1;
    A(2, 2) = 0;
    A(3, 2) = 0;
    A(4, 2) = 0;
    
    % df/dx3
    A(1, 3) = 0;
    A(2, 3) = ( d_num_x2 * den_x2 - d_den_x2 * num_x2 ) ... 
                / ...
             (den_x2)^2;
    A(3, 3) = 0;
    A(4, 3) = ( d_num_x4 * den_x4 - d_den_x4 * num_x4 ) ... 
                / ...
             (den_x4)^2;
    
    % df/dx4
    A(1, 4) = 0;
    A(2, 4) = 0;
    A(3, 4) = 1;
    A(4, 4) = 0;
    
    
end