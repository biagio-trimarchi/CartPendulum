function B = B(x, params)
    % Return the linearize B matrix around (x*, u*)
    % _______________
    % x : system state
    % params : parameter of the simulation
    
    %%% NB: Notice that B does not depends upon u*
    
    % Extract parameters
    M = params.M;   % Cart mass
    m = params.m;   % Pendulum mass
    L = params.L;   % Rod length
    
    % Auxiliary variables (just to avoid typing errors)
    den = M + m*sin(x(3))^2;          % Common denominator
    
    % Linearized matrix
    B = zeros(4,1);
    B(1) = 0;                       
    B(2) = 1 / den;    
    B(3) = 0;
    B(4) = - cos(x(3)) / (L * den);
    
    
end