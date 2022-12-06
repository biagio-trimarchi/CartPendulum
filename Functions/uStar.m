function u = uStar(y, params)
    % Compute the feedforward action for steady-state angle
    % of the pendulum

    % Extract parameters
    M = params.M;   % Cart mass
    m = params.m;   % Pendulum mass
    g = params.g;   % Gravity acceleration

    u = (M+m)*g*sin(y)/cos(y);
end