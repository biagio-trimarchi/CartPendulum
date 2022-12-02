function x = feedForwardState(y, dy)
    % Compute x* from dynamics inversion of the system
    % given the desired trajectory
    % _______________
    % y : Desired position
    % dy : Desired velocity
    
    x(1) = 0;       % Does not appear in the equation of A,B,C
    x(2) = 0;       % Does not appear in the equation of A,B,C
    
    x(3) = y;       % x(3) is just the output angle
    x(4) = dy;      % x(4) is the derivative of the output angle
end