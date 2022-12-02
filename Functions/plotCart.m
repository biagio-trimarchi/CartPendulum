function plotCart(x, params)
    % Plot the cart with pendulum
    % _______________
    % x : system state
    % params : parameter of the simulation
    
    % Extract parameters
    L = params.L;                       % Rod length

    % Rectangle
    h = 1.0;                            % Heigth
    w = 2.0;                            % Width
    pos = [x(1) - w/2, -h/2, w, h];     % Position vector for plot
                                            % Matlab requires the lower
                                            % left corner position along
                                            % with the width and the heigth
    rectangle('Position', pos, 'FaceColor', 'k');
    
    % Rod
    O = [x(1), 0];                          % Position of the rod hinge
    P = [x(1) + L*sin(x(3)), L*cos(x(3))];  % Position of the pendulum mass
    
    plot([O(1), P(1)], [O(2), P(2)], 'LineWidth', 2)
                                                
    
end