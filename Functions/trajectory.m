function [y, dy, ddy] = trajectory(t)
        % Compute the desired trajectory of the rod
        % _______________
        % t : time instant
        
        % Control points
        w = 1.5;
        A = 0.3;
        y = A*pi*sin(w*t) + pi;
        dy = w*A*pi*cos(w*t);
        ddy = -(w^2)*A*pi*sin(w*t); 

end