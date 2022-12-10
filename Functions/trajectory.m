function [y, dy, ddy] = trajectory(t)
        % Compute the desired trajectory of the rod
        % _______________
        % t : time instant
        
        T1 = 2.0;
        T2 = 2.0;

        % Control points
        start = pi;
        medium = 5/4*pi;
        goal = 0;
        P1 = [start; start; start; medium; medium; medium];
        dP1 = 5*diff(P1)/T1;
        ddP1 = 4*diff(dP1)/T1;
        P2 = [medium; medium; medium; goal; goal; goal];
        dP2 = 5*diff(P2)/T2;
        ddP2 = 4*diff(dP2)/T2;

        

        if t < T1
            tau = t/T1;
            y = 0;
            for k = 1:6
                y = y + bernstein(tau, 5, k-1) * P1(k);
            end
            
            dy = 0;
            for k = 1:5
                dy = dy + bernstein(tau, 4, k-1) * dP1(k);
            end

            ddy = 0;
            for k = 1:4
                ddy = ddy + bernstein(tau, 3, k-1) * ddP1(k);
            end
        end

        if t < T1 + T2 && t >= T1
            tau = (t-T1)/T2;
            y = 0;
            for k = 1:6
                y = y + bernstein(tau, 5, k-1) * P2(k);
            end
            
            dy = 0;
            for k = 1:5
                dy = dy + bernstein(tau, 4, k-1) * dP2(k);
            end

            ddy = 0;
            for k = 1:4
                ddy = ddy + bernstein(tau, 3, k-1) * ddP2(k);
            end
        end

        if t >= T1 + T2
            y = 0;
            dy = 0;
            ddy = 0;
        end
        
        % y   = A*t^2*pi*sin(w*t) + pi;
        % dy  = A*(2*t*pi*sin(w*t) + t^2*w*pi*cos(w*t));
        % ddy = A*(2*pi*sin(w*t) + 2*t*w*pi*cos(w*t) + 2*t*w*pi*cos(w*t) - t^2*w^2*pi*sin(w*t));

end