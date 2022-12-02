%%% 

%%% Setup environment
clc;
clear all;
addpath("./Functions")

%% Simulation parameters
dt = 0.001;                         % Time step
step = cast(1/dt, 'uint32') / 10;   % Animation time step
T = 10.0;                            % Total time
x0 = [0, 0, pi, 1.5*0.3*pi];        % Initial condition
tspan = 0:dt:T;                     % Time instants

%% Plant parameters nominal
paramsN.M = 11.0;            % Cart mass
paramsN.m = 6.0;             % Pendulum mass
paramsN.L = 4.0;             % Rod length
paramsN.g = 9.81;            % Gravity acceleration
paramsN.k = 0.0;             % Elastic coefficient
paramsN.c = 0.0;             % Friction force

%% Plant paramters real
paramsR.M = 12.0;            % Cart mass
paramsR.m = 5.6;             % Pendulum mass
paramsR.L = 3.8;             % Rod length
paramsR.g = 9.81;            % Gravity acceleration
paramsR.k = 0.0;             % Elastic coefficient
paramsR.c = 0.2;             % Friction force

%% Example simulation
% u = @(t) feedForwardAction(t, params);
% f = @(t, x) dynamics(x, u(t), params);
% 
% x0 = [0, 0, -pi, 0];
% tspan = 0:dt:T;
% [t, xx] = ode45(f, tspan, x0);

%% Example simulation euler

% xx(:, 1) = x0';
% for tt=1:length(tspan)-1
%     u = feedForwardAction(tspan(tt), paramsN);
%     xx(:, tt+1) = xx(:, tt) + dynamics(xx(:, tt), u, paramsR)*dt;
% end
% 
% xx = xx';

%% Example place
u_ff = feedForwardAction(tspan(1), paramsN);
[y, dy, ~] = trajectory(tspan(1));
y = wrapTo2Pi(y);
x_ = feedForwardState(y, dy);
x_(3) = wrapTo2Pi(x_(3));
A_ = A(x_, u_ff, paramsN);
B_ = B(x_, paramsN);
C_ = C();
B_ = B_(3:4);
A_ = A_(3:4, 3:4);
C_ = C_(3:4);
AA = [A_, zeros(2,1); C_, 0];
BB = [B_; 0];
poles = [-4, -5, -2];
K = place(AA, -BB, poles);
Kx = K(1:2);
Ks = K(end);

xx(:, 1) = x0;
sigma(1) = 1/Ks * ( u_ff' - Kx * x_(3:4)' );
for tt=1:length(tspan)-1
    x(3, :) = wrapTo2Pi(xx(3, tt));
    u_ff = feedForwardAction(tspan(tt), paramsN);
    [y, dy, ~] = trajectory(tspan(tt));
    y = wrapTo2Pi(y);
    yy(tt) = y;
    x_ = feedForwardState(y, dy);
    x_(3) = wrapTo2Pi(x_(3));
    A_ = A(x_, u_ff, paramsN);
    B_ = B(x_, paramsN);
    C_ = C();
    B_ = B_(3:4);
    A_ = A_(3:4, 3:4);
    C_ = C_(3:4);
    
    AA = [A_, zeros(2,1); C_, 0];
    BB = [B_; 0];
    aux = wrapTo2Pi(xx(3, tt));
    if ( abs(pi/2 - aux) < 0.01 || abs(-pi/2 - aux) < 0.01 )
        u = u_ff;
    else
        poles = [-4, -5, -2];
        K = place(AA, -BB, poles);
        Kx = K(1:2);
        Ks = K(end);
        u = Kx*(xx(3:4, tt)) + Ks*sigma(tt);
    end
    
    xx(:, tt+1) = xx(:, tt) + dynamics(xx(:, tt), u, paramsR)*dt;
    sigma(tt+1) = sigma(tt) + (xx(3, tt) - y)*dt; 
end

xx = xx';


%% Debug
% x = [0.0; 0.0; -pi/2; 0];
% u = 0;
% 
% B_ = B(x, paramsN);
% A_ = A(x, u, paramsN);
% C_ = C();
% 
% % Check controllability/observability properties
% Co = ctrb(A_, B_);
% Oo = obsv(A_, C_);
% rank(Co)
% rank(Oo)
% rank([-A_, B_; C_, 0])
% 
% % Notice that the linearization is completely controllable, but 
% % it has unosservable dynamics

%% Cart plot
% figure(1)
% xaxis([-10, 10])
% yaxis([-10, 10])
% hold on
% plotCart([0, 0, 0, 0], params)
% hold off

fig = figure(1);
for ii = 1:step:length(tspan)
    clf(fig)
    xaxis([-10, 10])
    yaxis([-10, 10])
    hold on
    plotCart(xx(ii, :), paramsR)
    hold off
    
    drawnow
end

plot(yy)
hold on
plot(xx(:, 3))
hold off
legend('y', 'x')