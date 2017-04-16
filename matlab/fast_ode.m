clear
segment = 90;
%{
% search 90
pos_end = [segment/2-10, segment/2-10, pi/2];
%}
%{
% fast 45
pos_end = [segment, segment/2, pi/4];
%}
%%{
% fast 90
pos_end = [segment/2*2-10, segment/2*2-10, pi/2];
%}
%{
% fast V90
pos_end = [segment/2*sqrt(2), segment/2*sqrt(2), pi/2];
%}
%{
% fast 135
pos_end = [0+25, segment, pi*3/4];
%}
%{
% fast 180
pos_end = [0, segment, pi];
%}

%{
% 30
pos_end = [segment/2*(sqrt(3)+1)/2, segment/2*(sqrt(3)-1)/2, pi/6];
%}
%{
% 60
pos_end = [segment*2/3, segment/2, pi/3];
%}
%{
% 90
pos_end = [segment/2, segment/2, pi/2];
%}
%{
% 120
pos_end = [segment/2, segment/2*sqrt(3), pi*2/3];
%}
%{
% 150
pos_end = [20, segment, pi/6*5];
%}
%{
% 180
pos_end = [0, segment, pi];
%}

omega_dot = 144 * pi;
omega_max = 6 * pi;
angle = pos_end(3);
dx = 1;

T = omega_max / omega_dot * pi;
[t, theta] = ode45(@(t, theta) omega_max * sin(pi*t/T)^2, [0 T], 0);

if angle < theta(end)
    theta_gain = sqrt(angle / theta(end));
    T = T * theta_gain;
    [t, x] = ode45(@(t, x) cos(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0 T], 0);
    [t, y] = ode45(@(t, x) sin(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0 T], 0);
    syms v;
    v = double(solve((pos_end(2)-v*y(end))*cos(angle)==(pos_end(1)-v*x(end))*sin(angle), v));
    dt = dx/v;
    x_end = x(end)*v; y_end = y(end)*v;
    omega = omega_max * sin(pi*[0:dt:T]/T).^2;
    subplot(6, 1, 1); hold off;
    plot(0:dt:T, omega, '.', 'MarkerSize', 12); grid on;
    [t, theta] = ode45(@(t, theta) theta_gain * omega_max * sin(pi*t/T)^2, [0:dt:T], 0);
    subplot(6, 1, 2); hold off;
    plot(t, theta, '.', 'MarkerSize', 12); grid on;
    [t, x] = ode45(@(t, x) v * cos(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0:dt:T], 0);
    [t, y] = ode45(@(t, y) v * sin(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0:dt:T], 0);
    subplot(6, 1, [3 6]); hold off;
    plot(x, y, '.', 'MarkerSize', 12); grid on;
    
    pos = [x, y, theta];
    pos = [pos; x_end, y_end, angle];
else
    T1 = T / 2;
    T2 = T1 + (angle - theta(end)) / omega_max;
    T3 = T2 + T / 2;
    [t, x1] = ode45(@(t, x1) cos((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), [0 T/2], 0);
    [t, x2] = ode45(@(t, x2) cos((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), [T1 T2], x1(end));
    [t, x3] = ode45(@(t, x3) cos(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), [T2 T3], x2(end));
    [t, y1] = ode45(@(t, y1) sin((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), [0 T/2], 0);
    [t, y2] = ode45(@(t, y2) sin((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), [T1 T2], y1(end));
    [t, y3] = ode45(@(t, y3) sin(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), [T2 T3], y2(end));
    syms v;
    v = double(solve((pos_end(2)-v*y3(end))*cos(angle)==(pos_end(1)-v*x3(end))*sin(angle), v));
    dt = dx/v;
    t1 = 0:dt:T1;
    t2 = t1(end):dt:T2;
    t3 = t2(end):dt:T3;
    x1_end = x1(end)*v; x2_end = x2(end)*v; x3_end = x3(end)*v;
    y1_end = x1(end)*v; y2_end = y2(end)*v; y3_end = y3(end)*v;
    
    subplot(6, 1, 1); hold off;
    plot(t1, omega_max * sin(pi*t1/T).^2, '.', 'MarkerSize', 12); grid on; hold on;
    plot(t2, omega_max+t2*0, '.', 'MarkerSize', 12); grid on; hold on;
    plot(t3, omega_max * sin(pi*(t3-T2+T1)/T).^2, '.', 'MarkerSize', 12); grid on; hold on;

    subplot(6, 1, 2); hold off;
    theta1 = (omega_max*t1)/2 - (T*omega_max*sin((2*pi*t1)/T))/(4*pi);
    theta2 = (omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t2-T1);
    theta3 = omega_max*(T2-T1) + (omega_max*(t3-T2+T1))/2 - (T*omega_max*sin((2*pi*(t3-T2+T1))/T))/(4*pi);
    plot(t1, theta1, '.', 'MarkerSize', 12); grid on; hold on;
    plot(t2, theta2, '.', 'MarkerSize', 12); grid on; hold on;
    plot(t3, theta3, '.', 'MarkerSize', 12); grid on; hold on;

    [t, x1] = ode45(@(t, x1) v*cos((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), t1, 0);
    [t, x2] = ode45(@(t, x2) v*cos((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), t2, x1(end));
    [t, x3] = ode45(@(t, x3) v*cos(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), t3, x2(end));
    [t, y1] = ode45(@(t, y1) v*sin((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), t1, 0);
    [t, y2] = ode45(@(t, y2) v*sin((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), t2, y1(end));
    [t, y3] = ode45(@(t, y3) v*sin(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), t3, y2(end));
    subplot(6, 1, [3 6]); hold off;
    plot(x1, y1, '.', 'MarkerSize', 12); hold on; grid on;
    plot(x2, y2, '.', 'MarkerSize', 12); hold on; grid on;
    plot(x3, y3, '.', 'MarkerSize', 12); hold on; grid on;
    
    pos = [x1, y1, theta1'; x2(2:end), y2(2:end), theta2(2:end)'; x3(2:end), y3(2:end), theta3(2:end)'];
    pos = [pos; x3_end, y3_end, omega_max*(T2-T1) + (omega_max*(T3-T2+T1))/2 - (T*omega_max*sin((2*pi*(T3-T2+T1))/T))/(4*pi)];
end

format long;
dlmwrite('data.csv', pos, 'precision', '%.10f');
velocity = v
extra_straight = (pos_end(2)-pos(end, 2)) / sin(angle)
length = size(pos, 1)

subplot(6,1,1);
title(sprintf('$$ \\dot{\\omega}_{max}: %.0f\\pi,\\ \\omega_{max}: %.0f\\pi $$', omega_dot/pi, omega_max/pi), 'Interpreter','latex', 'FontSize', 12);
xlabel('t', 'Interpreter','latex', 'FontSize', 12);
ylabel('\omega', 'FontSize', 12);

subplot(6,1,2);
title(sprintf('$$ \\theta_{end}: %.2f\\pi $$', angle/pi), 'Interpreter','latex', 'FontSize', 12);
xlabel('t', 'Interpreter','latex', 'FontSize', 12);
ylabel('\theta', 'FontSize', 12);

subplot(6,1,[3 6]);
title(sprintf('$$ v_{max}: %.3f $$', v), 'Interpreter','latex', 'FontSize', 12);
xlabel('x', 'Interpreter','latex', 'FontSize', 12);
ylabel('y', 'Interpreter','latex', 'FontSize', 12);
