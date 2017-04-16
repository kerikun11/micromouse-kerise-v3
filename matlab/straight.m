clear;
syms t T a_max v_max v_0

a_max = 6000;
v_0 = 600;
v_max = 1200;

T = 1.5 * (v_max - v_0) / a_max;
v(t) = v_0 + (v_max - v_0) * 6 * (-1/3*(t/T)^3+1/2*(t/T)^2);
a(t) = diff(v, t);
x(t) = int(v, t);

%{
T = (v_max-v_0) / a_max * pi;
v(t) = v_0 + (v_max-v_0) * sin((t/T)^2);
%}

%{
T = 4*(v_max-v_0)/a_max;
v(t) = v_0 + (v_max-v_0) * 2*(t/T)^2;
a(t) = diff(v, t);
x(t) = int(v, t);

x((2*2^(1/2)*(-v_max)^(1/2)*(v_0 - v_cur)^(1/2))/a_max)
%}
%{
a_max = 6000;
v_max = 1200;
v_0 = 0;
ezplot(v, [0 T])
%}
