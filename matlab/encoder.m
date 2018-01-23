N = 16834;

data = load('data.csv');
data = data(:, 1);
windowSize = 5;
data = filter(ones(1, windowSize) / windowSize, 1, data);
data = data(1900:2600, 1);
data = data - data(1) + N * 100;

v = (data(end)-data(1))/length(data);
datax = v * [1:length(data)];
datay = data';

characteristic = @(k, x) x + k(1) * sin(2 * pi * x / N + k(2)) + k(3);
k = lsqcurvefit(characteristic, [0,0,0], datax, datay);
fity = characteristic(k, datax);

figure(1);
hold off;
plot(datax, datay);
grid on;
hold on;
plot(datax, fity);

figure(2);
hold off;
plot(datay(2:end) - datay(1:end-1));
grid on;
hold on;
plot(fity(2:end) - fity(1:end-1));

f = @(x) x + k(1)*sin(2*pi*x/N + k(2));
invf = @(y) InverseFunction(f, y, -N/2, N+N/2);
figure(3); hold off;
x = 0:N;
plot(f(x));
hold on; grid on;
plot(arrayfun(invf, x));

dataa = data - floor(data / N) * N;
fix = arrayfun(invf, dataa) + floor(data / N) * N;

figure(4); hold off;
plot(data(2:end) - data(1:end-1));
hold on; grid on;
plot(fix(2:end) - fix(1:end-1));
