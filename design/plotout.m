clear;
close all;
data = csvread('out.txt');
% data = data(2:end, :) - data(1:end-1, :);
plot(data, '-','LineWidth', 4); grid on;
% xlabel('$t$ [ms]', 'Interpreter', 'Latex', 'FontSize', 14)
xlabel('時刻', 'Interpreter', 'Latex', 'FontSize', 12)
% legend({'$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'}, 'Interpreter', 'Latex', 'FontSize', 14);
legend({'加速度', '速度', '距離'}, 'FontSize', 14, 'Location', 'SouthWest');
title('曲線加速 (2次関数)','fontsize', 14);
% title('台形加速','fontsize', 14);
% title('始点速度 → 加速 → 最大速度 → 減速 → 終点速度','fontsize', 14);
hold off;
