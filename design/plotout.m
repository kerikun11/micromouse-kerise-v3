clear;
close all;
data = csvread('out.txt');
plot(data, 'LineWidth', 2); grid on;
xlabel('$t$ [ms]', 'Interpreter', 'Latex', 'FontSize', 14)
ylabel('value', 'Interpreter', 'Latex', 'FontSize', 14)
legend({'$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'}, 'Interpreter', 'Latex', 'FontSize', 14);
