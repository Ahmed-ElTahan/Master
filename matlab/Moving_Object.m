clc; clear all; close all;
t = 0:0.01:10;
h = [];
x_pos = 0:0.01:6*pi;
y_pos = sin(x_pos);

for i = 1:length(x_pos)
delete(h);
h = plot(x_pos(i), y_pos(i),'o', 'MarkerSize',20, 'MarkerFaceColor', 'b');
grid on;
xlim([0 6*pi])
ylim([-1 1])
drawnow;
end