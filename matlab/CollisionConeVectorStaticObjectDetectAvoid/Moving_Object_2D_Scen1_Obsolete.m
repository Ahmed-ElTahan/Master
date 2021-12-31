clc; clear all; close all;
Obj1.diagram = [];
Obj1.size = 20; % R = 10 cmMarker size, specified as a positive value in points, where 1 point = 1/72 of an inch.
x_pos = 0:0.01:6*pi;
y_pos = sin(x_pos);

for i = 1:length(x_pos)
delete(Obj1.diagram);
Obj1.diagram = plot(x_pos(i), y_pos(i),'o', 'MarkerSize',Obj1.size, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
m = plot(x_pos(i), y_pos(i),'.', 'MarkerSize',2, 'MarkerEdgeColor', 'k');
hold all
grid on;
xlim([0 6*pi])
ylim([-1 1])
drawnow;
end