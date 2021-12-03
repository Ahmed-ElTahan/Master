clc; clear all; close all;
theta = 0:0.1:2*pi;
r = 1;
x = r*cos(theta);
y = r*sin(theta);
g = hgtransform;
patch('XData',x,'YData',y,'FaceColor','yellow','Parent',g)
hold on
axis equal
xlim([-10 10])
ylim([-10 10])
pt1 = [-3 -4 0];
pt2 = [5 2 0];
for t=linspace(0,1,100)
  g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1));
  plot((-3+t*8), (-4+t*6),'.', 'MarkerSize',5, 'MarkerEdgeColor', 'k')
  grid on
  drawnow
end