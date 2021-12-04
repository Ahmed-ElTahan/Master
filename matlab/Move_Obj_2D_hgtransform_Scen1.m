clc; clear all; close all;
theta = 0:0.1:2*pi;

%% Ego
Ego.r = 1;
Ego.x = Ego.r*cos(theta);
Ego.y = Ego.r*sin(theta);
Ego.g = hgtransform;
patch('XData',Ego.x,'YData',Ego.y,'FaceColor','black','Parent',Ego.g) % Create the Ego

%% Objects
% Object 1 (static)
i = 1;
Obj(i).r = 1;
Obj(i).x = Obj(i).r*cos(theta);
Obj(i).y = Obj(i).r*sin(theta);
Obj(i).g = hgtransform;
patch('XData',Obj(i).x,'YData',Obj(i).y,'FaceColor','yellow','Parent',Obj(i).g) % Create the Object
Obj(i).g.Matrix = makehgtform('translate',[0 5 0]); % Move the object ahead the Ego


hold on
axis equal
xlim([-20 20])
ylim([-20 20])
grid on


%% Move the Ego
% for fixed velocity and linear move  between pt1 and pt2, equation is 
% y = pt1 + x*(pt2-pt1)/T_tot)

pt1 = [0 0 0];
pt2 = [0 10 0];
T_total = 10; % seconds

for t=linspace(0,T_total,T_total*10)
  % Move the object linearly
  Ego.g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1)/T_total);
  % Draw Tail
  plot(pt1(1)+t*(pt2(1)-pt1(1))/T_total, pt1(2)+t*(pt2(2)-pt1(2))/T_total,'.', 'MarkerSize',3, 'MarkerEdgeColor', 'k')
  drawnow
end

