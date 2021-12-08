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
x_obs = 5; y_obs = 5;
i = 1;
Obj(i).r = 1;
Obj(i).x = Obj(i).r*cos(theta);
Obj(i).y = Obj(i).r*sin(theta);
Obj(i).g = hgtransform;
patch('XData',Obj(i).x,'YData',Obj(i).y,'FaceColor','yellow','Parent',Obj(i).g) % Create the Object
Obj(i).g.Matrix = makehgtform('translate',[x_obs y_obs 0]); % Move the object ahead the Ego


hold on
axis equal
xlim([-10 10])
ylim([-5 20])
grid on
q1 = []
q2 = []
q3 = []
q4 = []
q5 = []
q6 = []
X_prev = [0, 0];
%% Move the Ego
% for fixed velocity and linear move  between pt1 and pt2, equation is 
% y = pt1 + x*(pt2-pt1)/T_tot)
pt1 = [0 0 0];
pt2 = [0 15 0];
T_total = 10; % seconds
N_pts = T_total * 10;
dt = T_total/(N_pts-1)
time = linspace(0,T_total,N_pts);
l = 1;
Ego.x_prev = 0;
Ego.y_prev = 0;
d = 2; % safety boundary
SafeBound_x = d*cos(theta) + x_obs;
SafeBound_y = d*sin(theta) + y_obs;
figure(1)
plot(SafeBound_x,SafeBound_y,'-')

for t=time

  % Move the object linearly
  Ego.g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1)/T_total);
  % Draw Tail
  Ego.x = pt1(1)+t*(pt2(1)-pt1(1))/T_total;
  Ego.y = pt1(2)+t*(pt2(2)-pt1(2))/T_total;
  figure(1)
  plot(Ego.x, Ego.y,'.', 'MarkerSize',3, 'MarkerEdgeColor', 'k')
  %% Collision Cone Vectors (stationary object)
X = [x_obs-Ego.x y_obs-Ego.y];
delete(q1)
delete(q2)
delete(q3)
delete(q4)
delete(q5)
delete(q6)
V = [(Ego.x-Ego.x_prev)/dt, (Ego.y-Ego.y_prev)/dt];
q1 = quiver(Ego.x, Ego.y, X(1), X(2),0, 'LineWidth', 2); % X vector
q2 = quiver(Ego.x, Ego.y, V(1), V(2),0, 'LineWidth', 2); % V vector
Ego.x_prev = Ego.x;
Ego.y_prev = Ego.y;

X_norm = norm(X);
V_norm = norm(V);
c = sqrt((X_norm^2-d^2)/(X_norm^2*V_norm^2-(dot(X,V))^2));
u1 = -1/X_norm^2*(c*dot(X,V)+d)*X+c*V;
u2 = 1/X_norm^2*(c*dot(X,V)-d)*X-c*V;
a = 1/2*(dot(X,V)/(X_norm^2-d^2)+1/c/d);
b = 1/2*(dot(X,V)/(X_norm^2-d^2)-1/c/d);

q3 = quiver(x_obs, y_obs, d*u1(1), d*u1(2),0, 'LineWidth', 2); % u1
q4 = quiver(x_obs, y_obs, d*u2(1), d*u2(2),0, 'LineWidth', 2); % u2

q5 = quiver(Ego.x, Ego.y, X(1)+d*u1(1), X(2)+d*u1(2),0, 'LineWidth', 2); % r1
q6 = quiver(Ego.x, Ego.y, X(1)+d*u2(1), X(2)+d*u2(2),0, 'LineWidth', 2); % r2

drawnow

figure(2)
subplot(2,2,1)
plot(t,a,'.', 'MarkerSize',3, 'MarkerEdgeColor', 'k')
hold on
subplot(2,2,2)
plot(t,b,'.', 'MarkerSize',3, 'MarkerEdgeColor', 'k')
hold on
subplot(2,2,3)
plot(t,c,'.', 'MarkerSize',3, 'MarkerEdgeColor', 'k')
hold on
end




