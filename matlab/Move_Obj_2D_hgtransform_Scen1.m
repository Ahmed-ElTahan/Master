clc; clear all; close all;
theta = 0:0.1:2*pi;
h = figure(1);

%% Ego
global Ego
Ego.r = 1;
Ego.x = Ego.r*cos(theta);
Ego.y = Ego.r*sin(theta);
Ego.g = hgtransform;
patch('XData',Ego.x,'YData',Ego.y,'FaceColor','black','Parent',Ego.g) % Create the Ego

%% Objects
% Object 1 (static)
x_obs = 5; y_obs = 5;
X_obs = [x_obs, y_obs];
i = 1;
Obj(i).r = 1;
Obj(i).x = Obj(i).r*cos(theta);
Obj(i).y = Obj(i).r*sin(theta);
Obj(i).g = hgtransform;
patch('XData',Obj(i).x,'YData',Obj(i).y,'FaceColor','yellow','Parent',Obj(i).g) % Create the Object
Obj(i).g.Matrix = makehgtform('translate',[x_obs y_obs 0]); % Move the object ahead the Ego

global q1
global q2
global q3
global q4
global q5
global q6


X_prev = [0, 0];
%% Move the Ego
% for fixed velocity and linear move  between pt1 and pt2, equation is
% y = pt1 + x*(pt2-pt1)/T_tot)
new_pos =[0 0 0];
pt1 = [0 0 0];
pt2 = [0 15 0];
T_total = 10; % seconds
N_pts = T_total * 10;
dt = T_total/(N_pts-1);
time = linspace(0,T_total,N_pts);
l = 1;
Ego.x_prev = 0;
Ego.y_prev = 0;
d = 2; % safety boundary
V_v = [0 1.5 0];

hold on
axis equal
xlim([-10 10])
ylim([-5 20])
grid on

for t=time
    
    % Move the object linearly
%     Ego.g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1)/T_total);
    new_pos = [V_v*dt + new_pos];
    Ego.g.Matrix = makehgtform('translate',new_pos);    
    Ego.x = new_pos(1);
    Ego.y = new_pos(2);
    %% Collision Cone Vectors (stationary object)
    X = [x_obs-Ego.x y_obs-Ego.y];
    
    X_v = [Ego.x, Ego.y];
    [a, b, collision] = CollisionConeDetect(X_v, X_obs, V_v(1:2), d, h);
    t_go = t + norm((X_obs - X_v))/norm(V_v);
    % if(a>0 && b>0)
    % Ego.x = 40*(t-t_go)
    % end
    
    figure(2)
    subplot(2,2,1)
    plot(t,a,'.', 'MarkerSize',3, 'MarkerEdgeColor', 'k')
    hold on
    subplot(2,2,2)
    plot(t,b,'.', 'MarkerSize',3, 'MarkerEdgeColor', 'k')
    hold on

end




