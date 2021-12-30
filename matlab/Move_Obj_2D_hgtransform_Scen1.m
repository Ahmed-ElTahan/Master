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
x_obs = 0.1; y_obs = 10;
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

%% Move the Ego
% for fixed velocity and linear move  between pt1 and pt2, equation is
% y = pt1 + x*(pt2-pt1)/T_tot)
x_target = 1;
y_target = 20;
new_pos =[0 0 0];
pt1 = [0 0 0];
pt2 = [x_target y_target 0];
T_total = 10; % seconds
N_pts = T_total * 10;
dt = T_total/(N_pts-1);
time = linspace(0,T_total,N_pts);
l = 1;
Ego.x_prev = 0;
Ego.y_prev = 0;
d = 3; % safety boundary
X_init = [0 0 0]
X_target = [x_target y_target 0];
hold all
plot(x_target, y_target, 'x', 'MarkerSize',40, 'MarkerEdgeColor', 'r')
V_v = (X_target - X_init)/T_total;
V_v_init = V_v;
norm(V_v_init)

collision = 0;
hold on
axis equal
xlim([-10 10])
ylim([-5 20])
grid on
r1 = [0 0];
r2 = [0 0];
ap_flag = 0;
was_collision = 0;
V_applied = [0 0];
ap = [0 0];
Ap_reached = 0;
for t=time

    % Move the object linearly
    %     Ego.g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1)/T_total);
    new_pos = [V_v*dt + new_pos];
    Ego.x = new_pos(1);
    Ego.y = new_pos(2);
    X_rel = [x_obs-Ego.x y_obs-Ego.y];
    
    % If collision cone gives there is a collision and the normal distance
    % equal or less than d_avo
    if(collision == 1 && (norm(X_rel)-d<= d))

        V_v(1:2) = dot(V_v(1:2), r1)/(norm(r1)^2).*r1;
        V_v(1:2) = V_v(1:2)*norm(V_v_init(1:2))/norm(V_v(1:2));
        V_applied(1:2) = V_v(1:2);
        if(ap_flag == 0)
            ap = r1 + [Ego.x, Ego.y];
            ap_flag=1;
        end
        was_collision = 1;
%         V_v(1:2) = (r1)/T_total
    else
        if(was_collision == 0)
            V_v(1:2) = (X_target(1:2) - X_init(1:2))/T_total;
        else
            if (norm([Ego.x Ego.y] - ap) <=norm(V_v)*dt*1.1)
                Ap_reached = 1;
            end
            if(Ap_reached == 1)
                UnitX = X_rel/norm(X_rel);
                NormX(1,1) = UnitX(1,1);
                NormX(1,2) = -UnitX(1,1)*UnitX(1,1)/UnitX(1,2);
                NormX(1,3) = 0;
                if (dot(NormX, V_v) < 0)
                    NormX = -NormX;
                end
                new_V_v = (X_target(1:2) - [Ego.x Ego.y])/T_total;
                angleCos = dot(V_applied/norm(V_applied), new_V_v/norm(new_V_v));
                if(angleCos>0.99)
                    new_V_v = (X_target(1:2) - X_escap)/T_total;
                    V_v(1:2) = new_V_v;
                    V_v(1:2) = V_v(1:2)*norm(V_v_init(1:2))/norm(V_v(1:2));
                else
                    V_v(1:2) = dot(V_v(1:2), NormX(1:2))/(norm(NormX(1:2))^2).*NormX(1:2);
                    V_v(1:2) = V_v(1:2)*norm(V_v_init(1:2))/norm(V_v(1:2));
                    V_applied(1:2) = V_v(1:2);
                    X_escap = [Ego.x Ego.y];
                end
            end
            
        end
    end
    Ego.g.Matrix = makehgtform('translate',new_pos);
    norm(V_v)
    %% Collision Cone Vectors (stationary object)
    X_rel = [x_obs-Ego.x y_obs-Ego.y];
    
    X_v = [Ego.x, Ego.y];
    [a, b, collision, r1, r2, u1, u2] = CollisionConeStaticDetect(X_v, X_obs, V_v(1:2), d, h);
    t_go = t + norm((X_obs - X_v))/norm(V_v);
    figure(2)
    subplot(2,2,1)
    plot(t,a,'.', 'MarkerSize',10, 'MarkerEdgeColor', 'k')
    xlabel('t (seconds)')
    ylabel('a')
    grid on
    hold on
    subplot(2,2,2)
    plot(t,b,'.', 'MarkerSize',10, 'MarkerEdgeColor', 'k')
    hold on
    xlabel('t (seconds)')
    ylabel('b')
    grid on
end