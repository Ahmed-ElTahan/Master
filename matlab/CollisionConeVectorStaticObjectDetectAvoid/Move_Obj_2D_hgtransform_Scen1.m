clc; clear all; close all;
theta = 0:0.1:2*pi;
h = figure(1);

%% Ego
global Ego
Ego.r = 1;
Ego.x = Ego.r*cos(theta);
Ego.y = Ego.r*sin(theta);
Ego.z = 0;
Ego.g = hgtransform;
patch('XData',Ego.x,'YData',Ego.y,'FaceColor','black','Parent',Ego.g) % Create the Ego

%% Objects
% Object 1 (static)
x_obs = 0.1; y_obs = 5; z_obs = 0;
X_obs = [x_obs, y_obs z_obs];
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
global sb


%% Move the Ego
% for fixed velocity and linear move  between pt1 and pt2, equation is
% y = pt1 + x*(pt2-pt1)/T_tot)
x_target = 1;
y_target = 20;
z_target = 0;
new_pos =[0 0 0];
pt1 = [0 0 0];
pt2 = [x_target y_target z_target];
T_total = 10; % seconds
N_pts = T_total * 10;
dt = T_total/(N_pts-1);
time = linspace(0,T_total,N_pts);
l = 1;
d = 2; % safety boundary
X_init = [0 0 0];
X_target = [x_target y_target 0];
hold all
plot(x_target, y_target, 'x', 'MarkerSize',40, 'MarkerEdgeColor', 'r')
V_v = (X_target - X_init)/T_total;
V_v_init = V_v;

%% Move the Obstacle
V_obs = [0, 0, 0];

%% Initialization
collision = 0;
hold on
axis equal
xlim([-10 10])
ylim([-5 20])
grid on
r1 = [0 0 0];
r2 = [0 0 0];
AP_calculated_flag = 0;
there_was_collision = 0;
V_applied = [0 0 0];
X_ap = [0 0 0];
AP_reached = 0;
EscP_reached = 0;
for t=time

    new_pos = [V_v*dt + new_pos];
    Ego.x = new_pos(1);
    Ego.y = new_pos(2);
    X_v = [Ego.x Ego.y Ego.z];
    X_rel = X_obs - X_v;
    
    X_obs = [V_obs*dt + X_obs];
    
    % If collision cone gives there is a collision and the normal distance
    % equal or less than d_avo
    if(collision == 1 && (norm(X_rel)-d<= d)) % Collision
        V_v = dot(V_v, r1)/(norm(r1)^2).*r1; % Project the Velocity to r1
        V_v = V_v*norm(V_v_init)/norm(V_v); % Apply same previous velocity Magnitude
        V_applied = V_v; % save the applied velocity
        if(AP_calculated_flag == 0) % Condition flag to get the aiming point position
            X_ap = r1 + X_v; % aiming point is equal to the current position + relative to the ap
            AP_calculated_flag=1; % set the condition to true, so ap is calculated
        end
        there_was_collision = 1; % indicate that there was a collision detected
    else
        if(there_was_collision == 0) % if no collision
            V_v = (X_target - X_init)/T_total; % apply the original velocity which is directed to the targed directly
        else
            if (norm(X_v - X_ap) <=norm(V_v)*dt*1.1) % checking that ap is reached by getting the distance between current position and the aiming point and compare to an error which is the minimum step according to current velocity for the error
                AP_reached = 1; % set flag that we reached the aimimng point
                X_escap = X_ap; % let the escap point at the same as aiming point
            end
            if(AP_reached == 1) % if reached the aiming point
                X_rel_unit = X_rel/norm(X_rel); % get the unit of the X_rel
                X_v_normal(1,1) = X_rel_unit(1,1); % assume x component of the normal to X_v same as of X_rel_unit
                X_v_normal(1,2) = -X_rel_unit(1,1)*X_rel_unit(1,1)/X_rel_unit(1,2); % getting y component of X_v_normal
                X_v_normal(1,3) = 0; % set the z component to 0
                if (dot(X_v_normal, V_v) < 0) % checking the direction between X_v_normal and V_v, if -ve opposite, if +ve same direction
                    X_v_normal = -X_v_normal; % reverse X_v_normal to be in same direction with V to be used for projection
                end
                new_V_v = (X_target - X_v)/T_total; % calculate the required velocity to reach the target from the current position
                angleCos = dot(V_applied/norm(V_applied), new_V_v/norm(new_V_v)); % calculate the angle between the unit vector of latest applied velocity and between unit vector of the needed targeted velocity
                if(angleCos>0.99) % if they are very close, i.e. angle is almost 0
                    EscP_reached = 1;
                end
                if(EscP_reached == 1)
                    new_V_v = (X_target - X_escap)/T_total; % use the target and the escape point to get the new needed velocity to go to the target
                    V_v = new_V_v; % set the V_v by the calculated velocity
                    V_v = V_v*norm(V_v_init)/norm(V_v); % apply the same magnitude of init velocity to current the velocity
                else % applied velocity and targetted velocity are not close to each other, so we need to move tangentially along the safety boundary to reach the escape point
                    V_v = dot(V_v, X_v_normal)/(norm(X_v_normal)^2).*X_v_normal; % project the current velocity to the normal to X_v
                    V_v = V_v*norm(V_v_init)/norm(V_v); %% apply the same init velocity magnitude
                    V_applied = V_v; % update the applied velocity with the new calculated velocity
                    X_escap = X_v; % update the escape point to the current position for the next check
                end
            end
            
        end
    end
    Ego.g.Matrix = makehgtform('translate',new_pos);
    Obj(1).g.Matrix = makehgtform('translate',X_obs); % Move the object ahead the Ego

    %% Collision Cone Vectors (stationary object)    
    [a, b, collision, r1, r2, u1, u2] = CollisionConeStaticDetect(X_v, X_obs, V_v, V_obs, d, h);
    t_go = t + norm((X_obs - X_v))/norm(V_v);
    figure(2)
    subplot(2,1,1)
    plot(t,a,'.', 'MarkerSize',10, 'MarkerEdgeColor', 'k')
    xlabel('t (seconds)')
    ylabel('a')
    grid on
    hold on
    subplot(2,1,2)
    plot(t,b,'.', 'MarkerSize',10, 'MarkerEdgeColor', 'k')
    hold on
    xlabel('t (seconds)')
    ylabel('b')
    grid on
end