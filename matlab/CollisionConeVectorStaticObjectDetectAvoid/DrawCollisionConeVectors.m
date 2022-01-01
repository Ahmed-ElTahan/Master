function DrawCollisionConeVectors(X_v, X_obs, V_v, V_obs, u1, u2, d, h)

global q1
global q2
global q3
global q4
global q5
global q6
global sb

theta = 0:0.1:2*pi;
SafeBound_x = d*cos(theta) + X_obs(1);
SafeBound_y = d*sin(theta) + X_obs(2);
X_rel = X_obs - X_v;
V_rel = V_v - V_obs;

delete(q1)
delete(q2)
delete(q3)
delete(q4)
delete(q5)
delete(q6)
delete(sb)


figure(h)
hold on
plot(X_v(1), X_v(2),'.', 'MarkerSize',10, 'MarkerEdgeColor', 'k')
q1 = quiver(X_v(1), X_v(2), X_rel(1), X_rel(2),0, 'LineWidth', 2, 'color', 'm'); % X_rel vector
q2 = quiver(X_v(1), X_v(2), V_rel(1), V_rel(2),0, 'LineWidth', 2, 'color', 'b'); % V_v vector
q3 = quiver(X_obs(1), X_obs(2), d*u1(1), d*u1(2),0, 'LineWidth', 2, 'color', 'g'); % u1
q4 = quiver(X_obs(1), X_obs(2), d*u2(1), d*u2(2),0, 'LineWidth', 2, 'color', 'g'); % u2
q5 = quiver(X_v(1), X_v(2), X_rel(1)+d*u1(1), X_rel(2)+d*u1(2),0, 'LineWidth', 2, 'color', 'r'); % r1
q6 = quiver(X_v(1), X_v(2), X_rel(1)+d*u2(1), X_rel(2)+d*u2(2),0, 'LineWidth', 2, 'color', 'r'); % r2
sb = plot(SafeBound_x,SafeBound_y,'-');

hold off
drawnow

end

