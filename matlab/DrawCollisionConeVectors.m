function DrawCollisionConeVectors(X_v, X_obs, V_v, u1, u2, d, h)

global q1
global q2
global q3
global q4
global q5
global q6

theta = 0:0.1:2*pi;
SafeBound_x = d*cos(theta) + X_obs(1);
SafeBound_y = d*sin(theta) + X_obs(2);
X_rel = X_obs - X_v;

delete(q1)
delete(q2)
delete(q3)
delete(q4)
delete(q5)
delete(q6)

figure(h)
hold on
plot(X_v(1), X_v(2),'.', 'MarkerSize',10, 'MarkerEdgeColor', 'k')
q1 = quiver(X_v(1), X_v(2), X_rel(1), X_rel(2),0, 'LineWidth', 2); % X_rel vector
q2 = quiver(X_v(1), X_v(2), V_v(1), V_v(2),0, 'LineWidth', 2); % V_v vector
q3 = quiver(X_obs(1), X_obs(2), d*u1(1), d*u1(2),0, 'LineWidth', 2); % u1
q4 = quiver(X_obs(1), X_obs(2), d*u2(1), d*u2(2),0, 'LineWidth', 2); % u2
q5 = quiver(X_v(1), X_v(2), X_rel(1)+d*u1(1), X_rel(2)+d*u1(2),0, 'LineWidth', 2); % r1
q6 = quiver(X_v(1), X_v(2), X_rel(1)+d*u2(1), X_rel(2)+d*u2(2),0, 'LineWidth', 2); % r2
plot(SafeBound_x,SafeBound_y,'-')

hold off
drawnow

end

