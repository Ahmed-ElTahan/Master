function [a, b, collision, r1, r2, u1, u2] = CollisionConeDetect(X_v,X_obs, V_v, V_obs, d, h)
% Relative Position
X_rel = X_obs - X_v;
V_rel = V_v - V_obs;
% Magnitude of Vehicle Velocity and Relative Position
X_rel_norm = norm(X_rel);
V_rel_norm = norm(V_rel);

% Static Obstacle Collsion Detection Using Vectors
c = sqrt((X_rel_norm^2-d^2)/(X_rel_norm^2*V_rel_norm^2-(dot(X_rel,V_rel))^2));
u1 = -1/X_rel_norm^2*(c*dot(X_rel,V_rel)+d)*X_rel+c*V_rel;
u2 =  1/X_rel_norm^2*(c*dot(X_rel,V_rel)-d)*X_rel-c*V_rel;
a = 1/2*(dot(X_rel,V_rel)/(X_rel_norm^2-d^2)+1/c/d);
b = 1/2*(dot(X_rel,V_rel)/(X_rel_norm^2-d^2)-1/c/d);
r1 = X_rel + d*u1;
r2 = X_rel + d*u2;

% Draw the vectors
DrawCollisionConeVectors(X_v, X_obs, V_v, V_obs, u1, u2, d, h);

% Collision Detection Criteria
a = round(a,4);
b = round(b,4);
if(a>0 && b>0)
    collision = 1;
else
    collision = 0;
end

end

