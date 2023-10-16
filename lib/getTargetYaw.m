function [Xd] = getTargetYaw(len, xc, yc, vx, vy, r, theta, omega)
dt = 0.01;
time = (1:len)'/100;
angle = [theta + omega * time, theta + omega * time + 0.5 * pi, theta + omega * time + pi, theta + omega * time + 1.5 * pi];
center = [xc + vx * time, yc + vy * time];
yaw = zeros(len, 1);
yaw_dot = zeros(len, 1);
for i = 1:len
    position = zeros(4,2);
    for j = 1:4
        position(j,:) = center(i,:) + [r * cos(angle(i, j)), r * sin(angle(i, j))];
    end
    index = 1;
    for j = 1:4
        if norm(position(j,:)) < norm(position(index,:))
            index = j;
        end
    end
    yaw(i) = atan2(position(index,2), position(index,1));
end
yaw_dot(1:len-1) = (yaw(2:len) - yaw(1:len-1)) / dt;
yaw_dot(len) = 0;
Xd = [yaw, yaw_dot]';
% Xd=yaw;
end

