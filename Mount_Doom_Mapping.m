load('encoders.mat');

dataset(492,:) = [];

t = dataset(:,1);
LPos = dataset(:,2);
RPos = dataset(:,3);
Gx = dataset(:,4);
Gy = dataset(:,5);
Gz = dataset(:,6);

d = 0.254; % Length of wheel base
yaw = 0; % Initial yaw
flat = .28; % Flat value
pos = [0;0;0]

posList = [0; 0; 0];
for i=2:size(t,1)
    roll = atan(Gy(i)/Gz(i)); % Calculate roll
    pitch = atan(-Gx(i)/sqrt(Gy(i)^2+Gz(i)^2))+flat;
    dt = t(i)-t(i-1); % Change in time
    dL = LPos(i)-LPos(i-1);
    dR = RPos(i)-RPos(i-1);
    VL = dL/dt; % Current L velocity
    VR = dR/dt; % Current R velocity

    
    omega = (VR-VL)/d; % Angular velocity
    deltaTheta = omega*dt; % Change in yaw
    yaw = yaw + deltaTheta;
    
    Rx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
    
    rot = Rx*Ry*Rz;
    orientation = inv(rot);
    N = vrrotvec2mat([orientation(:,3)' deltaTheta]);
    
    R = d/2*((dL+dR)/(dR-dL+.000001)); % Radius of curvature
    posICC = orientation*[0; R; 0]+pos; % Position of ICC
    
    pos = N*(pos-posICC)+posICC; % New position
    posList(:, i) = pos;
    clf;
    plot3(-posList(1,:), posList(2,:), posList(3,:), 'k');
    hold on;
%     scatter3(-posICC(1), posICC(2), posICC(3), 'k');
    quiver3(-pos(1), pos(2), pos(3), -orientation(1,1), orientation(2, 1), orientation(3, 1), 'r'); % X
    quiver3(-pos(1), pos(2), pos(3), -orientation(1,2), orientation(2, 2), orientation(3, 2), 'g'); % Y
    quiver3(-pos(1), pos(2), pos(3), -orientation(1,3), orientation(2, 3), orientation(3, 3), 'b'); % Z
    axis([-1 1 -1 1 -1 1]);
    drawnow;
    if (norm(posList(:,i)-posList(:,i-1)) > .1)
        display(VL);
        display(VR);
        display(i);
    end
end