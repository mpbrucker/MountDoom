clear;

% % Arbitrary speed constant
d = .25*3.28; % Wheelbase of robot (m)

syms x(t) y(t);
mtoFT = 1/3.28;
a = .045*t
xT = (y-(2)*x-(2));
yT = (x-(2)*y-(2));

% a = .1/sqrt(xT.^2 + yT.^2)

ode1 = diff(x) == a*xT
ode2 = diff(y) == a*yT
odes = [ode1; ode2];

cond1 = x(0) == 4;
cond2 = y(0) == 1;
conds = [cond1; cond2];
[xFunc(t), yFunc(t)] = dsolve(odes,conds)

% tFinal = double(solve(xFunc(t) == -2, t))

t_vals = linspace(0, 10, 100);

plot(xFunc(t_vals), yFunc(t_vals));
hold on
axis([-2.5 5 -2.5 5]);

r = [xFunc yFunc 0]; % Vector function
vel = simplify(diff(r, t)); % Derivative of position function
spd = simplify(norm(vel));

tHat = simplify(vel/spd) % Unit tangent vector
N = simplify(diff(tHat, t)) % Normal vector (not normalized)
omega = simplify(cross(tHat, N)) % Calculate angular velocity as a function of time


pub = rospublisher('raw_vel');
msg = rosmessage(pub);


startTime = tic;
time = 0; % Elapsed time in the loop
    
robotOrigin = [4; 1]; % Values for calculating the position of the robot based on VL and VR
calcTheta = 0;
curTime = tic;
loopNum = 0;

plot(-2, -2, 'k*');
hold on;

while (norm(robotOrigin-[-2; -2]) > 0.1)
    plot(robotOrigin(1), robotOrigin(2), 'ro');
    loopTime = toc(curTime); % Value of deltaT (s)
    curTime = tic;
    
    
    time = toc(startTime); % Current time (s)
    curSpd = double(spd(time)); % Current spd(m/s)
    omegaCalc = double(omega(time)); % Current angular velocity(rad/s)
    curOmega = -omegaCalc(3);
    R = curSpd/curOmega;
    VL = (R+(d/2))*curOmega; % Speed of left wheel (m/s)
    VR = (R-(d/2))*curOmega; % Speed of right wheel (m/s)
    
    VL*mtoFT
    VR*mtoFT
    
    msg.Data = [VL*mtoFT, VR*mtoFT];
    send(pub, msg)
%     
    omegaComp = -(VL-VR)/d; % Computed angular velocity (rad/s)
    deltTheta = omegaComp*loopTime; % Angle swept around ICC (delta theta)
    rotMatrix = [cos(deltTheta) -sin(deltTheta); sin(deltTheta) cos(deltTheta)]; % Rotation matrix
    translationMatrix = robotOrigin+[-sin(calcTheta)*R; cos(calcTheta)*R]; % Translation matrix
    if (loopNum == 1) % This is a flag to avoid a large jump in theta at the beginning
        robotOrigin = (rotMatrix*(robotOrigin-translationMatrix))+translationMatrix; % Perform rotation of robot position around ICC
        calcTheta = calcTheta + deltTheta;
    end
    loopNum = 1;
%     axis([-2.5 5 -2.5 5]);
    hold on;


end
display(time);
msg.Data = [0, 0];
send(pub, msg);

