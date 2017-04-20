encSub = rossubscriber('/accel');
Gx = 0;
Gy = 0;
Gz = 0;

% Define the distance between the wheel base
d = 0.254;

flat = .28; % Spooky magic number for flatness
slope = 100;

% Define the initial step-size
lambda = 1/16;

% Define the step-size multiplier
delta = 1.2;

% Define the angular velocity (rad/s)
omega = 0.3;

%ROS stuff
pub = rospublisher('/raw_vel'); %Sender
msgOut = rosmessage(pub);

count = 0;

while (abs(slope) > 0.01 || count <= 100)
    while 1
        encMessage = receive(encSub);
        if any(encMessage.Data)
            Gx = encMessage.Data(1);
            Gy = encMessage.Data(2);
            Gz = encMessage.Data(3);
            break;
        end
    end


    slope = sqrt((1-Gz^2)/Gz^2) -flat% Slope corrected for flatness
    rotAngle = atan(-Gy/Gx);
    
    %calculate the time it takes to rotate (sec)
    t_rot = (double(rotAngle/omega))

    %calculate the the time it takes to reach the second point (sec)
%     t_step = abs(double((slope*lambda)/(0.1*3.28)))
    t_step = .2;
    
    
    V_rot = -sign(t_rot)*omega*d/2;
    
    Vl = -V_rot;
    Vr = V_rot;

    %sending the wheel velocities to the NEATO
    msgOut.Data = [double(Vl), double(Vr)];
    [double(Vl), double(Vr)];
    send(pub, msgOut);

    %stopping the NEATO after it rotates the appropriate angle
    pause(abs(t_rot));

    %making the NEATO move straight
    Vl = 0.12;
    Vr = 0.12;

    msgOut.Data = [double(Vl), double(Vr)];
    [double(Vl), double(Vr)];
    send(pub, msgOut);

    pause(t_step);
    lambda = lambda.*delta;
    
    msgOut.Data = [0, 0];
    send(pub, msgOut);
    pause(.2);
    count = count + 1
end

