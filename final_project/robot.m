%%%%%%%%%%%%%
% ECE 3610
% Final Project
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Description 
%%%%%%%%%%%%%%

%% 1. CONNECT TO THE NANOBOT
clc
clear all

% for PC:
nb = nanobot('COM12', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

% ROBOT = 'R3'



%% Straight Line Test
mOffScale = 1.11;
motorBaseSpeed = 9;
m1Duty = mOffScale * motorBaseSpeed; % Right motor
m2Duty = motorBaseSpeed;

tic
nb.setMotor(1, mOffScale * 10);
nb.setMotor(2, 10);
pause(0.03);

while (toc < 3)
    nb.setMotor(1, m1Duty);
    nb.setMotor(2, -m2Duty);
end

% Turn off the motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Put the sensor array over a white background and find the expected min
% reflectance array values.

% Initialize the reflectance array.
nb.initReflectance();

% Average a few values 
avgVals = zeros(10, 6);
for i = 1:10
    read = nb.reflectanceRead();
    avgVals(i, 1) = read.one;
    avgVals(i, 2) = read.two;
    avgVals(i, 3) = read.three;
    avgVals(i, 4) = read.four;
    avgVals(i, 5) = read.five;
    avgVals(i, 6) = read.six;
end
minVals = [mean(avgVals(:,1)), mean(avgVals(:,2)), mean(avgVals(:,3)), ...
    mean(avgVals(:,4)), mean(avgVals(:,5)), mean(avgVals(:,6))];

fprintf(['Min Reflectance - one: %.2f, two: %.2f, three: %.2f four:' ...
    '%.2f five: %.2f six: %.2f\n'], ...
    minVals(1), minVals(2), minVals(3), minVals(4), minVals(5), minVals(6));

%% MAX REFLECTANCE VALUE CALIBRATION (all sensors over black tape)
% Put the sensor array over a black background and find the expected max
% reflectance array values.

% Initialize the reflectance array.
nb.initReflectance();

% Average a few values
avgVals = zeros(10, 6);
for i = 1:10
    read = nb.reflectanceRead();
    avgVals(i, 1) = read.one;
    avgVals(i, 2) = read.two;
    avgVals(i, 3) = read.three;
    avgVals(i, 4) = read.four;
    avgVals(i, 5) = read.five;
    avgVals(i, 6) = read.six;
end
maxVals = [mean(avgVals(:,1)), mean(avgVals(:,2)), mean(avgVals(:,3)), ...
    mean(avgVals(:,4)), mean(avgVals(:,5)), mean(avgVals(:,6))];
fprintf(['Max Reflectance - one: %.2f, two: %.2f, three: %.2f '...
    'four: %.2f five: %.2f six: %.2f\n'], ...
    maxVals(1), maxVals(2), maxVals(3), maxVals(4), maxVals(5), maxVals(6));

%% MAIN BEHAVIOR ------------------------------------------------------------
% Initialization
minVals = [93.8,84.2,74.0,71.6,83.00,108.4]; % Set me to min reflectance 
maxVals = [556.1,476.7,748.9,668.9,273.8,318.1]; % Set me to max reflectance 


mOffScale = 1.11;
motorBaseSpeed = 9;
m1Duty = mOffScale * motorBaseSpeed; % Right motor
m2Duty = motorBaseSpeed;

% MODE TOGGLE
mode = 'wall';   % 'line' or 'wall'
if strcmp(mode, 'wall')
    performWallFollow(nb, mOffScale, minVals, maxVals);
else

    %% 5. LINE FOLLOWING PID LOOP

    % First initialize the reflectance array.
    nb.initReflectance();

    % Get an initial reading
    vals = nb.reflectanceRead();

    % Set the motor offset factor (use the value you found earlier)
    mOffScale = 1.1;

    % TUNING:
    kp = 0.9;
    ki = 0.1;
    kd = 0.15;

    % Basic initialization
    vals = 0;
    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;
    checkpoints = 0;

    % Determine a threshold to detect when white is detected
    whiteThresh = 250;

    % The base duty cycle "speed" you wish to travel down the line
    motorBaseSpeed = 10;

    turn180Speed = 9; 
    turn180Time = 1.2; 

    tic % start time

    % Use a higher duty cycle briefly to overcome static friction
    nb.setMotor(1, mOffScale*11);
    nb.setMotor(2, 11);
    pause(0.03);

    while (toc < 30)

        % TIME STEP
        dt = toc - prevTime;
        prevTime = toc;

        % Take a reading
        vals = nb.reflectanceRead();

        % Change from a struct to a list for convenience
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
        
        % Calibrate sensor readings
        calibratedVals = zeros(1,6);
        for i = 1:6
            calibratedVals(i) = (vals(i) - minVals(i))/(maxVals(i) - minVals(i));
            if vals(i) < minVals(i) 
                calibratedVals(i) = 0;
            end
            if vals(i) > maxVals(i) 
                calibratedVals(i) = 1;
            end
        end

        fprintf(['Max Reflectance - one: %.2f, two: %.2f, three: %.2f '...
        'four: %.2f five: %.2f six: %.2f\n'], ...
        calibratedVals(1), calibratedVals(2), calibratedVals(3), calibratedVals(4), calibratedVals(5), calibratedVals(6));

        % Calculate the error
        extScalar = 10;
        midScalar = 4;
        innScalar = 2;
        rightWeight = 1.35;

        error = 6*extScalar*calibratedVals(1) + 1.3*midScalar*calibratedVals(2) + innScalar*(1-calibratedVals(3)) - ...
                rightWeight*innScalar*(1-calibratedVals(4)) - rightWeight*midScalar*calibratedVals(5) - 3*rightWeight*extScalar*calibratedVals(6);

        % Clamp error
        maxError = 6;
        if (error < -maxError)
            error = -maxError;
        elseif (error > maxError)
            error = maxError;
        end

        fprintf('Error: %.3f \n', error);
         
        if all(calibratedVals >= .8)
            if (~checkpoints)
                performTurn180(nb, turn180Speed, turn180Time, mOffScale, minVals, maxVals);
                prevError = 0;
                error = 0;
                integral = 0;
                derivative = 0;
            else
                nb.setMotor(1, 0); 
                nb.setMotor(2, 0);
                break;
            end
            checkpoints = checkpoints + 1;
            continue;
        end

        integral = integral + error*dt;
        derivative = (error - prevError) / dt;

        % PID output
        control = kp * error + ki * integral + kd * derivative;
        fprintf('ctrl: %.3f \n', control);

        % STATE CHECKING 
        if (vals(1) < whiteThresh && ...
                vals(2) < whiteThresh && ...
                vals(3) < whiteThresh && ...
                vals(4) < whiteThresh && ...
                vals(5) < whiteThresh && ...
                vals(6) < whiteThresh)

            % ALL SENSORS READ WHITE
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            break;

        else

            % LINE DETECTED
            m1Duty = mOffScale * max(0, motorBaseSpeed + min(control,0));
            m2Duty = -max(0, motorBaseSpeed - max(control,0));

            nb.setMotor(1, m1Duty);
            nb.setMotor(2, m2Duty);
        end

        prevError = error;
    end
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end

%% STOP
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% 5. DISCONNECT
clc
delete(nb);
clear('nb');
clear all

%% FUNCTIONS

function performTurn180(nb, turnSpeed, turnTime, mOffScale, minVals, maxVals) 
    fprintf('TURNING');

    % Briefly ignore sensor feedback at the start of the turn
    tic
    while(toc < 0.7)
        nb.setMotor(1, (mOffScale * turnSpeed)); 
        nb.setMotor(2, turnSpeed); 
    end

    % Continue turning until the inner sensors find the black line again
    while true
        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        calibratedVals = zeros(1,6);
        for i = 1:6
            calibratedVals(i) = (vals(i) - minVals(i))/(maxVals(i) - minVals(i));
            if vals(i) < minVals(i)
                calibratedVals(i) = 0;
            end
            if vals(i) > maxVals(i)
                calibratedVals(i) = 1;
            end
        end

        nb.setMotor(1, (mOffScale * turnSpeed)); 
        nb.setMotor(2, turnSpeed); 

        if calibratedVals(3) >= 0.3 || calibratedVals(4) >= 0.3
            break;
        end
    end

    nb.setMotor(1, 0); 
    nb.setMotor(2, 0); 
    pause(0.05); 
end
%% Print ultra sonic values
% Initialize ultrasonic sensors using your required API/pins
    nb.initUltrasonic1('D2','D3');   % front: trig D2, echo D3
    nb.initUltrasonic2('D4','D5');   % side: trig D4, echo D5
tic
while(toc < 5)
    fprintf('Front: %f3f \n', nb.ultrasonicRead1());
    %fprintf('Side: %f3f \n', nb.ultrasonicRead2());
end
%% 
function performWallFollow(nb, mOffScale, minVals, maxVals)
    fprintf('Beginning wall following...\n');

    % --- line-follow parameters
    nb.initReflectance();

    kp = 0.9;
    ki = 0.1;
    kd = 0.15;

    prevError = 0;
    prevTime = 0;
    integral = 0;

    motorBaseSpeed = 10;
    whiteThresh = 250;

    % --- wall-follow parameters
    frontStopDist = 175;    % tune this
    wallBaseSpeed = 8;
    wallKp = 0.35;         % tune this
    turnSpeed = 8;

    % Initialize ultrasonic sensors using your required API/pins
    nb.initUltrasonic1('D2','D3');   % front: trig D2, echo D3
    nb.initUltrasonic2('D4','D5');   % side: trig D4, echo D5

    %% Phase 1: follow line until all-black wall marker is reached
    fprintf('Following line until marker...\n');

    tic
    while true
        dt = toc - prevTime;
        prevTime = toc;

        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        calibratedVals = zeros(1,6);
        for i = 1:6
            calibratedVals(i) = (vals(i) - minVals(i))/(maxVals(i) - minVals(i));
            if vals(i) < minVals(i)
                calibratedVals(i) = 0;
            end
            if vals(i) > maxVals(i)
                calibratedVals(i) = 1;
            end
        end

        if all(calibratedVals >= 0.8)
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end

        extScalar = 10;
        midScalar = 4;
        innScalar = 2;
        rightWeight = 1.35;

        error = 6*extScalar*calibratedVals(1) + 1.3*midScalar*calibratedVals(2) + innScalar*(1-calibratedVals(3)) - ...
                rightWeight*innScalar*(1-calibratedVals(4)) - rightWeight*midScalar*calibratedVals(5) - 3*rightWeight*extScalar*calibratedVals(6);

        maxError = 6;
        if error < -maxError
            error = -maxError;
        elseif error > maxError
            error = maxError;
        end

        integral = integral + error*dt;
        derivative = (error - prevError) / dt;
        control = kp * error + ki * integral + kd * derivative;

        if (vals(1) < whiteThresh && vals(2) < whiteThresh && vals(3) < whiteThresh && ...
            vals(4) < whiteThresh && vals(5) < whiteThresh && vals(6) < whiteThresh)

            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            break;
        else
            m1Duty = mOffScale * max(0, motorBaseSpeed + min(control,0));
            m2Duty = -max(0, motorBaseSpeed - max(control,0));

            nb.setMotor(1, m1Duty);
            nb.setMotor(2, m2Duty);
        end

        prevError = error;
    end

    %% Phase 2: advance toward wall using front ultrasonic
    fprintf('Advancing to wall...\n');

    while true
        front = nb.ultrasonicRead1();

        if front <= frontStopDist
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end

        nb.setMotor(1, mOffScale * motorBaseSpeed);
        nb.setMotor(2, -motorBaseSpeed);
    end

    % LINE FOLLOW UNTIL HITTING THE WALL!!
    %% Phase 3: turn right ~90 degrees
    fprintf('Turning right 90 degrees...\n');

    tic
    while toc < 0.55   % tune this
        nb.setMotor(1, -mOffScale * turnSpeed);
        nb.setMotor(2, -turnSpeed);
    end
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
    pause(0.05);

    %% Phase 4: use side ultrasonic target distance
    left = nb.ultrasonicRead2();
    targetDist = left;

    %% Phase 5: follow wall until black line is found again
    while true
        left = nb.ultrasonicRead2();
        wallError = targetDist - left;
        wallControl = wallKp * wallError;

        % Only slow one side to turn
        rightCmd = wallBaseSpeed;
        leftCmd = wallBaseSpeed;

        if wallControl > 0
            % too close to wall -> turn away
            rightCmd = max(0, wallBaseSpeed - wallControl);
        else
            % too far from wall -> turn toward
            leftCmd = max(0, wallBaseSpeed + wallControl);
        end

        nb.setMotor(1, mOffScale * rightCmd);
        nb.setMotor(2, -leftCmd);

        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        calibratedVals = zeros(1,6);
        for i = 1:6
            calibratedVals(i) = (vals(i) - minVals(i))/(maxVals(i) - minVals(i));
            if vals(i) < minVals(i)
                calibratedVals(i) = 0;
            end
            if vals(i) > maxVals(i)
                calibratedVals(i) = 1;
            end
        end

        if all(calibratedVals >= 0.8)
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end
    end

    %% Phase 6: line follow back HOME
    kp = 0.9;
    ki = 0.1;
    kd = 0.15;

    prevError = 0;
    prevTime = 0;
    integral = 0;

    tic
    while true
        dt = toc - prevTime;
        prevTime = toc;

        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        calibratedVals = zeros(1,6);
        for i = 1:6
            calibratedVals(i) = (vals(i) - minVals(i))/(maxVals(i) - minVals(i));
            if vals(i) < minVals(i)
                calibratedVals(i) = 0;
            end
            if vals(i) > maxVals(i)
                calibratedVals(i) = 1;
            end
        end

        if all(calibratedVals >= 0.8)
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end

        extScalar = 10;
        midScalar = 4;
        innScalar = 2;
        rightWeight = 1.35;

        error = 6*extScalar*calibratedVals(1) + 1.3*midScalar*calibratedVals(2) + innScalar*(1-calibratedVals(3)) - ...
                rightWeight*innScalar*(1-calibratedVals(4)) - rightWeight*midScalar*calibratedVals(5) - 3*rightWeight*extScalar*calibratedVals(6);

        maxError = 6;
        if error < -maxError
            error = -maxError;
        elseif error > maxError
            error = maxError;
        end

        integral = integral + error*dt;
        derivative = (error - prevError) / dt;
        control = kp * error + ki * integral + kd * derivative;

        if (vals(1) < whiteThresh && vals(2) < whiteThresh && vals(3) < whiteThresh && ...
            vals(4) < whiteThresh && vals(5) < whiteThresh && vals(6) < whiteThresh)

            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            break;
        else
            m1Duty = mOffScale * max(0, motorBaseSpeed + min(control,0));
            m2Duty = -max(0, motorBaseSpeed - max(control,0));

            nb.setMotor(1, m1Duty);
            nb.setMotor(2, m2Duty);
        end

        prevError = error;
    end

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end