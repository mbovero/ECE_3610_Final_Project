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


%% STOP
nb.setMotor(1, 0);
nb.setMotor(2, 0);

% 5. DISCONNECT
clc
delete(nb);
clear('nb');
clear all
%% Turn 180
nb.setMotor(1, 0);
nb.setMotor(2, 0);

minVals = [93.8,84.2,74.0,71.6,83.00,108.4]; % Set me to min reflectance 
maxVals = [556.1,476.7,748.9,668.9,273.8,318.1]; % Set me to max reflectance 
mOffScale = 1.11;
turn180Speed = 10; 
turn180Time = 1.5; 
CCW = 1;
performTurn180(nb, turn180Speed, turn180Time, minVals, maxVals, CCW);

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
mode = 'wall';   % 'line' or 'wall' or 'color'
if strcmp(mode, 'wall')
    performWallFollow(nb, mOffScale, minVals, maxVals);
elseif strcmp(mode, 'color')
    performColorFind(nb, mOffScale, minVals, maxVals);
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

    turn180Speed = 10; 
    turn180Time = 1; 

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
                performTurn180(nb, turn180Speed, turn180Time, minVals, maxVals, 0);
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


%% FUNCTIONS

function performTurn180(nb, turnSpeed, turnTime, minVals, maxVals, CCW)
    fprintf('TURNING\n');
    mOffScale = 1;


    % Briefly ignore sensor feedback at the start of the turn
    tic
    while(toc < turnTime)
        if (CCW == 1)
            nb.setMotor(1, (mOffScale * turnSpeed)); 
            nb.setMotor(2, turnSpeed); 
        else
            nb.setMotor(1, -(mOffScale * turnSpeed)); 
            nb.setMotor(2, -turnSpeed); 
        end 
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

        if (CCW == 1)
            nb.setMotor(1, (mOffScale * turnSpeed)); 
            nb.setMotor(2, turnSpeed); 
        else
            nb.setMotor(1, -(mOffScale * turnSpeed)); 
            nb.setMotor(2, -turnSpeed); 
        end

        if calibratedVals(3) >= 0.3 || calibratedVals(4) >= 0.3
            fprintf('Line found, turn sequence stopped.\n')
            break;
        end
    end

    nb.setMotor(1, 0); 
    nb.setMotor(2, 0); 
    pause(0.05); 
end

%% 
function performColorFind(nb, mOffScale, minVals, maxVals)
% --- line-follow parameters
    nb.initReflectance();

    kp = 0.9;
    ki = 0.2;
    kd = 0.15;

    prevError = 0;
    prevTime = 0;
    integral = 0;

    motorBaseSpeed = 10;
    whiteThresh = 250;

%% Phase 1: follow line until red marker is reached
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

    %% Phase 2: identify color and rotate accordingly
    %Initialize the RGB color sensor
    nb.initColor();
    %Take a single RGB color sensor reading
    values = nb.colorRead();
    
    %The sensor values are saved as fields in a structure:
    red = values.red;
    green = values.green;
    blue = values.blue;
    fprintf('red: %.2f, green: %.2f, blue: %.2f\n', red, green, blue);

    if (red > 170)
        CCW = 1;
        fprintf('RED: CCW\n');
    elseif (green > 100)
        CCW = 0;
        fprintf('BLUE: CW\n');

    end

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
    turn180Speed = 10.5; 
    turn180Time = 2; 
    performTurn180(nb, turn180Speed, turn180Time, minVals, maxVals, CCW);

    %% Phase 3: follow line home
    fprintf('Following line home...\n');

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
end

%% Print ultra sonic values
% Initialize ultrasonic sensors using your required API/pins
    nb.initUltrasonic1('D2','D3');   % front: trig D2, echo D3
    nb.initUltrasonic2('D4','D5');   % side: trig D4, echo D5
tic
while(toc < 5)
    %fprintf('Front: %f3f \n', nb.ultrasonicRead1());
    fprintf('Side: %f3f \n', nb.ultrasonicRead2());
end




%% 
function performWallFollow(nb, mOffScale, minVals, maxVals)
    fprintf('Beginning wall following...\n');

    % --- line-follow parameters
    nb.initReflectance();

    kp = 0.9;
    ki = 0.2;
    kd = 0.15;

    prevError = 0;
    prevTime = 0;
    integral = 0;

    motorBaseSpeed = 10;
    whiteThresh = 250;

    % --- wall-follow parameters
    frontStopDist = 350;    % tune this
    targetDist = 300;
    wallBaseSpeed = 11;
    turnSpeed = 11;

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

    %% Phase 2: line follow until reaching wall distance
    fprintf('Line following to wall...\n');
    
    nb.setMotor(1, mOffScale * 9);
    nb.setMotor(2, -9);
    pause(0.2);

    error = 0;
    prevError = 0;
    prevTime = 0;
    integral = 0;
    
    while true
        front = nb.ultrasonicRead1();
    
        if front <= frontStopDist
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end
    
        % TIME STEP
        dt = toc - prevTime;
        prevTime = toc;
    
        % Reflectance read
        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
    
        % Calibrate sensor readings
        calibratedVals = zeros(1,6);
        for i = 1:6
            calibratedVals(i) = (vals(i) - minVals(i)) / (maxVals(i) - minVals(i));
            if vals(i) < minVals(i)
                calibratedVals(i) = 0;
            end
            if vals(i) > maxVals(i)
                calibratedVals(i) = 1;
            end
        end
    
        % Same line-following error as before
        extScalar = 10;
        midScalar = 4;
        innScalar = 2;
        rightWeight = 1.35;
    
        error = 6*extScalar*calibratedVals(1) + 1.3*midScalar*calibratedVals(2) + innScalar*(1-calibratedVals(3)) - ...
                rightWeight*innScalar*(1-calibratedVals(4)) - rightWeight*midScalar*calibratedVals(5) - 3*rightWeight*extScalar*calibratedVals(6);
    
        % Clamp error
        maxError = 6;
        if error < -maxError
            error = -maxError;
        elseif error > maxError
            error = maxError;
        end
    
        % PID
        integral = integral + error * dt;
        derivative = (error - prevError) / dt;
        control = kp * error + ki * integral + kd * derivative;
    
        % Motor command: line follow while advancing
        m1Duty = mOffScale * max(0, motorBaseSpeed + min(control,0));
        m2Duty = -max(0, motorBaseSpeed - max(control,0));
    
        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
    
        prevError = error;
    end
    %% Phase 3: turn right ~90 degrees
    fprintf('Turning right 90 degrees...\n');

    tic
    while toc < 0.65   % tune this
        nb.setMotor(1, -mOffScale * turnSpeed);
        nb.setMotor(2, -turnSpeed);
    end
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
    pause(0.05);


    %% Phase 4: follow wall until black line is found again
    fprintf('Following wall...\n');
    
    prevWallError = 0;
    wallIntegral = 0;
    prevWallTime = 0;
    
    wallKp = 0.2;
    wallKi = 0.0;
    wallKd = 0.2;
    
    tic
    while true
        dt = toc - prevWallTime;
        prevWallTime = toc;
    
        left = nb.ultrasonicRead2();
        wallError = targetDist - left;
        if (left == 0)
            wallError = -3500;
        elseif (wallError < -2000)
            wallError = -2000;
        elseif (wallError > 2000)
            wallError = 2000;
        end
        fprintf('wallError: %0.3f\n', wallError);

    
        wallIntegral = wallIntegral + wallError * dt;
        wallDerivative = 0;
        if dt > 0
            wallDerivative = (wallError - prevWallError) / dt;
        end
    
        wallControl = wallKp * wallError + wallKi * wallIntegral + wallKd * wallDerivative;


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
            calibratedVals(i) = (vals(i) - minVals(i)) / (maxVals(i) - minVals(i));
            if vals(i) < minVals(i)
                calibratedVals(i) = 0;
            end
            if vals(i) > maxVals(i)
                calibratedVals(i) = 1;
            end
        end
    
        if (calibratedVals(3) >= 0.7 || calibratedVals(4) >= 0.7)
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end
    
        prevWallError = wallError;
    end

    %% Phase 5: line follow back HOME
    fprintf('Line following to start...\n');

    turn90Speed = 9; 
    turn90Time = 0.5; 
    performTurn180(nb, turn90Speed, turn90Time, minVals, maxVals, 1);



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