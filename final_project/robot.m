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

% Initialization
mOffScale = 1.3;
motorBaseSpeed = 11;
m1Duty = mOffScale * motorBaseSpeed; % Right motor
m2Duty = motorBaseSpeed;
%% Straight Line Test

tic
nb.setMotor(1, mOffScale * 10);
nb.setMotor(2, 10);
pause(0.03);

while (toc < 3) % adjust the time to test if robot goes in straight line
                % (shorter time saves battery; longer tests longer path)
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

%Average a few values 
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
minReflectance = [85.2,79.2,70.3,61.2,68,87.8]; % Set me to min reflectance 
                                             % values for each sensor for
                                             % future reference
%% MAX REFLECTANCE VALUE CALIBRATION (all sensors over black tape)
% Put the sensor array over a black background and find the expected max
% reflectance array values.

% Initialize the reflectance array.
nb.initReflectance();

%Average a few values
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
maxReflectance = [727.6,527.4,390.6,357,420.7,646.9]; % Set me to max reflectance 
                                             % values for each sensor for
                                             % future reference

%% 5.  LINE FOLLOWING PID LOOP
% Though PID tuning can be tedious and frustrating at times, the payoff is
% often worth it! A well-tuned PID system can be surprisingly robust.
% Good luck, and don't hesitate to ask for help if you're stuck.

% 1st Note:  In the last lab, you used a PID controller to control the  
% motor speed (using information from the motor encoder).  In this lab, you 
% will be using a PID controller to help the robot follow the line (using 
% information from the IR sensing array).  As a result, you will not be 
% able to just copy and paste your exact PID code from the last lab to 
% this lab. 

% 2nd Note: Make sure you only use the values produced by the reflectance 
% array when the red LEDs on the underside of the reflectance array are on!
% When your battery is getting low, the red LEDs on the underside of the 
% reflectance array will turn off, and the numbers you get from the 
% reflectance array will no longer be reliable. If you suspect that your 
% battery is dead, grab an instructor's attention and they will swap your 
% battery for a fully charged one.

% Depending on the wiring, if you used a negative sign for any of the motor
% duty cycles in section 3 above, make sure to implement those here as well.

% First initialize the reflectance array.
nb.initReflectance();

% Get an initial reading
vals = nb.reflectanceRead();

% Set the motor offset factor (use the value you found earlier)
mOffScale = 1.1;

% TUNING:
% Start very small. (Using reflectance values, the calculated error can  
% range from zero to several thousand! Think about what coefficient you 
% want multiplying by values that could get up to a thousand or so.)
kp = 0.9;
ki = 0.3;
kd = 0.13;

% Basic initialization
vals = 0;
prevError = 0;
prevTime = 0;
integral = 0;
derivative = 0;
checkpoints = 0;

% Determine a threshold to detect when white is detected 
% (choose a value that will be used as a threshold by all sensors to know 
% if the robot has lost the line)
whiteThresh = 250; % Max value detected for all white

% The base duty cycle "speed" you wish to travel down the line 
% (recommended value is 9)
motorBaseSpeed = 9;

turn180Speed = 11; 
turn180Time = 1.2; 

tic % start time

% Use a higher duty cycle for a very brief moment to overcome the gearbox 
% force of static friction 
% (recommendation: 10, with mOffScale if needed)
nb.setMotor(1, mOffScale*11);
nb.setMotor(2, 11);
pause(0.03);

while (toc < 30)  % Adjust me if you want to stop your line following 
                 % earlier or let it run longer.

    % TIME STEP
    dt = toc - prevTime; % find the amount of time that has elapsed

    prevTime = toc; % set the current time to the previous time in the 
                    % next time step

    % take a reading                
    vals = nb.reflectanceRead();

    % change from a struct to a list for convenience
    vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
    
    % Calibrate sensor readings
    calibratedVals = zeros(1,6); % initialize to zero
    for i = 1:6
        calibratedVals(i) = (vals(i) - minVals(i))/(maxVals(i) - minVals(i));
        % overwrite the calculated calibrated values if get a reading below 
        % or above our set minVals (white) or maxVals (black), respectively
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

    % Calculate the three errors to be used in the PID control 
    
    extScalar = 6;
    midScalar = 2;
    innScalar = 2;
    rightWeight = 1.35;

    error = 3*extScalar*calibratedVals(1) + 1.3*midScalar*calibratedVals(2) + innScalar*(1-calibratedVals(3)) - ...
            rightWeight*innScalar*(1-calibratedVals(4)) - rightWeight*midScalar*calibratedVals(5) - 2*rightWeight*extScalar*calibratedVals(6);
                 % Designing this error term can sometimes be just as 
                 % important as the tuning of the feedback loop (how you  
                 % set the PID controller output). In the last lab, this  
                 % error was a simple difference between two values (RPM  
                 % and the target RPM). Here, you're working with 6 sensors  
                 % that are located in different positions. You might want 
                 % to Look back at how you calculated the error in the
                 % Sensors 5 lab. Don't forget here to use the calibrated 
                 % values to calculate the error.  
    % Clamp error
    if (error < -5)
        error = -5;
    elseif (error > 5)
        error = 5;
    end

    fprintf('Error: %.3f \n', error);
     



    if all(calibratedVals >= .8) % ---new line ---
        if (~checkpoints)
            performTurn180(nb, turn180Speed, turn180Time, mOffScale); % ---new line ---
            prevError = 0; % ---new line ---
            error = 0;
            integral = 0; % ---new line ---
            derivative = 0;
        else
            nb.setMotor(1, 0); 
            nb.setMotor(2, 0);
            break;
        end
        checkpoints = checkpoints + 1;
        continue; % ---new line ---
    end % ---new line ---


    integral = integral + error*dt;

    derivative = (error - prevError) / dt;

    % Create your PID controller output here using the previously defined 
    % gain values and the three errors computed above. 
    control = kp * error + ki * integral + kd * derivative;
    fprintf('ctrl: %.3f \n', control);


    % STATE CHECKING 
    if (vals(1) < whiteThresh && ...
            vals(2) < whiteThresh && ...
            vals(3) < whiteThresh && ...
            vals(4) < whiteThresh && ...
            vals(5) < whiteThresh && ...
            vals(6) < whiteThresh)

        % ALL SENSORS READ WHITE (lost tracking):
        nb.setMotor(1, 0); % stop the motors
        nb.setMotor(2, 0);
        break; % exit the while loop

    else

        % LINE DETECTED:
        % We want to travel at a fixed speed down the line and the control 
        % should make minor adjustments that allow the robot to stay 
        % centered on the line as it moves. These should be equations and
        % not just numbers:
        m1Duty = mOffScale * (motorBaseSpeed + control);
        m2Duty = -(motorBaseSpeed - control);

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
    end

    prevError = error;
end
nb.setMotor(1, 0);
nb.setMotor(2, 0);
%% 
performTurn180(nb, 10, 1.35, 1.1);
%% STOP
nb.setMotor(1, 0); % Right motor
nb.setMotor(2, 0); % Left motor

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all

%%
function performTurn180(nb, turnSpeed, turnTime, mOffScale) 
    fprintf('TURNING');
    tic
    while(toc < 0.5)
        nb.setMotor(1, 0); 
        nb.setMotor(2, 0); 
    end

    tic 
    while toc < turnTime 
        nb.setMotor(1, (mOffScale * turnSpeed)); 
        nb.setMotor(2, turnSpeed); 
    end 
    nb.setMotor(1, 0); 
    nb.setMotor(2, 0); 
    pause(0.05); 
end 