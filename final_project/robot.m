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
%% 
nb.setMotor(2, m1Duty);
%% 
% Turn off the motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Line Following
% First initialize the reflectance array.
nb.initReflectance();

% Get an initial reading
vals = nb.reflectanceRead();

% Set the motor offset factor (use the value you found earlier)
mOffScale = 1.1;
w
% TUNING:
% Start very small. (Using reflectance values, the calculated error can  
% range from zero to several thousand! Think about what coefficient you 
% want multiplying by values that could get up to a thousand or so.)
kp = 1.5;
ki = 0;
kd = 0.00;

% Basic initialization
maxVals = [846.9,740.7,549.8,432.8,416.8,454.6]; % Set me to max reflectance 
minVals = [74.2,51.2,38.4,26.7,26.7,38.4]; % Set me to min reflectance 

vals = 0;
prevError = 0;
prevTime = 0;
integral = 0;
derivative = 0;

% Determine a threshold to detect when white is detected 
% (choose a value that will be used as a threshold by all sensors to know 
% if the robot has lost the line)
whiteThresh = 300; % Max value detected for all white

% The base duty cycle "speed" you wish to travel down the line 
% (recommended value is 9)
motorBaseSpeed = 10;

tic % start time

fprintf("initialized\n")

% Use a higher duty cycle for a very brief moment to overcome the gearbox 
% force of static friction 
% (recommendation: 10, with mOffScale if needed)
nb.setMotor(1, 12*mOffScale);
nb.setMotor(2, 12);
pause(0.03);

while (toc < 5)  % Adjust me if you want to stop your line following 
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
            calibratedVals(i) = maxVals(i);
        end
    end

    % Calculate the three errors to be used in the PID control 
    
    error = 0.5*calibratedVals(1) + 1*calibratedVals(2) + 1*calibratedVals(3) - ...
            1*calibratedVals(4) - 1*calibratedVals(5) - 0.5*calibratedVals(6); % Designing this error term can sometimes be just as 
                 % important as the tuning of the feedback loop (how you  
                 % set the PID controller output). In the last lab, this  
                 % error was a simple difference between two values (RPM  
                 % and the target RPM). Here, you're working with 6 sensors  
                 % that are located in different positions. You might want 
                 % to Look back at how you calculated the error in the
                 % Sensors 5 lab. Don't forget here to use the calibrated 
                 % values to calculate the error.  
    fprintf("three: " + 1*calibratedVals(3) + "\n");
    fprintf("four: " + calibratedVals(4) + "\n");
    
    EScaler = 0.0075;
    error = EScaler * error;

    fprintf("Error:" + error + "\n");
    integral = integral + error*dt;

    derivative = (prevError - error) / dt;

    % Create your PID controller output here using the previously defined 
    % gain values and the three errors computed above. 
    control = kp * error + ki * integral + kd * derivative;
    fprintf("Control: " + control + "\n");

    % STATE CHECKING 
    if (vals(1) < whiteThresh && ...
            vals(2) < whiteThresh && ...
            vals(3) < whiteThresh && ...
            vals(4) < whiteThresh && ...
            vals(5) < whiteThresh && ...
            vals(6) < whiteThresh)
        
        fprintf("LOST LINE")

        % ALL SENSORS READ WHITE (lost tracking):
        nb.setMotor(1, 0); % stop the motors
        nb.setMotor(2, 0);
        break; % exit the while loop

    else
        fprintf("LINE DETECTED\n")

        % LINE DETECTED:
        % We want to travel at a fixed speed down the line and the control 
        % should make minor adjustments that allow the robot to stay 
        % centered on the line as it moves. These should be equations and
        % not just numbers:
        m1Duty = mOffScale * (motorBaseSpeed + control);
        m2Duty = -(motorBaseSpeed - control);

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
        fprintf("m1Duty: " + m1Duty + "\n")
        fprintf("m2Duty: " + m2Duty + "\n")
    end

    prevError = error;
end
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all