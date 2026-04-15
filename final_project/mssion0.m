%%%%%%%%%%%%%
% ECE 3610
% Final Project
%%%%%%%%%%%%%

clc
clear all
%% Train Neural Network For Gesture Classification
% for PC:
nb = nanobot('COM10', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

nb.ledWrite(0); % make sure the LED is off


clear; clc; close all; %initialization
filename = "2026413_15457_TrainingSet_2Digits8Trials.mat";  % add the directory before the filename 
                                 % if needed
data = importdata(filename);


%Calculate features for each "image"
%determine digitCount and trialCount based on data size
digitCount = height(data); %number of digits is the number of rows (height)
trialCount = width(data)-1; %number of trials is the number of columns (width)
Features = zeros(digitCount, trialCount, 3); % 3 because the accelerometer sends 3 axes of data
for a = 1:digitCount %iterate through all digits
    for b = 1:trialCount %iterate through all trials
        singleLetter = data{a,b+1}; %get the individual gesture data   
        
        x = singleLetter(1,:);
        y = singleLetter(2,:);
        z = singleLetter(3,:);
        mag = sqrt(x.^2 + y.^2 + z.^2);
        

        Features(a,b,1) = mean(y); % YOU SHOULD MODIFY THIS LINE
        Features(a,b,2) = std(y); % YOU SHOULD MODIFY THIS LINE
        Features(a,b,3) = std(z); % YOU SHOULD MODIFY THIS LINE

    end
end


% Store Data as at Stack for Input to Neural Network
% Features are stored as a stack in a 4D array (b/c the MATLAB function 
% requries a 4D array as input; we are only using the 1st and 4 dimensions)
% Initialize to zero.
TrainingFeatures = zeros(3,1,1,digitCount*trialCount); 
%labels are stored as a 1D array, initialize to zero
labels = zeros(1,digitCount*trialCount); 

k=1; %simple counter
for a = 1:digitCount %iterate through digits
    for b = 1:trialCount %iterate through trials
        TrainingFeatures(:,:,:,k) = Features(a,b,:); %put each feature into image stack
        labels(k) = data{a,1}; %put each label into label stack
        k = k + 1; %increment
    end
end
labels = categorical(labels); %convert labels into categorical


% Split Training and Testing Data
%selection is an array that will hold the value of 1 when that data is 
%selected to be training data and a 0 when that data is selected to be 
%testing data
selection = ones(1,digitCount*trialCount); %allocate logical array
                                             %initialize all to 1 at first
selectionIndices = []; %initialization
for b = 1:digitCount %pick 1/4 of the data for testing
    selectionIndices = [selectionIndices,  round(linspace(1,trialCount,...
        round(trialCount/4))) + (trialCount*(b-1))];
end
selection(selectionIndices) = 0; %set logical to zero to indicate testing 
                                 %data

%training data
xTrain = TrainingFeatures(:,:,:,logical(selection)); %get subset (3/4) of features to train on
yTrain = labels(logical(selection)); %get subset (3/4) of labels to train on
%testing data
xTest = TrainingFeatures(:,:,:,~logical(selection)); % get subset (1/4) of features to test on
yTest = labels(~logical(selection)); %get subset (1/4) of labels to test on


% Define Neural Network

[inputsize1,inputsize2,~] = size(TrainingFeatures); %input size is defined by features
numClasses = length(unique(labels)); %output size (classes) is defined by number of unique labels

%%%%%%%%%%%%%%%%%%%%% YOU SHOULD MODIFY THESE PARAMETERS %%%%%%%%%%%%%%%%%%%

learnRate = .1; % how quickly network makes changes and learns
maxEpoch = 400; % how long the network learns (how many times all the data 
               % is passed through the CNN)

%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%%

layers= [ ... %NN architecture for a simple perceptron
    imageInputLayer([inputsize1,inputsize2,1])
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
    ];

options = trainingOptions('sgdm','InitialLearnRate', learnRate, ...
    'MaxEpochs', maxEpoch, 'Shuffle','every-epoch','Plots', ...
    'training-progress', 'ValidationData',{xTest,yTest}); %options for NN


% Train Neural Network
% This section will create a plot of the training progress.  Before going 
% on to the next section, check the validation accuracy. If the results 
% don't look good yet, go back to the previous section and adjust the 
% learnRate and maxEpoch. 
[myNeuralNetwork,info] = trainNetwork(xTrain,yTrain,layers,options); 
                         %output is the trained NN







%% Run-Time Predictions 
% (This is essentially a copy of the lab 1 code with determination 
% replaced with NN code.)
% Perform a gesture and see if the perceptron classification works! 
% Rerun this part of the code and try testing all digits you are trying 
% to classify.

% make sure NN exists
if(~exist('myNeuralNetwork'))
    error("You have not yet created your neural network! Be sure you" + ...
        " run this section AFTER your neural network is created.");
end

% clear the old singleLetter and nb
clear nb singleLetter;

%%%%%%%%%%%%%%%%%%%%%%%%% RECONNECT TO ARDUINO %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add your port below (same as at the beginning of the code)
% for PC:
nb = nanobot('COM10', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nb.ledWrite(0); % turn off the LED

numreads = 150; % about 2 seconds (on serial); adjust as needed, but we 
                % will be using a value of 150 for Labs 4 and 5
pause(.5);

clc; % clear the command line
countdown("Beginning in", 3);
disp("Make A Gesture!");
nb.ledWrite(1);  % Turn on the LED to signify the start of recording data

% Gesture is performed during the segement below
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end

nb.ledWrite(0); % Turn the LED off to signify end of recording data

singleLetter = [vals(1,:);vals(2,:);vals(3,:)];

% put accelerometer data into NN input form
xTestLive = zeros(3,1,1,1); %allocate the size

%%%%%%%%%%%%%%%%%%%%% YOU SHOULD MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%

xTestLive(1,:,:,1) = mean(vals(2,:)); % y mean
xTestLive(2,:,:,1) = std(vals(2,:)); % y std
xTestLive(3,:,:,1) = std(vals(3,:)); % z std

%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%%

% Prediction based on NN
prediction = classify(myNeuralNetwork,xTestLive);
fprintf('%s\n', string(prediction));


% CONNECT TO THE NANOBOT
% for PC:
nb = nanobot('COM12', 115200, 'serial');
% for Mac:
% nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

% MAIN SETUP --------------------------------------------------------------
minVals = [93.8,84.2,74.0,71.6,83.00,108.4];
maxVals = [556.1,476.7,748.9,668.9,273.8,318.1];

mOffScale = 1.11;
motorBaseSpeed = 9;

%
if strcmp(string(prediction), '0')
    performMission0Abstracted(nb, mOffScale, minVals, maxVals);
else
    performMission1Abstracted(nb, mOffScale, minVals, maxVals);
end

%% OPTIONAL MANUAL STOP / DISCONNECT
nb.setMotor(1, 0);
nb.setMotor(2, 0);%
delete(nb);%

clear('nb');


%% ========================================================================
%% ABSTRACTED MISSION 1
%% ========================================================================
function performMission1Abstracted(nb, mOffScale, minVals, maxVals)
    fprintf('ABSTRACTED MISSION 1 START\n');

    nb.initReflectance();

    % Tunable values
    straightSpeed = 9;
    linePause = 0.15;
    homeCrossTime = 0.25;
    ignoreHorizontalAfterWall = 1.0;

    % Turn tuning
    turnSpeed = 10;
    ignoreTurn90Time = 0.5;   % shorter ignore time for "90" style turns
    ignoreTurn180Time = 1.00; % same idea as your original 180

    %% Start on a path and line follow until the first horizontal black line = HOME
    fprintf('Following line to HOME...\n');
    lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, 0, linePause);

    %% FIRST: RIGHT into WALL PATH
    fprintf('At HOME -> turning RIGHT into WALL path\n');
    turnRight90(nb, minVals, maxVals, turnSpeed, ignoreTurn90Time);
    driveStraightForTime(nb, mOffScale, straightSpeed, homeCrossTime);
    followWallPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause, ignoreHorizontalAfterWall);

    %% SECOND: LEFT PATH
    fprintf('Back at HOME -> go STRAIGHT\n');
    driveStraightForTime(nb, mOffScale, straightSpeed, homeCrossTime);
    followLeftPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause, turnSpeed, ignoreTurn180Time);

    %% THIRD: RIGHT INTO COLOR PATH
    fprintf('Back at HOME -> turning RIGHT into COLOR path\n');
    turnLeft90(nb, minVals, maxVals, turnSpeed, ignoreTurn90Time);
    followColorPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause);

    %% FINAL STOP AT HOME
    fprintf('MISSION 1 COMPLETE AT HOME\n');
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end



%% ========================================================================
%% ABSTRACTED MISSION 0
%% ========================================================================
function performMission0Abstracted(nb, mOffScale, minVals, maxVals)
    fprintf('ABSTRACTED MISSION 0 START\n');

    nb.initReflectance();

    % Tunable values
    straightSpeed = 9;
    linePause = 0.15;
    homeCrossTime = 0.18;
    ignoreHorizontalAfterWall = 1.0;

    % Turn tuning
    turnSpeed = 10;
    ignoreTurn90Time = 0.5;   % shorter ignore time for "90" style turns
    ignoreTurn180Time = 1.00;  % same idea as your original 180


    %% Start on a path and line follow until the first horizontal black line = HOME
    fprintf('Following line to HOME...\n');
    lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, 0, linePause);

    %% LEFT PATH
    fprintf('At HOME -> turning LEFT\n');
    turnLeft90(nb, minVals, maxVals, turnSpeed, ignoreTurn90Time);
    followLeftPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause, turnSpeed, ignoreTurn180Time);

    %% CROSS HOME INTO WALL PATH
    fprintf('Back at HOME -> crossing HOME into WALL path\n');
    driveStraightForTime(nb, mOffScale, straightSpeed, homeCrossTime);
    followWallPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause, ignoreHorizontalAfterWall);

    %% RIGHT INTO COLOR PATH
    fprintf('Back at HOME -> turning RIGHT into COLOR path\n');
    turnRight90(nb, minVals, maxVals, turnSpeed, ignoreTurn90Time);
    followColorPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause);

    %% FINAL STOP AT HOME
    fprintf('MISSION 0 COMPLETE AT HOME\n');
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end

%% ========================================================================
%% ABSTRACTED SUB-PATHS
%% ========================================================================
function followLeftPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause, turnSpeed, ignoreTurn180Time)
    fprintf('LEFT PATH START\n');

    % Follow path until line
    fprintf('Following left branch until turnaround line...\n');
    lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, 0, linePause);

    % 180
    fprintf('Doing 180 on left branch...\n');
    performTurnToLine(nb, turnSpeed, ignoreTurn180Time, minVals, maxVals, 0, 0);

    % Return to HOME
    fprintf('Returning from left branch to HOME...\n');
    lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, 0, linePause);

    fprintf('LEFT PATH END (HOME reached)\n');
end

function followWallPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause, ignoreHorizontalAfterWall)
    fprintf('WALL PATH START\n');

    % This version assumes the robot has already crossed HOME and is entering
    % the wall branch. So it line follows until the wall trigger line, then
    % uses the front ultrasonic, loops around, refinds the path, and returns HOME.

    frontStopDist = 300;
    wallMinDist = 50;
    wallMaxDist = 600;
    wallBaseSpeed = 9;
    turnSpeed = 11;
    motorBaseSpeed = 10;
    wallReturnNudgeTime = 0.18;

    kp = 0.9;
    ki = 0.0;
    kd = 0.15;

    prevError = 0;
    prevTime = 0;
    integral = 0;

    % Initialize required sensors
    nb.initReflectance();
    nb.initUltrasonic1('D2','D3');   % front
    nb.initUltrasonic2('D4','D5');   % side

    %% 1) Follow path until wall-trigger line
    fprintf('Following line to wall-trigger line...\n');
    lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, 0, linePause);

    %% 2) Keep line following until front sensor says wall is close
    fprintf('Front ultrasonic enabled. Advancing to wall...\n');

    nb.setMotor(1, mOffScale * 10);
    nb.setMotor(2, -10);
    pause(0.2);

    prevError = 0;
    prevTime = 0;
    integral = 0;
    tic

    while true
        front = nb.ultrasonicRead1();

        if front <= frontStopDist
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end

        dt = toc - prevTime;
        prevTime = toc;

        [vals, calibratedVals] = getCalibratedReflectance(nb, minVals, maxVals);

        error = computeLineError(calibratedVals);
        error = clampValue(error, -6, 6);

        integral = integral + error * dt;

        if dt > 0
            derivative = (error - prevError) / dt;
        else
            derivative = 0;
        end

        control = kp * error + ki * integral + kd * derivative;

        m1Duty = mOffScale * max(0, motorBaseSpeed + min(control,0));
        m2Duty = -max(0, motorBaseSpeed - max(control,0));

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);

        prevError = error;
    end

    %% 3) Turn into wall-follow arc
    fprintf('Turning into wall-follow arc...\n');
    while true
        left = nb.ultrasonicRead2();
        nb.setMotor(1, -mOffScale * turnSpeed);
        nb.setMotor(2, -turnSpeed);

        if left <= 500
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
            break;
        end
    end

    %% 4) Follow wall until path is refound
    fprintf('Following wall until path is refound...\n');
    while true
        left = nb.ultrasonicRead2();
        fprintf('Side distance: %0.3f\n', left);

        [vals, calibratedVals] = getCalibratedReflectance(nb, minVals, maxVals);

        if calibratedVals(3) >= 0.7 || calibratedVals(4) >= 0.7
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
        
            % Small right-turn correction after refinding the path
            performTurnToLine(nb, 9.5, 0, minVals, maxVals, 0, 0);
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(0.05);
        
            break;
        end

        if left >= wallMinDist && left <= wallMaxDist
            rightCmd = wallBaseSpeed;
            leftCmd = wallBaseSpeed;
        else
            rightCmd = 9.6;
            leftCmd = -9.6;
            pause(0.05);
        end

        nb.setMotor(1, mOffScale * rightCmd);
        nb.setMotor(2, -leftCmd);
    end

    %% 5) Detect horizontal line, nudge through it, then return HOME
    fprintf('Following line to horizontal return line...\n');
    lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, 0, linePause);

    fprintf('Nudging through horizontal return line...\n');
    driveStraightForTime(nb, mOffScale, 9, wallReturnNudgeTime);

    fprintf('Following line from return line to HOME...\n');
    lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, 0, linePause);

    fprintf('WALL PATH END (HOME reached)\n');
end

function followColorPathAndReturnHome(nb, mOffScale, minVals, maxVals, linePause)
    fprintf('COLOR PATH START\n');

    % Reuse your color behavior, then treat the ending horizontal line as HOME.
    performColorFind(nb, mOffScale, minVals, maxVals);

    % Extra stop for stability
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
    pause(linePause);

    fprintf('COLOR PATH END (HOME reached)\n');
end

%% ========================================================================
%% HELPER MOTION FUNCTIONS
%% ========================================================================
function turnLeft90(nb, minVals, maxVals, turnSpeed, ignoreLineTime)
    performTurnToLine(nb, turnSpeed, ignoreLineTime, minVals, maxVals, 1, 1);
end

function turnRight90(nb, minVals, maxVals, turnSpeed, ignoreLineTime)
    performTurnToLine(nb, turnSpeed, ignoreLineTime, minVals, maxVals, 0, 1);
end

function driveStraightUntilBlackLine(nb, mOffScale, minVals, maxVals, speedCmd, linePause)
    fprintf('Driving straight until black line...\n');

    while true
        nb.setMotor(1, mOffScale * speedCmd);
        nb.setMotor(2, -speedCmd);

        [vals, calibratedVals] = getCalibratedReflectance(nb, minVals, maxVals);

        if all(calibratedVals >= 0.8)
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(linePause);
            break;
        end
    end
end

function driveStraightForTime(nb, mOffScale, speedCmd, moveTime)
    tic
    while toc < moveTime
        nb.setMotor(1, mOffScale * speedCmd);
        nb.setMotor(2, -speedCmd);
    end
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
    pause(0.05);
end

function lineFollowUntilBlack(nb, mOffScale, minVals, maxVals, stopDelay, linePause)
    kp = 2.0;
    ki = 0.0;
    kd = 0.1;

    prevError = 0;
    prevTime = 0;
    integral = 0;
    motorBaseSpeed = 10;

    tic
    while true
        dt = toc - prevTime;
        prevTime = toc;

        [vals, calibratedVals] = getCalibratedReflectance(nb, minVals, maxVals);

        if toc > stopDelay && all(calibratedVals >= 0.8)
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            pause(linePause);
            break;
        end

        error = computeLineError(calibratedVals);
        error = clampValue(error, -6, 6);

        integral = integral + error * dt;

        if dt > 0
            derivative = (error - prevError) / dt;
        else
            derivative = 0;
        end

        control = kp * error + ki * integral + kd * derivative;

        m1Duty = mOffScale * max(0, motorBaseSpeed + min(control,0));
        m2Duty = -max(0, motorBaseSpeed - max(control,0));

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);

        prevError = error;
    end
end

function error = computeLineError(calibratedVals)
    extScalar = 10;
    midScalar = 4;
    innScalar = 2;
    rightWeight = 1.35;

    % error = 3*extScalar*calibratedVals(1) + ...
    %         1.3*midScalar*calibratedVals(2) + ...
    %         innScalar*(1-calibratedVals(3)) - ...
    %         rightWeight*innScalar*(1-calibratedVals(4)) - ...
    %         rightWeight*midScalar*calibratedVals(5) - ...
    %         3*rightWeight*extScalar*calibratedVals(6);

    error = 6*calibratedVals(1) + ...
            4*calibratedVals(2) + ...
            0.5*(1-calibratedVals(3)) - ...
            0.5*(1-calibratedVals(4)) - ...
            4*calibratedVals(5) - ...
            6*calibratedVals(6);
end

function [vals, calibratedVals] = getCalibratedReflectance(nb, minVals, maxVals)
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
end

function out = clampValue(val, lowVal, highVal)
    out = val;
    if out < lowVal
        out = lowVal;
    elseif out > highVal
        out = highVal;
    end
end

%% ========================================================================
%% TURN-UNTIL-LINE FUNCTION
%% ========================================================================
function performTurnToLine(nb, turnSpeed, ignoreLineTime, minVals, maxVals, CCW, turnMode)
    fprintf('TURNING TO LINE\n');
    mOffScale = 1;

    % turnMode:
    % 0 = original two-wheel spin turn
    % 1 = home pivot turn (only outside wheel spins)

    % First ignore line detection for a short time
    tic
    while toc < ignoreLineTime
        if turnMode == 1
            if CCW == 1
                % Left turn from HOME: only right/outside wheel spins
                nb.setMotor(1, mOffScale * turnSpeed);
                nb.setMotor(2, 0);
            else
                % Right turn from HOME: only left/outside wheel spins
                nb.setMotor(1, 0);
                nb.setMotor(2, -turnSpeed);
            end
        else
            if CCW == 1
                % Original spin turn behavior
                nb.setMotor(1, mOffScale * turnSpeed);
                nb.setMotor(2, turnSpeed);
            else
                nb.setMotor(1, -(mOffScale * turnSpeed));
                nb.setMotor(2, -turnSpeed);
            end
        end
    end

    % Then keep turning until center sensors see the line
    while true
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

        if turnMode == 1
            if CCW == 1
                nb.setMotor(1, mOffScale * turnSpeed);
                nb.setMotor(2, 0);
            else
                nb.setMotor(1, 0);
                nb.setMotor(2, -turnSpeed);
            end
        else
            if CCW == 1
                nb.setMotor(1, mOffScale * turnSpeed);
                nb.setMotor(2, turnSpeed);
            else
                nb.setMotor(1, -(mOffScale * turnSpeed));
                nb.setMotor(2, -turnSpeed);
            end
        end

        if calibratedVals(3) >= 0.3 || calibratedVals(4) >= 0.3
            fprintf('Line found, turn stopped.\n');
            break;
        end
    end

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
    pause(0.05);
end

%% ========================================================================
%% ORIGINAL COLOR FUNCTION
%% ========================================================================
function performColorFind(nb, mOffScale, minVals, maxVals)
    nb.initReflectance();

    kp = 0.9;
    ki = 0.2;
    kd = 0.15;

    prevError = 0;
    prevTime = 0;
    integral = 0;

    motorBaseSpeed = 10;
    whiteThresh = 250;

    %% Phase 1: follow line until marker
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

        error = computeLineError(calibratedVals);
        error = clampValue(error, -6, 6);

        integral = integral + error * dt;

        if dt > 0
            derivative = (error - prevError) / dt;
        else
            derivative = 0;
        end

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
    nb.initColor();
    values = nb.colorRead();

    red = values.red;
    green = values.green;
    blue = values.blue;
    fprintf('red: %.2f, green: %.2f, blue: %.2f\n', red, green, blue);

    if red > 170
        CCW = 1;
        fprintf('RED: CCW\n');
    elseif green > 100
        CCW = 0;
        fprintf('BLUE: CW\n');
    else
        CCW = 0;
        fprintf('No strong color detected, default CW\n');
    end

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);

    % Color turn keeps original spin-turn behavior
    turnSpeed = 10;
    ignoreTurnTime = 1.00;
    performTurnToLine(nb, turnSpeed, ignoreTurnTime, minVals, maxVals, CCW, 0);

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

        error = computeLineError(calibratedVals);
        error = clampValue(error, -6, 6);

        integral = integral + error * dt;

        if dt > 0
            derivative = (error - prevError) / dt;
        else
            derivative = 0;
        end

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