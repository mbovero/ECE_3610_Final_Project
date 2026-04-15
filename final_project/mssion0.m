%%%%%%%%%%%%%
% ECE 3610
% Final Project
%%%%%%%%%%%%%

clc
clear all
%% Train Neural Network For Gesture Classification


%% CONNECT TO THE NANOBOT
% for PC:
nb = nanobot('COM12', 115200, 'serial');
% for Mac:
% nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%% MAIN SETUP --------------------------------------------------------------
minVals = [93.8,84.2,74.0,71.6,83.00,108.4];
maxVals = [556.1,476.7,748.9,668.9,273.8,318.1];

mOffScale = 1.11;
motorBaseSpeed = 9;

mode = 'mission0_abstracted';   % only mode used in this file

%%
if strcmp(mode, 'mission0_abstracted')
    performMission0Abstracted(nb, mOffScale, minVals, maxVals);
end

%% OPTIONAL MANUAL STOP / DISCONNECT
nb.setMotor(1, 0);
nb.setMotor(2, 0);%
delete(nb);%

clear('nb');

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
    turnSpeed = 9;
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