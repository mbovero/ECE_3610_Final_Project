clear; clc; close all; %initialization

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





%%
% Run-Time Predictions 
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












%% 14. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all
close all