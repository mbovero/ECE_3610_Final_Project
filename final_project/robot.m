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
nb = nanobot('COM4', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%%


%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all