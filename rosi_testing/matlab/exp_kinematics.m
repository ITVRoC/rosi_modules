% This code reads a csv file containing kinematics results
% and plots the results
clear all;
clc;
close all;

% path to the data file
addpath('../output/kinematics');

% path to the DualQuaternion library
addpath('/home/filipe/pCloud_sync/DOC/DOC/pratico/front_dq/implementing/lib/dq');


%% Parameters

% file to read
p_fileName = '2023-06-21_16-34-39.csv';

% gdl to plot
p_gdlPlot = 'vl_x';

%% Executing

% reading csv data
M = csvread(p_fileName, 1, 0);

% splitting data
time = M(:,1);
























