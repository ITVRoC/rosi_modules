% This code reads a csv file create by node traction_vel_calibration
% and finds the linear equation that realtes traction joint velocities
% to base linear velocities
clear all;
clc;
close all;
addpath('../output/rosi_vicon');

%% Parameters

% file to read
fileName = '2023-06-19_19-50-20.csv';

% polynomial degree
polyDeg = 1;

%% Executing

% reading csv
M = csvread(fileName);

% performing the polyfit
coefs = polyfit(M(:,1), M(:,2), polyDeg)

%% Plot
figure;
scatter(M(:,1), M(:,2));
hold on;
plot([M(1,1), M(end,1)],[coefs(1)*M(1,1)+coefs(2), coefs(1)*M(end,1)+coefs(2)], '-r');
title('Polyfit of found data.')
xlabel('Joint Velocity [rad/s]');
ylabel('Robot base linear velocity [m/s]');

















