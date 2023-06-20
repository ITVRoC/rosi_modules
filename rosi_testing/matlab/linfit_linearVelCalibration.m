% This code reads a csv file create by node traction_vel_calibration
% and finds the linear equation that realtes traction joint velocities
% to base linear velocities
clear all;
clc;
close all;
addpath('../output/rosi_vicon');

%% Parameters

% file to read
fileName = '2023-06-20_14-47-12.csv';

% polynomial degree
polyDeg = 1;

%% Executing

% reading csv data
M = csvread(fileName);
x_data = M(:,1);
y_data = M(:,2);

% performing the polyfit
coefs = polyfit(x_data, y_data, polyDeg)

% calculate the predicted values using the fitted line
y_pred = polyval(coefs, x_data);

% Calculate the total sum of squares (TSS)
mean_y = mean(y_data);
tss = sum((y_data - mean_y).^2);

% Calculate the residual sum of squares (RSS)
rss = sum((y_data - y_pred).^2);

% Calculate the coefficient of determination (R-squared)
r_squared = 1 - (rss / tss)

%% Plot
figure;
scatter(M(:,1), M(:,2));
hold on;
plot([M(1,1), M(end,1)],[coefs(1)*M(1,1)+coefs(2), coefs(1)*M(end,1)+coefs(2)], '-r');
title('Polyfit of found data.')
xlabel('Joint Velocity [rad/s]');
ylabel('Robot base linear velocity [m/s]');

















