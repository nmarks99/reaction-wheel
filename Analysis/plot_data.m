function data = plot_data
% PLOT_DATA loads the selected TXT file into the workspace and plots it

% Open file explorer to select data you want to plot
[name,D] = uigetfile('*.txt');
filepath = fullfile(D,name);

% Read in the data array
data = readmatrix(filepath);
theta = data(:,1);
time = data(:,2);
target = 1;

% Plot the data
figure;
plot(time,theta,'b')
hold on
grid on
yline(target,'--r');
legend('Measured position','Target position');
title('Angular position vs. Time');
xlabel('Time (s)');
ylabel('Angular position (degrees)');

end