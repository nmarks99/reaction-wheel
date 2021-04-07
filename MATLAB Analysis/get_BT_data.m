b% This script reads, saves, and plots data from Arduino bluetooth module

delete(instrfindall);   % Close all serial ports
BT = serialport('COM5',9600);    % Define bluetooth serial port

% Make a button to stop data aquisition when desired
figure;
set(gcf,'position',[500,500,100,50])
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop', ...
                         'Callback', 'delete(gcbf)');

% Aquire data until button press
data = [];
i = 1;
while 1 == 1 
    if ~ishandle(ButtonHandle)
        disp('Data aquisition stopped');
        break
    end
    rawline = readline(BT);
    elem = strsplit(rawline,',');
    fprintf('theta: %f\ttime: %.2f\n',elem(1),elem(2));
    data(i,1) = elem(1);
    data(i,2) = elem(2);
    i = i + 1;
end

% Normalize time to first data point and convert to seconds
for i = 1:length(data(:,2)) 
    data(i,2) = (data(i,2) - data(1,2))/1000;
end
theta = data(:,1);
time = data(:,2);
target = 1; % Target is 1 degree

% Save the data as a text file to the DATA folder
name = input('Enter filename to save, or 0 to not save: ');
if string(name) ~= '0'
    folderpath = "C:\Users\Owner\OneDrive\Documents\ME399 Reaction Wheel\MATLAB Code\DATA";
    filename  = fullfile(folderpath,name);
    writematrix(data,filename);
end

% Plot the data
figure;
plot(time,theta,'b');
hold on 
grid on
yline(target,'--r');
legend('Measured position','Target position');
title('Angular position vs. time');
xlabel('Time (s)');
ylabel('Position (degrees)');

clearvars -except data