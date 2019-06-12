%This code is for applying kalman filter.

clear all; %free up system memory
close all;
clc;

% creat the frames address as a directory
directory = 'D:\Matlab-2013a\bin\ALL_TRACKING\realcar2 (29-Jul-17 4-43-48 AM)';

%by using this command enter the directory
cd(directory);

% for get the list of frames
frame_list =  dir('*jpg');

% for load tracking data
load('Track_data_store.mat');

%% define variables for kalman filter

dt = 1; % sampling rate

Starting_frame = 1; % starting frame

u = .005;  % define acceleration magnitude

Q = [Track_data(Starting_frame,1); Track_data(Starting_frame,2); 0; 0]; %initized-[positionX; positionY; velocityX; velocityY] of the object

Q_estimate = Q;  % estimate of initial location estimation of where the object is.

Accel_noise_mag = .1; % process noise

noise_x = 1;  % measurement noise(x axis).
noise_y = 1;  % measurement noise(y axis).

Ez = [noise_x 0; 0 noise_y];

Ex = [dt^4/4 0 dt^3/2 0; ...
      0 dt^4/4 0 dt^3/2; ...
      dt^3/2 0 dt^2 0; ...
      0 dt^3/2 0 dt^2].*Accel_noise_mag^2; % convert the process noise into covariance matrix
  
P = Ex; % estimate of initial object position variance

%% Define update equations in 2-D

A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % state update matrice
B = [(dt^2/2); (dt^2/2); dt; dt];
C = [1 0 0 0; 0 1 0 0];  % this is measurement function C, that apply to the state estimate Q to get expect next measurement


%%Initialize result variables

Q_loc = []; % actual object motion path
vel = []; % actual object velocity
Q_location_measurement = []; % the objectpath extracted by the tracking algorithm


%% Initize estimation variables

Q_location_estimate = []; % position estimate
velocity_estimate = []; % velocity estimate
P_estimate = P;
predic_state = [];
predic_var = [];

r = 15; % define radius for the plotting circle
j=0:.01:2*pi; % for make the plotting circle
%m = zeros(length(frame_list),2);


for t = Starting_frame:length(frame_list)
    
    % loading image
    img_tmp = double(imread(frame_list(t).name));
    img = img_tmp(:,:,1);
    subplot(121);imagesc(img);
    axis off
    title('Raw Video');
    
    % Load the given tracking
    Q_location_measurement(:,t) = [ Track_data(t,1); Track_data(t,2)];  %measurement value from image processing 
    
    %Applying of kalman filter   
    
    % Predict next state of the object with the last state and predicted motion.
    Q_estimate = A * Q_estimate + B * u;
    predic_state = [predic_state; Q_estimate(1)] ; %  
    
    %Predict next covariance
    P = A * P * A' + Ex;
    predic_var = [predic_var; P] ;
    
    % Kalman gain
    K = P*C'*inv(C*P*C'+Ez);  %P=P(predict),Ez=measurement noise covarience
    
    % Update the state estimate.
    if ~isnan(Q_location_measurement(:,t))
        
        Q_estimate = Q_estimate + K * (Q_location_measurement(:,t) - C * Q_estimate); %Updated real position
        
    end
    
    %m(t,:)=[Q_estimate(t,1)  Q_estimate(t,2)];
    % Update error covariance estimation.
    P =  (eye(4)-K*C)*P;
    
    % Store data
    Q_location_estimate = [Q_location_estimate; Q_estimate(1:2)];
    velocity_estimate = [velocity_estimate; Q_estimate(3:4)];
   
    
    % Plot the images with the tracking
    subplot(122);
    imagesc(img);
    axis off
    colormap(gray);
    
    hold on;
     %plot(r*sin(j)+Q_location_measurement(2,t),r*cos(j)+Q_location_measurement(1,t),'.g'); 
    plot(r*sin(j)+Q_estimate(2),r*cos(j)+Q_estimate(1),'.r'); % the kalman filtered tracking
    %plot(Q_estimate(2),Q_estimate(1),'.g');
    hold off;
    
    title('Object Tracked');
  
    pause(0.001)
end

for t = 1:length(frame_list)
   
   hold on;
   plot(Track_data(:,2),Track_data(:,1),'.b');
   hold off
   
end

figure;
plot(Track_data(:,2),Track_data(:,1),'.b');
xlabel('X-Axis');
ylabel('Y-Axis');
title(' Object traveling path')


