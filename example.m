%function pos = example()

%%
% Initialize Client
close all;
clear;clc;
delete(instrfind({'Port'},{'COM6'}))

addpath('C:\Users\yzeleke\Desktop\HSL_exp');
addpath('C:\Users\yzeleke\Desktop\HSL_exp\NatNetSDK\Samples\Matlab');
%dllPath = fullfile('c:','Users','HSL','Desktop','NatNetSDK','lib','x64','NatNetML.dll');  % remove 'x64' extension if using 32 bit matlab
dllPath = fullfile('c:','Users','yzeleke','Desktop','HSL_exp','NatNetSDK','lib','x64','NatNetML.dll');

%Stream data from motive ... motive is an optitrack software for tracking
%the vehicle and other objects

assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0);
HostIP = char('127.0.0.1');
theClient.Initialize(HostIP, HostIP);

% Initialize Arduino port
ppmValues = [1500,1500,1500,1500,1500,1500]
[s,flag] = initSerial_copy()     % initSerial('COM3', 6);
transmitSerial(s, ppmValues);

% close port
%fclose(s);

% disconnect client
%theClient.Uninitialize;
%%
% set bounds  
upper_bound = 1.2;
lower_bound = -.5;
distance = .3;
thrust = 1600;

%%




for(i = 1:750)
    % Get current frame info
    frameOfData = theClient.GetLastFrameOfData();
    vehicle = frameOfData.RigidBodies(1);
    left_node = [vehicle.Markers(4).x vehicle.Markers(4).z -vehicle.Markers(4).y];
    right_node = [vehicle.Markers(2).x vehicle.Markers(2).z -vehicle.Markers(2).y];
    front_node = [vehicle.Markers(1).x vehicle.Markers(1).z -vehicle.Markers(1).y];
    back_node = [vehicle.Markers(3).x vehicle.Markers(3).z -vehicle.Markers(3).y];
    
    % sensor output = x,y, and heading 
    x = (front_node(1,1) + back_node(1,1))/2;
    y = (front_node(1,2) + back_node(1,2))/2;
   
    %x= vehicle.x;
    %y= vehicle.y;
    %z = vehicle.z;
    %theta_car = atan2(y,x);
    
    % controller output = v,w
    x_d = front_node(1) - back_node(1);
    y_d = front_node(2) - back_node(2);
    theta_car = atan2(y_d,x_d);
    % 
    dest_node = frameOfData.RigidBodies(2);
    % Retrieve the (relative) goal location
    x_g = dest_node.x;
    y_g = dest_node.z;
    
    
    distance1 = sqrt((x - x_g)^2 + (y - y_g)^2)
    if(distance1 <= distance) 
        break;
    end
    
    u_y = y_g-y;
    u_x = x_g-x;       
    
    % angle from robot to goal.
    theta_goal = atan2(u_y,u_x);
    e_k = theta_goal-theta_car;
    e_k = -atan2(sin(e_k),cos(e_k));
    rudder = double(round(1500-1000*e_k/pi));
    %rudder = 2000;
    if(rudder > 2000)
        rudder = 2000;
    elseif(rudder < 1000)
        rudder = 1000;
    end
    
    ppmValues = [thrust,rudder,1500,1500,1500,1500];
    transmitSerial(s, ppmValues);                       % send current signal
    
end



ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);

% close port
fclose(s);

% disconnect client
theClient.Uninitialize;

pos = 1;
%end