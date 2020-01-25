%function pos = ex_vehicle_control()
% Initialize Client
%addpath('c:\Users\HSL\Desktop\NatNetSDK\Samples\Matlab');
%%
%Quick quide
% For testing Communication b/n arduino and RC car
% open arduino IDE -> tools -> serial monitor -> paste the following
% "65535, 1700, 1000, 1500, 1500, 1500, 1500"
% - Remember you might need to send each commands two times.

%%
%delete any related port that is com6 from matlab workstation
delete(instrfind({'Port'},{'COM6'}))

% Setup for motive....no need to change this
dllPath = fullfile('c:','Users','yzeleke','Desktop','HSL_exp','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0);
HostIP = char('127.0.0.1');
theClient.Initialize(HostIP, HostIP);
%%

% Initialize Arduino port
ppmValues = [1500,1500,1500,1500,1500,1500];
[s,flag] = initSerial_copy();     % initSerial('COM3', 6);
transmitSerial(s, ppmValues);

% set bounds  
upper_bound = 1.2;
lower_bound = -.5;
distanceToDest = .3;
distanceToObs = 1;
thrust = 1640;
0;
state = 0;


%%


% for(i = 1:500)
while (1)
    % Get current frame info
    
    frameOfData = theClient.GetLastFrameOfData();
    vehicle = frameOfData.RigidBodies(1);
    
    r=2; %r = 4
    l=3; % l = 2
    f=4; %f = 1
    b=1; %b = 3
    
    right_node = [vehicle.Markers(r).x vehicle.Markers(r).z -vehicle.Markers(r).y];
    left_node = [vehicle.Markers(l).x vehicle.Markers(l).z -vehicle.Markers(l).y];
    front_node = [vehicle.Markers(f).x vehicle.Markers(f).z -vehicle.Markers(f).y];
    back_node = [vehicle.Markers(b).x vehicle.Markers(b).z -vehicle.Markers(b).y];
    
    % sensor output = x,y, and heading 
    x = (front_node(1,1) + back_node(1,1))/2;
    y = (front_node(1,2) + back_node(1,2))/2;
    
 
    
    % controller output = v,w
    x_d = front_node(1) - back_node(1);
    y_d = front_node(2) - back_node(2);
    theta_car = atan2(y_d,x_d);
    
    % Retrieve the (relative) goal location 
    dest_node = frameOfData.RigidBodies(2);
    x_g = dest_node.x;
    y_g = dest_node.z;
   
    % Retrieve the (relative) obstacle location
    dest_node = frameOfData.RigidBodies(3);
    x_o = dest_node.x;
    y_o = dest_node.z;
    
    %distance1 is the distance from the robot to the goal
    distance1 = sqrt((x - x_g)^2 + (y - y_g)^2);
    if(distance1 <= distanceToDest) 
        disp('Target Found!!!!!!!!!!!');
           ppmValues = [0,0,1500,1500,1500,1500];
           transmitSerial(s, ppmValues);  
        continue;
        %break;
    end
    
    %distance2 is the distance from the robot to the obstacle
    distance2 = sqrt((x-x_o)^2 + (y-y_o)^2);
    
      
    %vector point from robot to the goal
    u_y_g = y_g-y;
    u_x_g = x_g-x;  
    
    % angle from robot to goal.positive:right, negative:left
    theta_goal = atan2(u_y_g,u_x_g);
    e_k_g = theta_goal-theta_car;
    e_k_g = atan2(sin(e_k_g),cos(e_k_g));
    
    %vector point from robot to the obstacle
    u_y_o = y_o-y;
    u_x_o = x_o-x;
    
    %angle from robot to obstacle
    theta_obstacle = atan2(u_y_o,u_x_o);
    e_k_o = theta_obstacle-theta_car;
    e_k_o = -atan2(sin(e_k_o),cos(e_k_o));
    
    %information about the state variable:
    %      state == 0: we are close enough to the goal, stop the whole loop
    %      state == 1: we are close to the obstacle, we follow the wall
    %      state == 2: we go to the goal without worrying about the
    
    %angle between the goal vector and obstacle vector
    theta_obstacle = theta_obstacle;
    theta_goal = theta_goal;
    e_k_d = theta_obstacle-theta_goal;
    e_k_d = atan2(sin(e_k_d),cos(e_k_d));
    
    %if we are very close to the goal we stop
    if distance1 < distanceToDest
        state = 0;
    %if we are close to the obstacle and the angle between (u_y_o,u_x_o),
    %(u_y_g,u_x_g) is smaller than 90 degree
    elseif distance2 < distanceToObs && abs(e_k_d)<(pi/2)
        state = 1;
    %we go to the goal
    else
        state = 2;
    end
        
            
    
    %determin the desired rudder
    if state == 0
        display('shouldnt be here');
        break;
    elseif state == 1
        %turn right
        if e_k_d>0
            right = 1;
            theta_obstacle = theta_obstacle-pi/2 ;  
        %turn left
        else
            left = 1;
            theta_obstacle = theta_obstacle+pi/2 ;    
        end
        e_k_o = theta_obstacle-theta_car;
        e_k_o = -atan2(sin(e_k_o),cos(e_k_o));
        rudder = double(round(1500-1000*e_k_o/pi));
        
    else
        e_k_g = e_k_g;
        rudder = double(round(1500+1000*e_k_g/pi));
    end

    
    
    if(rudder > 2000)
        rudder = 2000;
    elseif(rudder < 1000)
        rudder = 1000;
    end
    if(rudder < 1600 && rudder > 1500)
        rudder = 1600;
    elseif(rudder < 1500 && rudder > 1400)
        rudder = 1400;
    end
    
    ppmValues = [thrust,rudder,1500,1500,1500,1500];
    transmitSerial(s, ppmValues);                       % send current signal
    
end

ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);                       % send current signal

%% Just incase if it keeps going Ctrl+C and paste the following on terminal
%ppmValues = [0,1500,1500,1500,1500,1500];
%transmitSerial(s, ppmValues);   


% close port
fclose(s);

% disconnect client
theClient.Uninitialize;

pos = 1;

%end