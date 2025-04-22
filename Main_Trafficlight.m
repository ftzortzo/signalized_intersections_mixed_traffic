clc;
clear all;
close all;
sum_previous_cycles=0;

global vmax vmin umin gamma umax v_des length_of_control_zone Replanning_flag


%% Create the Pressure structure
Pressure = struct();

% Τhis is a flag we use letter for traffic
traffic_light_change_flag=0;

% Define branch names
branches = {'Incoming1', 'Incoming2', 'Incoming3', 'Outgoing', 'Final_lane','Final'};

% Assign cell arrays with 8 elements to each of the first 4 branches
for i = 1:5
    Pressure.(branches{i}) = cell(1, 12);  % Create an empty cell array of size 1x8
end

% Assign an empty array to the Final branch
Pressure.Final = [];

%% Simulation parameters

Replanning_flag=0;
v_des = 15;
Simulation_Time=150;
range_of_coordinator=300;
length_of_control_zone=600;
dt=0.05;
number_of_CAVs =20;
reaction_time = 1.2; % Example value for reaction time
gamma = 5; % Example value for gamma


%% Vehicles parameters
umax = 5;
vmax = 20;
vmin= 0.1;
umin=-5;
L = 2;   % Wheelbase length (m)
rect_length = 2.5;  % Vehicle length (m)
rect_width = 1.2; % Vehicle width (m)

% Road parameters
road_width = 10;
road_length = 50;

%% Upload data
load("Traffic_Light_info.mat")

% Initialize_Cars
load("_Last_Initial_Conditions.mat")

% Maximize figure to full screen
fig = figure;
set(fig, 'Position', get(0, 'Screensize')); % Set figure to full screen
set(fig, 'WindowState', 'maximized'); % Maximizes the figure in newer MATLAB versions


%% the following variables are used to calculate the pressure in the intersection
Incoming1 = zeros(Simulation_Time/dt, 11); 
Incoming2 = zeros(Simulation_Time/dt, 11);
Incoming3 = zeros(Simulation_Time/dt, 11);
Outgoing = zeros(Simulation_Time/dt, 11);

Incoming1_local = [];
Incoming2_local = [];
Incoming3_local = [];
Outgoing_local = [];
reprediction=0;


%% Simulation loop
plot_counter = 1;
modeDurations = [30/4,30/4,30/4,30/4]; % here we get the initial mode durations
for t = 0:dt:Simulation_Time
    
    plot_counter = plot_counter + 1;   
    cla; % Clear axis for dynamic plot update
    % Calculate_Pressure
    Plot_traffic_lights_and_Intersection 

    UpdateCAVs
   
    traffic_light_change_flag=0; 
            pause(0.0001)

    clc

end



