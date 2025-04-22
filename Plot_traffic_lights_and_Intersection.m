    
%% plot traffic lights 
    % Position of the traffic light near the road (adjust as necessary)
traffic_light_position{1} = [15, -40];  % [x, y] coordinates of the traffic light
traffic_light_position{2} = [25, -40];  % [x, y] coordinates of the traffic light
% traffic_light_position{3} = [40, -35];  % [x, y] coordinates of the traffic light

traffic_light_position{4} = [35, 15];  % [x, y] coordinates of the traffic light
traffic_light_position{5} = [35, 25];  % [x, y] coordinates of the traffic light
% traffic_light_position{6} = [35, 40];  % [x, y] coordinates of the traffic light

traffic_light_position{7} = [-20, 35];  % [x, y] coordinates of the traffic light
traffic_light_position{8} = [-30, 35];  % [x, y] coordinates of the traffic light
% traffic_light_position{9} = [-40, 35];  % [x, y] coordinates of the traffic light

traffic_light_position{10} = [-40, -20];  % [x, y] coordinates of the traffic light
traffic_light_position{11} = [-40, -30];  % [x, y] coordinates of the traffic light
% traffic_light_position{12} = [-35, -40];  % [x, y] coordinates of the traffic light


% Traffic light logic
traffic_light_green = 'g';  % Red light
traffic_light_red = 'r';  % Green light
traffic_light_yellow = 'y';

% Define the durations for each mode (adjust these values as needed)
cumulativeDurations = cumsum(modeDurations(1:4)); % Cumulative time points for each mode

% Determine the total cycle time
cycle_time =sum(modeDurations(1:4));

% Get the current time in the cycle
current_time_in_cycle = mod(t-sum_previous_cycles, cycle_time);

% Find the current mode based on the current time in the cycle
modeIndex = find(current_time_in_cycle <= cumulativeDurations, 1);

% modeDurations = [10,10,10,10]; % Sum should match cycle_time


currentMode = maxLengthSets{modeIndex};



time_at_next_mode = floor(t/cycle_time)*cycle_time + sum(modeDurations(1:modeIndex));

time_until_next_node = time_at_next_mode -t;

%check if I have two consequtive traffic lights
repeated_cycle=-1;
if length(maxLengthSets)>4
    if sum(currentMode==maxLengthSets{modeIndex+1})==2
        repeated_cycle=1;
    end
end



% Get the traffic lights that should be green for the current mode


% Loop through all traffic lights and set their colors based on the current mode
for i = 1:12
    if ismember(i, currentMode)
        % Set the traffic light to green
        if time_until_next_node > 3 || repeated_cycle==1
        rectangle('Position', [traffic_light_position{i}(1), traffic_light_position{i}(2), 5, 5], ...
                  'Curvature', [1, 1], 'FaceColor', traffic_light_green, 'EdgeColor', 'k');
        else
                    rectangle('Position', [traffic_light_position{i}(1), traffic_light_position{i}(2), 5, 5], ...
                  'Curvature', [1, 1], 'FaceColor', traffic_light_yellow, 'EdgeColor', 'k');
        end
    elseif ~(mod(i,3)==0)
        % Set the traffic light to red
        rectangle('Position', [traffic_light_position{i}(1), traffic_light_position{i}(2), 5, 5], ...
                  'Curvature', [1, 1], 'FaceColor', traffic_light_red, 'EdgeColor', 'k');
    end
end














%% plot intersection

    hold on
    
    radius = 20; % Radius of the corner curve
    theta = linspace(0, pi/2, 100); % Angle for quarter-circle arcs
    % Plot the horizontal and vertical road boundaries
    plot([-600, -radius-10.5], [-10.5, -10.5], 'k', 'LineWidth', 2); % Left horizontal boundary
    plot([-600, -radius-10.5], [10.5, 10.5], 'k', 'LineWidth', 2);  % Right horizontal boundary
    plot([radius+10.5, 600], [-10.5, -10.5], 'k', 'LineWidth', 2);  % Left horizontal boundary
    plot([radius+10.5, 600], [10.5, 10.5], 'k', 'LineWidth', 2);   % Right horizontal boundary
    plot([-10.5, -10.5], [-600, -radius-10.5], 'k', 'LineWidth', 2);  % Vertical boundaries
    plot([-10.5, -10.5], [radius+10.5, 600], 'k', 'LineWidth', 2);  % Vertical boundaries
    plot([10.5, 10.5], [-600, -radius-10.5], 'k', 'LineWidth', 2);  % Vertical boundaries
    plot([10.5, 10.5], [radius+10.5, 600], 'k', 'LineWidth', 2);  % Vertical boundaries
    %Plot the dotted lines
    % plot([-600, -radius-10.5], [7, 7], 'k--', 'LineWidth', 0.2); % Left horizontal boundary
    plot([-600, -radius-10.5], [5.25, 5.25], 'k--', 'LineWidth', 0.2); % Left horizontal boundary
    % plot([-600, -radius-10.5], [-3.5, -3.5], 'k--', 'LineWidth', 0.2); % Left horizontal boundary
    plot([-600, -radius-10.5], [-5.25, -5.25], 'k--', 'LineWidth', 0.2); % Left horizontal boundary


    % plot([radius+10.5, 600], [7, 7], 'k--', 'LineWidth', 0.2);  % Left horizontal boundary
    plot([radius+10.5, 600], [5.25, 5.25], 'k--', 'LineWidth', 0.2);   % Right horizontal boundary
    % plot([radius+10.5, 600], [-3.5, -3.5], 'k--', 'LineWidth', 0.2);  % Left horizontal boundary
    plot([radius+10.5, 600], [-5.25, -5.25], 'k--', 'LineWidth', 0.2);   % Right horizontal boundary

    plot([-5.25, -5.25], [-600, -radius-10.5], 'k--', 'LineWidth', 0.2);  % Vertical boundaries
    % plot([-3.5, -3.5], [-600, -radius-10.5], 'k--', 'LineWidth', 0.2);  % Vertical boundaries

    % plot([-3.5, -3.5], [radius+10.5, 600], 'k--', 'LineWidth', 0.2);  % Vertical boundaries
    plot([-5.25, -5.25], [radius+10.5, 600], 'k--', 'LineWidth', 0.2);  % Vertical boundaries

    % plot([3.5, 3.5], [-600, -radius-10.5], 'k--', 'LineWidth', 0.2);  % Vertical boundaries
    plot([5.25, 5.25], [-600, -radius-10.5], 'k--', 'LineWidth', 0.2);  % Vertical boundaries

    % plot([3.5, 3.5], [radius+10.5, 600], 'k--', 'LineWidth', 0.2);  % Vertical boundaries
    plot([5.25, 5.25], [radius+10.5, 600], 'k--', 'LineWidth', 0.2);  % Vertical boundaries
    % Plot quarter-circle curves at the corners (curved paths)
    % Top-right corner
    % plot(radius * cos(theta), radius * sin(theta), 'g', 'LineWidth', 2);
    plot(30.5-radius * cos(theta), 30.5-radius * sin(theta), 'k', 'LineWidth', 2);
    % Top-left corner
    plot(-30.5+radius * cos(theta), 30.5-radius * sin(theta), 'k', 'LineWidth', 2);
    % Bottom-left corner
    plot(-30.5+radius * cos(theta), -30.5+radius * sin(theta), 'k', 'LineWidth', 2);
    % Bottom-right corner
    % plot(radius * cos(theta), -radius * sin(theta), 'g', 'LineWidth', 2);
    plot(30.5-radius * cos(theta), -30.5+radius * sin(theta), 'k', 'LineWidth', 2);

    % Plot the yellow dashed lines in the middle (centerlines)
    plot([-600, -radius], [0, 0], '-', 'Color', [1, 1, 0], 'LineWidth', 2); % Left yellow dashed line
    plot([radius, 600], [0, 0], '-', 'Color', [1, 1, 0], 'LineWidth', 2);   % Right yellow dashed line
    plot([0, 0], [radius, 600], '-', 'Color', [1, 1, 0], 'LineWidth', 2);   % Vertical yellow dashed line
    plot([0, 0], [-600, -radius], '-', 'Color', [1, 1, 0], 'LineWidth', 2); % Vertical yellow dashed line

    % Fix axis limits

    % Fix axis to avoid zooming
    axis([-320, 320, -320, 320]);
    axis equal;
    % ax = gca;
    % get(ax);
    % Make sure the curves are properly proportioned
    % Plot the control zone as a light-colored circle with transparency
    angle_points = linspace(0, 2*pi, 100); % Generate angle points
    x_circle = range_of_coordinator * cos(angle_points); % x-coordinates of the circle
    y_circle = range_of_coordinator * sin(angle_points); % y-coordinates of the circle
    % Plot the circle with a very light color and transparency
    fill(x_circle, y_circle, [0.9, 0.9, 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Light blue color with transparency
    % title([num2str(t)]); % Replace 'tif' with your variable name

    % hold on;

    plot(x_circle, y_circle, 'k--', 'LineWidth', 1); % Black dashed line for the outline
        title(sprintf('Time: %.2f', t));  % update title with current t


        
%% Following there some additional optional plots to visualize the coordinates of each path




        % 
        % %% paths corners
        % plot(30.5-2.625-radius * cos(theta), 30.5-2.625-radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % plot(-30.5+2.625+radius * cos(theta), 30.5-2.625-radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % 
        % plot(30.5-2.625-radius * cos(theta), -30.5+2.625+radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % 
        % plot(-30.5+2.625+radius * cos(theta), -30.5+2.625+radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % 
        % %% curves indide
        % plot(-30.5+2.625+10.5+radius * cos(theta), -30.5+2.625+10.5+radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % plot(30.5-2.625-10.5-radius * cos(theta), -30.5+2.625+10.5+radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % 
        % plot(-30.5+2.625+10.5+radius * cos(theta), 30.5-2.625-10.5-radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % 
        % plot(30.5-2.625-10.5-radius * cos(theta), 30.5-2.625-10.5-radius * sin(theta), 'b--', 'LineWidth', 0.8);
        % 
        % %% straight lines
        % 
        % plot([-40, 40], [-7.875, -7.875], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % 
        % plot([-40, 40], [+7.875, +7.875], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % 
        % plot([-7.875, -7.875],[-40, 40], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % plot([+7.875, +7.875],[-40, 40], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % 
        % plot([17.375, 40], [2.625, 2.625], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % plot([17.375, 40], [-2.625,-2.625], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % plot([-40,-17.375], [-2.625,-2.625], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % plot([-40,-17.375], [2.625,2.625], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % 
        % plot([-40,-17.375], [2.625,2.625], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        % 
        % plot([2.625,2.625],[-40,-17.375], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        %         plot([-2.625,-2.625],[-40,-17.375], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        %         plot([-2.625,-2.625],[17.375,40], 'b--', 'LineWidth', 0.8); % Left horizontal boundary
        % 
        %         plot([2.625,2.625],[17.375,40], 'b--', 'LineWidth', 0.8); % Left horizontal boundary




%  conflict points
% 
% plot(0, 7.72, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(0, -7.72, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(7.72, 0, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(-7.72, 0, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% 
% plot(7.875, 7.875, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(-7.875, -7.875, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(-7.875, 7.875, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(7.875, -7.875, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% 
% plot(27.3, -7.875, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(-7.875,-27.3, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(7.875, 27.3, 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary
% plot(-27.3,7.875 , 'yo', 'MarkerSize', 11, 'MarkerFaceColor', 'b'); % Yellow boundary



 








