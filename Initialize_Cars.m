% Initialize the array for CAVs as an empty struct array
CAVs = struct('v', {}, 'p', {}, 'x', {}, 'y', {},'theta',{}, 'u', {},'vstart',[],'phis',[], 'path', {}, 'tf', {},'direction',[],'cant_cross_traffic_light',0,'prediction',-1,'Passed_control_zone',0,'Passed_traffic_light',0,'Preceding_CAV',{},'Lateral',struct('ID', [], 'Conflict_Point', [],'Distance_from_Conflict',[]),'Type',[],'Turn',[],'Conflict_Points',struct('ID',[],'time',[],'distance',[],'live_distance',[]),'Feasible_Time_Set',[],'Time_Interval_flag',-1,'Green_phases',[],'k',1,'flag_check',0);
phi=1;
for i = 1:number_of_CAVs

    while 1
        % Generate initial conditions for the CAV
        v = 10;
        % Initial velocity
        possible_paths = [1,2,4,5,7,8,10,11];
        index = randi(8);
        path = possible_paths(index);


        % Set an initial random position
        p = 300 + 20*i ; % Assuming initial positions between 0 and 100 (you can adjust this)

        % Check if any previous CAVs are on the same path
        min_diff=10^10;
        for j=1:i-1
            if CAVs(j).path == path
                diff_p = abs(CAVs(j).p - p);
                % Update if this vehicle has a smaller difference in p
                if diff_p < min_diff
                    min_diff = diff_p;
                    closest_vehicle_idx = j;
                end
            end
        end


        if exist('closest_vehicle_idx', 'var') == 1
            if CAVs(closest_vehicle_idx).p > p
                safe_distance = CAVs(closest_vehicle_idx).v*phi + gamma + 5;
            else
                safe_distance = v*phi + gamma;
            end


            if min_diff < safe_distance;
                continue;
            else
                break;
            end
        else
            break;
        end
    end

    if path == 1
        x = 2.625;
        y = -p;
        theta = pi/2; % 90 degrees (facing upwards towards the origin)
    elseif path == 2
        x = 7.875;
        y = -p;
        theta = pi/2; % 180 degrees (facing left towards the origin)
    elseif path == 3
        x = 8.75;
        y = -p;
        theta = pi/2; % -90 degrees (facing downwards towards the origin)



    elseif path == 4
        x = p;
        y = 2.625;
        theta = pi; % 0 degrees (facing right towards the origin)
    elseif path == 5
        x = p;
        y = 7.875;
        theta = pi; % 0 degrees (facing right towards the origin)
    elseif path == 6
        x = p;
        y = 8.75;
        theta = pi; % 0 degrees (facing right towards the origin)



    elseif path == 7
        x = -2.625;
        y = p;
        theta = pi + pi/2; % 180 degrees (facing left towards the origin)
    elseif path == 8
        x = -7.875;
        y = p;
        theta = pi + pi/2; % -90 degrees (facing downwards towards the origin)
    elseif path == 9
        x = -8.75;
        y = p;
        theta = pi + pi/2; % 0 degrees (facing right towards the origin)



    elseif path == 10
        x = -p;
        y = -2.625;
        theta = 0; % 0 degrees (facing right towards the origin)
    elseif path == 11
        x = -p;
        y = -7.875;
        theta = 0; % 0 degrees (facing right towards the origin)
    elseif path == 12
        x = -p;
        y = -8.75;
        theta = 0; % 0 degrees (facing right towards the origin)
        

    end

    type_vector=["CAV","HDV"];

    type = type_vector(randi(2));

    turn_vector=["left","straight","right"];

    if path==1 || path==4 || path==7 || path==10
        turn=turn_vector(1);
    else
        turn=turn_vector(randi(2)+1);
    end

    % Store the generated CAV structure including x0 and y0
    CAVs(i) = struct('v', v, 'p', p, 'x', x, 'y', y,'theta',theta, 'u', NaN, 'vstart',[],'phis',[], 'path', path, 'tf',NaN,'direction',[],'cant_cross_traffic_light',0,'prediction',-1,'Passed_control_zone',0,'Passed_traffic_light',0,'Preceding_CAV',-1,'Lateral',struct('ID', [], 'Conflict_Point', [],'Distance_from_Conflict',[]),'Type',type,'Turn',turn,'Conflict_Points',struct('ID',[],'time',[],'distance',[],'live_distance',[]),'Feasible_Time_Set',[],'Time_Interval_flag',-1,'Green_phases',[],'k',1,'flag_check',0);
end


save("_Last_Initial_Conditions",'CAVs')