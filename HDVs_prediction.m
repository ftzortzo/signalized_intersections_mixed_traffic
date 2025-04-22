% IDM Parameters
a = 4;          % max acceleration (m/s^2)
b = 2;            % comfortable deceleration (m/s^2)
v0 = 15;          % desired speed (m/s)
delta = 4;        % acceleration exponent
T = 1;          % desired time headway (s)
s0 = 5;           % minimmum distance

if abs(t-Simulation_Time)<0.001
    return
end

if CAVs(i).prediction==-1
    if CAVs(i).Preceding_CAV==-1

        % I dont have preceding vehicle
        x0 = [300-CAVs(i).p; CAVs(i).v];             % State vector: only vehicle 1

        % Simulation time span
        tspan = linspace(t,Simulation_Time,3000);

        % Define ODE system
        idm_ode= @(t, x) conditional_idm_ode_without_preceding(t, x, a, b, delta, T, s0, CAVs(i).path, v0,cumulativeDurations, cycle_time,maxLengthSets,modeDurations);
        % idm_ode = @(t, x) multi_vehicle_ODE(t, x, a, b,CAVs(i).path, delta, T, s0, v_des, cumulativeDurations, cycle_time, maxLengthSets, modeDurations)

        % Solve the system
        [t_1, x] = ode15s(idm_ode, tspan, x0);

        % Extract positions and velocities
        CAVs(i).p1 = x(:,1);
        CAVs(i).v1 = x(:,2);
        CAVs(i).t = t_1;
        CAVs(i).prediction=1;

    elseif CAVs(CAVs(i).Preceding_CAV).Type=="HDV"

        [~, idx] = min(abs(t - CAVs(i).t));

        % x0 = [300-CAVs(i).p; CAVs(i).v;300-CAVs(CAVs(i).Preceding_CAV).p; CAVs(CAVs(i).Preceding_CAV).v];             % State vector: only vehicle 1
          x0 = [300-CAVs(i).p; CAVs(i).v];             % State vector: only vehicle 1


        tspan = linspace(t,Simulation_Time,3000);

        idm_ode= @(t, x) conditional_idm_ode_with_preceding(t, x, a, b, delta, T, s0, CAVs,i, v0, cumulativeDurations, cycle_time, maxLengthSets, modeDurations);
        % idm_ode= @(t, x) multi_vehicle_ODE(t, x, a, b,CAVs(i).path, delta, T, s0, v_des, cumulativeDurations, cycle_time, maxLengthSets, modeDurations)

        % Solve the system
        [t_1, x] = ode15s(idm_ode, tspan, x0);

        % Extract positions and velocities
        CAVs(i).p1 = x(:,1);
        CAVs(i).v1 = x(:,2);
        CAVs(i).t = t_1;
        CAVs(i).prediction=1;

    elseif CAVs(CAVs(i).Preceding_CAV).Type=="CAV"
        %
         x0 = [300-CAVs(i).p; CAVs(i).v];             % State vector: only vehicle 1
        %
        tspan = linspace(t,Simulation_Time,3000);
        %
        idm_ode= @(t, x) conditional_idm_ode_with_preceding_CAV(t, x, a, b, delta, T, s0, CAVs,i, v0, cumulativeDurations, cycle_time, maxLengthSets, modeDurations);

        % Solve the system
        [t_1, x] = ode15s(idm_ode, tspan, x0);

        % Extract positions and velocities
        CAVs(i).p1 = x(:,1);
        CAVs(i).v1 = x(:,2);
        CAVs(i).t = t_1;
        CAVs(i).prediction=1;
    end

end

    % 
    % [~, idx] = min(abs(t - CAVs(i).t));
    % 
    % (300-CAVs(i).p)-CAVs(i).p1(idx);






    function dxdt = conditional_idm_ode_without_preceding(t, x, a, b, delta, T, s0, path, v_des,cumulativeDurations, cycle_time,maxLengthSets,modeDurations)

    p_i = x(1);
    v_i = x(2);

    % Get the current time in the cycle
    current_time_in_cycle = mod(t, cycle_time);

    % Find the current mode based on the current time in the cycle
    modeIndex = find(current_time_in_cycle <= cumulativeDurations, 1);
    currentMode = maxLengthSets{modeIndex};

    % ---- IDM Behavior (based on preceding vehicle if any) ----
    % Free-road IDM (no preceding vehicle)
    u_IDM = a * (1 - (v_i / v_des)^delta);

    distance_to_light = 260-p_i;
    time_at_next_mode = floor(t/cycle_time)*cycle_time + sum(modeDurations(1:modeIndex));
    time_until_next_node = time_at_next_mode - t;

    % ---- Traffic Light Logic ----
    % Assume traffic_light_phase = 1 for RED, 0 for GREEN (example)
    if ((~ismember(path,currentMode) || (ismember(path,currentMode) && time_until_next_node < 3 && abs(distance_to_light - (+40))>(v_i^2)/(2*3.5) )) ) && distance_to_light > 0
        % Vehicle needs to decide if it should stop
        % Only act if vehicle is beyond the light (positive distance)
        % Similar logic to what you wrote
        s_tr_light = distance_to_light;
        Dv_tr_light = v_i;  % No relative speed, light is static
        s_star_tr_light = s0 + v_i * T + (v_i * Dv_tr_light) / (2 * sqrt(a * b));
        u_tr_light = a * (1 - (v_i / v_des)^delta - (s_star_tr_light / s_tr_light)^2);
    else
        u_tr_light = a;  % Green light, no restriction
    end
    % ---- Combine IDM and Traffic Light Acceleration ----
    acc = min(u_IDM, u_tr_light);
    % ---- Return Derivatives ----
    dxdt = [v_i;acc] ;  % dv/dt = acc
    end


    function dxdt = conditional_idm_ode_with_preceding(t, x, a, b, delta, T, s0, CAVs, i, v_des,cumulativeDurations, cycle_time,maxLengthSets,modeDurations)

    p_i = x(1);
    v_i = x(2);

    % Get the current time in the cycle
    current_time_in_cycle = mod(t, cycle_time);

    % Find the current mode based on the current time in the cycle
    modeIndex = find(current_time_in_cycle <= cumulativeDurations, 1);
    currentMode = maxLengthSets{modeIndex};

    distance_to_light = 260-p_i;
    time_at_next_mode = floor(t/cycle_time)*cycle_time + sum(modeDurations(1:modeIndex));
    time_until_next_node = time_at_next_mode - t;


    % ---- IDM Behavior (based on preceding vehicle. Recall we have a preceding vehicle) ----
    % Free-road IDM (no preceding vehicle)

    [~, idx] = min(abs(CAVs(CAVs(i).Preceding_CAV).t - t));

    s = abs (p_i- CAVs(CAVs(i).Preceding_CAV).p1(idx));

    Dv = v_i - CAVs(CAVs(i).Preceding_CAV).v1(idx);

    s_star = s0 + v_i * T + (v_i * Dv) / (2 * sqrt(a * b));

    u_preced = a*(1-(v_i/v_des)^delta - (s_star/s)^2 );

    % ---- Traffic Light Logic ----
    % Assume traffic_light_phase = 1 for RED, 0 for GREEN (example)
    if ((~ismember(CAVs(i).path,currentMode) || (ismember(CAVs(i).path,currentMode) && time_until_next_node < 3 && abs(distance_to_light - (+40))>(v_i^2)/(2*3.5) )) ) && distance_to_light > 0
        % Vehicle needs to decide if it should stop
        % Only act if vehicle is beyond the light (positive distance)
        % Similar logic to what you wrote
        s_tr_light = distance_to_light;
        Dv_tr_light = v_i;  % No relative speed, light is static
        s_star_tr_light = s0 + v_i * T + (v_i * Dv_tr_light) / (2 * sqrt(a * b));
        u_tr_light = a * (1 - (v_i / v_des)^delta - (s_star_tr_light / s_tr_light)^2);
    else
        u_tr_light = a;  % Green light, no restriction
    end


    % ---- Combine IDM and Traffic Light Acceleration ----
    acc = min(    u_preced, u_tr_light);
    % ---- Return Derivatives ----
    dxdt = [v_i;acc] ;  % dv/dt = acc

    end



    function dxdt = conditional_idm_ode_with_preceding_CAV(t, x, a, b, delta, T, s0, CAVs, i, v_des,cumulativeDurations, cycle_time,maxLengthSets,modeDurations)

    p_i = x(1);
    v_i = x(2);

    % Get the current time in the cycle
    current_time_in_cycle = mod(t, cycle_time);

    % Find the current mode based on the current time in the cycle
    modeIndex = find(current_time_in_cycle <= cumulativeDurations, 1);
    currentMode = maxLengthSets{modeIndex};

    distance_to_light = 260-p_i;
    time_at_next_mode = floor(t/cycle_time)*cycle_time + sum(modeDurations(1:modeIndex));
    time_until_next_node = time_at_next_mode - t;


    % ---- IDM Behavior (based on preceding vehicle. Recall we have a preceding vehicle) ----
    % Free-road IDM (no preceding vehicle)

    phis = CAVs(CAVs(i).Preceding_CAV).phis

    p_preced = phis(1)*t^3 + phis(2)*t^2 + phis(3)*t + phis(4) ; 
    
    v_preced = 3*phis(1)*t^2 + 2*phis(2)*t + phis(3) 

    s = abs (p_i- p_preced );

    Dv = v_i -  v_preced;

    s_star = s0 + v_i * T + (v_i * Dv) / (2 * sqrt(a * b));

    u_preced = a*(1-(v_i/v_des)^delta - (s_star/s)^2 );

    % ---- Traffic Light Logic ----
    % Assume traffic_light_phase = 1 for RED, 0 for GREEN (example)
    if ((~ismember(CAVs(i).path,currentMode) || (ismember(CAVs(i).path,currentMode) && time_until_next_node < 3 && abs(distance_to_light - (+40))>(v_i^2)/(2*3.5) )) ) && distance_to_light > 0
        % Vehicle needs to decide if it should stop
        % Only act if vehicle is beyond the light (positive distance)
        % Similar logic to what you wrote
        s_tr_light = distance_to_light;
        Dv_tr_light = v_i;  % No relative speed, light is static
        s_star_tr_light = s0 + v_i * T + (v_i * Dv_tr_light) / (2 * sqrt(a * b));
        u_tr_light = a * (1 - (v_i / v_des)^delta - (s_star_tr_light / s_tr_light)^2);
    else
        u_tr_light = a;  % Green light, no restriction
    end


    % ---- Combine IDM and Traffic Light Acceleration ----
    acc = min(u_preced, u_tr_light);
    % ---- Return Derivatives ----
    dxdt = [v_i;acc] ;  % dv/dt = acc

    end
