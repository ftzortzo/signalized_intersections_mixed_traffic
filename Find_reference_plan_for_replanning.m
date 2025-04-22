function [phis, CAVs] = Find_reference_plan_for_replanning(path, v_start, t0,CAVs,se,interval)


global vmax vmin umax umin length_of_control_zone


path_number = str2double(regexp(path, '\d+$', 'match')); % Note! path_number is just a double while the input path is a string in the form 'Path1'.



if path_number==1 || path_number == 4 || path_number==7 || path_number == 10
    length_of_control_zone = 600 - 2*17.375 + (1/2)*pi*20; % we have put 20 here because 20 is the range of the cycle whose tetarimorio is the curve the vehicle is on
elseif (path_number==2 || path_number == 5 || path_number==8 || path_number == 11) && CAVs(se).Turn == "right"

    length_of_control_zone = 600 - 2*27.875 + (1/2)*pi*20 ;% we have put 20 here because 20 is the range of the cycle whose tetarimorio is the curve the vehicle is on
else
    length_of_control_zone = 600;
end

remaining_control_zone = abs(length_of_control_zone + (CAVs(se).p) - 300);

center_cycle_path{1} = [-17.3750, -17.3750];
center_cycle_path{2} = [27.875, -27.875];
center_cycle_path{4} = [17.3750, -17.3750];
center_cycle_path{5} = [27.875, 27.875];
center_cycle_path{7} = [17.3750, 17.3750];
center_cycle_path{8} = [-27.875, 27.875];
center_cycle_path{10} = [-17.3750, 17.3750];
center_cycle_path{11} = [-27.875, -27.875];


initial_cycle_path{1} = [2.625, -17.3750];
initial_cycle_path{2} = [7.875, -27.875];
initial_cycle_path{4} = [17.3750, 2.625];
initial_cycle_path{5} = [27.875, 7.875];
initial_cycle_path{7} = [-2.625, 17.3750];
initial_cycle_path{8} = [-7.875, 27.875];
initial_cycle_path{10} = [-17.3750, -2.625];
initial_cycle_path{11} = [-27.875, -7.875];


% Find the earliest exit time from the intersection

tf_u = 6 * remaining_control_zone / (3*v_start + sqrt(3)*sqrt(3*v_start^2 + 4*remaining_control_zone*umax));
tf_v = 3 * remaining_control_zone / (v_start + 2 * vmax);
tif=t0+max(tf_u,tf_v);
tif0=tif;


if (9*v_start^2 + 12*remaining_control_zone*umin > 0)
    tf_u = (sqrt(9*v_start^2 + 12*remaining_control_zone*umin) - 3*v_start)/(2*umin);
else
    tf_u = -1;
end
tf_v = 3 * remaining_control_zone / (v_start + 2 * vmin);
tif_max = t0 + max(tf_v, tf_u);



% %% This needs attention!!
% tif=interval(1);
% tif_max = interval(2);




flag_unconstrained=0;

% load('coordinator.mat')
CAVs(se).cant_cross_traffic_light=0;

while 1

    if tif>t0+80
        phis =[];
        CAVs=[];
        return
    end

    A = [t0^3 t0^2 t0 1; 3*t0^2 2*t0 1 0; tif^3 tif^2 tif 1; 6*tif 2 0 0];

    b = [(300-CAVs(se).p); v_start; length_of_control_zone; 0];

    phis = (A^-1)*b;

    if CAVs(se).Passed_traffic_light==0
        t = linspace(t0, tif, 5000);
        p_new = polyval(phis, t);

        [~, idx] = min(abs(p_new - 256));
        t_value = t(idx);

        green_phases_number_of_rows_clomuns = size(CAVs(se).Green_phases);
        green_phases_number_of_rows = green_phases_number_of_rows_clomuns(1);
        flag_passed_green=0;

        for j=1:green_phases_number_of_rows
            if t_value > CAVs(se).Green_phases(j,1) && t_value < CAVs(se).Green_phases(j,2)
                flag_passed_green=1;
                break;
            end
        end

        if flag_passed_green==0
            tif=tif+0.01;
            continue;
        end

    end


    %Rear_End Constraints

    if CAVs(se).Preceding_CAV==-1

        break;

    elseif CAVs(CAVs(se).Preceding_CAV).Type=="CAV"
        t = linspace(t0, tif, 1000);
        p_new = polyval(phis, t);
        p = polyval(CAVs(CAVs(se).Preceding_CAV).phis, t);

        % p-p_new-15
        if sum((p-p_new-10.5)>0.001)==1000
            break; % disp('Rear-End constraint is satisfied')
        else
            % disp('Rear-end constraint is violated')
            tif=tif+0.01;
            continue;
        end

    elseif CAVs(CAVs(se).Preceding_CAV).Type=="HDV"

        t = linspace(t0, tif, 1000);
        p_new = polyval(phis, t);
        pos_pre_aligned_with_p_new = interp1(CAVs(CAVs(se).Preceding_CAV).t, CAVs(CAVs(se).Preceding_CAV).p1, t, 'linear', 'extrap');

        if sum((pos_pre_aligned_with_p_new-p_new-10.5)>0.001)== length(pos_pre_aligned_with_p_new)
            break; % disp('Rear-End constraint is satisfied')
        else
            % disp('Rear-end constraint is violated')
            tif=tif+0.01;
            continue;
        end

    end

end

CAVs(se).Feasible_Time_Set = [tif0,tif_max,tif];
CAVs(se).t0 = t0;
CAVs(se).tf = tif;





