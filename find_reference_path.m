function [phis, CAVs] = find_reference_path(path, v_start, t0,x,y,CAVs,i)


global vmax vmin umax umin length_of_control_zone

id=i;
path_number = str2double(regexp(path, '\d+$', 'match')); % Note! path_number is just a double while the input path is a string in the form 'Path1'.


if path_number==1 || path_number == 4 || path_number==7 || path_number == 10
    length_of_control_zone = 600 - 2*17.375 + (1/2)*pi*20 ;% we have put 20 here because 20 is the range of the cycle whose tetarimorio is the curve the vehicle is on
elseif (path_number==2 || path_number == 5 || path_number==8 || path_number == 11) && CAVs(id).Turn == "right"

    length_of_control_zone = 600 - 2*27.875 + (1/2)*pi*20; % we have put 20 here because 20 is the range of the cycle whose tetarimorio is the curve the vehicle is on
else
    length_of_control_zone = 600;
end


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

tf_u = 6 * length_of_control_zone / (3*v_start + sqrt(3)*sqrt(3*v_start^2 + 4*length_of_control_zone*umax));
tf_v = 3 * length_of_control_zone / (v_start + 2 * vmax);
tif=t0+max(tf_u,tf_v);
tif0=tif;


if (9*v_start^2 + 12*length_of_control_zone*umin > 0)
    tf_u = (sqrt(9*v_start^2 + 12*length_of_control_zone*umin) - 3*v_start)/(2*umin);
else
    tf_u = -1;
end

vmin;
v_start;
tf_v = 3 * length_of_control_zone / (v_start + 2 * vmin);
tif_max = t0 + max(tf_v, tf_u);

id=i;
CAVs(id).Feasible_Time_Set = [tif0,tif_max];




flag_unconstrained=0;

% load('coordinator.mat')

while 1


    if tif>=tif_max && CAVs(i).cant_cross_traffic_light==0

            distance_untill_traffi_light = 256;

            CAVs(i).cant_cross_traffic_light=1;
            
            u_1 = (-4*v_start^2)/(6*distance_untill_traffi_light);

            u_final = max(u_1,umin);


            t_1 = (-2*v_start + sqrt(4*v_start^2+6*u_final*distance_untill_traffi_light)) / u_final;
            t_2 = (-2*v_start - sqrt(4*v_start^2+6*u_final*distance_untill_traffi_light)) / u_final;

            t_candidates = [t_1, t_2];
            positive_ts = t_candidates(t_candidates > 0);
            tif = t0 + min(positive_ts);


            % Delta= (-umin/3+2*v_start/3)^2 + 4* 260 *(umin/2);   %260 is the position of the traffic light
            % tif = (-(-umin/3+2*v_start/3)+sqrt(Delta))/(2*(umin/2))

    end


    % 


    if CAVs(i).cant_cross_traffic_light==0

    A = [t0^3 t0^2 t0 1; 3*t0^2 2*t0 1 0; tif^3 tif^2 tif 1; 6*tif 2 0 0];

    b = [0; v_start; length_of_control_zone; 0];

    phis = (A^-1)*b;

    else
       
    A = [t0^3 t0^2 t0 1; 3*t0^2 2*t0 1 0; tif^3 tif^2 tif 1;3*tif^2 2*tif 1 0];

    b = [0; v_start; 256; 0];

    phis = (A^-1)*b;

    break;

    end



    
        % Traffic light constraints

        %% Get set of green time intervals



    %     % Get a vector of the times position 160 is available

    if CAVs(i).cant_cross_traffic_light == 0
        t = linspace(t0, tif, 5000);
        p_new = polyval(phis, t);

        [~, idx] = min(abs(p_new - 256));
        t_value = t(idx);

        green_phases_number_of_rows_clomuns = size(CAVs(id).Green_phases);
        green_phases_number_of_rows = green_phases_number_of_rows_clomuns(1);
        flag_passed_green=0;

        for j=1:green_phases_number_of_rows
            if t_value > CAVs(i).Green_phases(j,1) && t_value < CAVs(i).Green_phases(j,2)
            flag_passed_green=1;
            end
        end

        if flag_passed_green==0
           tif=tif+0.01;
            continue;
        end
    end



    % Rear End Constraints

    if CAVs(i).Preceding_CAV==-1

        break;

    elseif CAVs(CAVs(i).Preceding_CAV).Type=="CAV"
        t = linspace(t0, tif, 1000);
        p_new = polyval(phis, t);
        p = polyval(CAVs(CAVs(i).Preceding_CAV).phis, t);

        % p-p_new-15
        if sum((p-p_new-10.5)>0.001)==1000
            break; % disp('Rear-End constraint is satisfied')
        else
            % disp('Rear-end constraint is violated')
            tif=tif+0.01;
            continue;
        end
        
    elseif CAVs(CAVs(i).Preceding_CAV).Type=="HDV"

        t = linspace(t0, tif, 1000);
        p_new = polyval(phis, t);
        pos_pre_aligned_with_p_new = interp1(CAVs(CAVs(i).Preceding_CAV).t, CAVs(CAVs(i).Preceding_CAV).p1, t, 'linear', 'extrap');

        if sum((pos_pre_aligned_with_p_new-p_new-10.5)>0.001)== length(pos_pre_aligned_with_p_new)
            break; % disp('Rear-End constraint is satisfied')
        else
            % disp('Rear-end constraint is violated')
            tif=tif+0.01;
            continue;
        end

    end
end


    

