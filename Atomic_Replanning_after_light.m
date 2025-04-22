if CAVs(i).path==1 || CAVs(i).path == 4 || CAVs(i).path==7 || CAVs(i).path== 10
    length_of_control_zone = 600 - 2*17.375 + (1/2)*pi*20; % we have put 20 here because 20 is the range of the cycle whose tetarimorio is the curve the vehicle is on
elseif (CAVs(i).path==2 || CAVs(i).path == 5 || CAVs(i).path==8 || CAVs(i).path == 11) && CAVs(i).Turn == "right"
    length_of_control_zone = 600 - 2*27.875 + (1/2)*pi*20 ;% we have put 20 here because 20 is the range of the cycle whose tetarimorio is the curve the vehicle is on
else
    length_of_control_zone = 600;
end

remaining_control_zone = abs(length_of_control_zone + (CAVs(i).p) - 300);

tf_u = 6 * remaining_control_zone / (3*CAVs(i).v + sqrt(3)*sqrt(3*CAVs(i).v^2 + 4*remaining_control_zone*umax));
tf_v = 3 * remaining_control_zone / (CAVs(i).v + 2 * vmax);
tif=t+max(tf_u,tf_v);
tif0=tif;


if (9*CAVs(i).v^2 + 12*remaining_control_zone*umin > 0)
    tf_u = (sqrt(9*CAVs(i).v^2 + 12*remaining_control_zone*umin) - 3*CAVs(i).v)/(2*umin);
else
    tf_u = -1;
end

tf_v = 3 * remaining_control_zone / (CAVs(i).v + 2 * vmin);
tif_max = t + max(tf_v, tf_u);


% The following commands refer to the case where I am searching to find the
% uncontrained trajectories associated to the boundary values. I want in
% the case of replaning before the light because I can identify the
% crossing times associated with these unconstrained trajectories!

% A1 = [t^3 t^2 t 1; 3*t^2 2*t 1 0; tif^3 tif^2 tif 1; 6*tif 2 0 0];
% 
% b1 = [300-CAVs(i).p; CAVs(i).v; length_of_control_zone; 0];
% 
% phis1 = (A1^-1)*b1;
% 
% t1 = linspace(t, tif0, 8000);
% p_1 = polyval(phis1, t1);
% [~, idx] = min(abs(p_1 - 255));
% [~, idx1] = min(abs(p_1 - length_of_control_zone));
% t_value1 = t1(idx);
% t_valuefin1 = t1(idx1);
% 
% A2 = [t^3 t^2 t 1; 3*t^2 2*t 1 0; tif_max^3 tif_max^2 tif_max 1; 6*tif_max 2 0 0];
% 
% b2 = [300-CAVs(i).p; CAVs(i).v; length_of_control_zone; 0];
% 
% phis2 = (A2^-1)*b2;
% 
% t2 = linspace(t, tif_max, 8000);
% p_2 = polyval(phis2, t2);
% [~, idx] = min(abs(p_2 - 255));
% [~, idx2] = min(abs(p_2 - length_of_control_zone));
% t_value2 = t2(idx);
% t_valuefin2 = t2(idx2);

id=i;
CAVs(i).Passed_traffic_light=1;

[newphis, newCAVs] = Find_reference_plan_for_replanning(['Path', num2str(CAVs(id).path)], CAVs(id).v, t,CAVs,id,CAVs(i).Feasible_Time_Set);

% Mesa sto Find_reference, iparxei mia periptosi pou girnao empty CAVs and
% phis. Prepei na dv ti kano se auti tin periptosi.

if ~isempty(newphis)
    CAVs(i).phis=newphis;
end


















