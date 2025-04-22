function y = check_upcoming_flow_cavs(CAVs,id)

global vmax umax

if CAVs(id).path == 2
    conflict_path = 11;
elseif CAVs(id).path == 5
    conflict_path = 2;
elseif CAVs(id).path == 8
    conflict_path = 5;
elseif CAVs(id).path == 11
    conflict_path = 8;
end

list_of_confict_CAVs = find([CAVs.path] == conflict_path);

% Initialize an empty array to store the highest-p CAV after filtering
highest_p_CAV = [];

% Iterate over the list of conflicting CAVs
for i = length(list_of_confict_CAVs):-1:1  % Loop backward to safely remove elements
    % Check if the CAV's p value is less than 0
    if CAVs(list_of_confict_CAVs(i)).p < 0
        % Remove the CAV from the list
        list_of_confict_CAVs(i) = [];
    end
end

% After removing invalid CAVs, find the one with the highest p value
if ~isempty(list_of_confict_CAVs)
    % Extract p values from the remaining CAVs
    p_values = arrayfun(@(x) CAVs(x).p, list_of_confict_CAVs);
    
    % Find the index of the CAV with the highest p value
    [~, min_index] = min(p_values);
    
    % Store the corresponding CAV information
    highest_p_CAV = list_of_confict_CAVs(min_index);
end

% I deduct with 20 because the conflict points with respect to the i
% vehicle is at 20 meters after the reference 0 (the reference 0 is at the
% center of the intersection

if ~isempty(list_of_confict_CAVs)


d_i = abs(-20-CAVs(highest_p_CAV).p);
v_i0 = CAVs(highest_p_CAV).v;

check_point_distance = (vmax^2-v_i0^2)/2*umax;

if v_i0>1.5

    if d_i <= check_point_distance

        tau_i = (-v_i0 + sqrt(v_i0^2 + 2*umax*d_i))/(umax) ;
    else
        tau_i = (vmax - v_i0)/umax + (d_i-check_point_distance)/vmax ;
    end

else
    tau_i=100;
end

%Now find the tau_j based on the position of vehicle j on the secondary
%road.

d_j = (CAVs(id).p-28.25) + 2*pi*20/4;
%a norma acceleration as described in the manuscript
ucomf=4;

tau_j = (CAVs(id).v + sqrt(CAVs(id).v^2+2*ucomf*d_j))/ucomf;

tau_threshold = 0;

if tau_j < tau_i + tau_threshold
    y=1;
else
    y=0;
end

else

  y=1;

end


end