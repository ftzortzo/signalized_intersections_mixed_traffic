
for i = 1:number_of_CAVs
    if CAVs(i).p < 300 && CAVs(i).p > -300 
        if CAVs(i).p > (300 - 260/3) % first part of the incoming link
            if ~ismember(i,Pressure.Incoming1{CAVs(i).path}) && (CAVs(i).cant_cross_traffic_light==1 || CAVs(i).Type=="HDV")
                Pressure.Incoming1{CAVs(i).path}(end+1) = i;
            end            
        elseif CAVs(i).p > (300 - 2*260/3) % second part of the incoming link
            if ~ismember(i,Pressure.Incoming2{CAVs(i).path}) && (CAVs(i).cant_cross_traffic_light==1 || CAVs(i).Type=="HDV")
                Pressure.Incoming2{CAVs(i).path}(end+1) = i; 
            end

            if ismember(i, Pressure.Incoming1{CAVs(i).path}) 
            % Remove i from the array
            Pressure.Incoming1{CAVs(i).path} = Pressure.Incoming1{CAVs(i).path}(Pressure.Incoming1{CAVs(i).path} ~= i);
            end

        elseif CAVs(i).p > (300 - 3*260/3) % third part of the incoming link
            if ~ismember(i,Pressure.Incoming3{CAVs(i).path}) && (CAVs(i).cant_cross_traffic_light==1 || CAVs(i).Type=="HDV")
                Pressure.Incoming3{CAVs(i).path}(end+1) = i ;
            end

            if ismember(i, Pressure.Incoming2{CAVs(i).path}) 
                % Remove i from the array
                Pressure.Incoming2{CAVs(i).path} = Pressure.Incoming2{CAVs(i).path}(Pressure.Incoming2{CAVs(i).path} ~= i);
            end


        elseif CAVs(i).p < 40 % outgoing lane, after the traffic light
            if ~ismember(i,Pressure.Outgoing{CAVs(i).path})
                Pressure.Outgoing{CAVs(i).path}(end+1) = i ;
            end
            if ismember(i, Pressure.Incoming3{CAVs(i).path})
                % Remove i from the array
                Pressure.Incoming3{CAVs(i).path} = Pressure.Incoming3{CAVs(i).path}(Pressure.Incoming3{CAVs(i).path} ~= i);
            end
        end
    end
end


for path=1:12
    Pressure.Final_lane{path} = length(Pressure.Incoming1{path})*0.8 + length(Pressure.Incoming2{path}) + length(Pressure.Incoming3{path})*1.2; %- length(Pressure.Outgoing{path});
end


pres_mode_1_7 = Pressure.Final_lane{1} + Pressure.Final_lane{7};
pres_mode_4_10 = Pressure.Final_lane{4} + Pressure.Final_lane{10};
pres_mode_5_11 = Pressure.Final_lane{5} + Pressure.Final_lane{11};
pres_mode_2_8 = Pressure.Final_lane{2} + Pressure.Final_lane{8};


Pressure.Final = sum(cell2mat(Pressure.Final_lane));

if abs(t-15)<0.001 || abs(t-45)<0.001 || abs(t-75)<0.001 || abs(t-105)<0.001 || abs(t-135)<0.001 || abs(t-165)<0.001 || abs(t-195)<0.001 || abs(t-225)<0.001 || abs(t-255)<0.001  || abs(t-285)<0.001 || abs(t-315)<0.001 || abs(t-345)<0.001 && t>1

    if t>10
        sum_previous_cycles = sum_previous_cycles + sum(modeDurations(1:4));
    end



    % Pressures vector
    % pres_values = [pres_mode_2_8, pres_mode_5_11, pres_mode_4_10, pres_mode_1_7];

pres_values = [pres_mode_2_8, pres_mode_5_11, pres_mode_4_10 , pres_mode_1_7];     

lane_sets = {[2, 8], [5, 11], [4, 10], [1, 7]};

% Sort pressures in descending order
[sorted_pres, idx] = sort(pres_values, 'descend');

% Sorted lane sets
new_maxLengthSets = lane_sets(idx);

% Total cycle time
cycle_timee = 30;
min_duration = 4; % Minimum duration per mode

% Initially assign minimum duration to each mode
next_modeDurations = min_duration * ones(size(sorted_pres));

% Remaining time after minimum durations allocated
remaining_time = cycle_timee - sum(next_modeDurations);

% Allocate remaining time proportionally based on pressures
proportional_pres = sorted_pres / sum(sorted_pres);
additional_durations = round(proportional_pres * remaining_time);

% Adjust the durations
next_modeDurations = next_modeDurations + additional_durations;

% Correct possible mismatch due to rounding
diff_time = cycle_timee - sum(next_modeDurations);
while diff_time ~= 0
    [~, idx_max] = max(sorted_pres);
    next_modeDurations(idx_max) = next_modeDurations(idx_max) + sign(diff_time);
    diff_time = cycle_timee - sum(next_modeDurations);
end
    

   % next_modeDurations = [8,12,15,5]; % Sum should match cycle_timee
   % new_maxLengthSets={[2, 8],[5, 11],[4, 10],[1, 7]};
   maxLengthSets=[maxLengthSets(end-3:end),new_maxLengthSets]; 
   modeDurations=[modeDurations(end-3:end), next_modeDurations];
   traffic_light_change_flag=1;

end

if abs(t-30)<0.001 || abs(t-60)<0.001 || abs(t-90)<0.001 || abs(t-120)<0.001 || abs(t-150)<0.001 || abs(t-180)<0.001 || abs(t-210)<0.001 || abs(t-240)<0.001 || abs(t-270)<0.001 || abs(t-300)<0.001 && t>1

maxLengthSets = maxLengthSets(end-3:end);
modeDurations = modeDurations(end-3:end);

end











