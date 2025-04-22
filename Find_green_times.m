path_number = CAVs(i).path;


flag=0;
Cycles_look_ahead=7;



for m=modeIndex:length(modeDurations)

    if ismember(path_number, maxLengthSets{m})
        if flag~=1

            if m==modeIndex
                t0 = t ;
            else
                t0 = t + time_until_next_node + sum(modeDurations(modeIndex+1:m-1));
            end
            flag = 1;

        end

    end

    if ismember(path_number, maxLengthSets{m}) && m < length(maxLengthSets)

        if ismember(path_number, maxLengthSets{m}) && ~ismember(path_number, maxLengthSets{m+1})

            if m==modeIndex
                tf = t+ time_until_next_node;
            else
                tf = t+ time_until_next_node + sum(modeDurations(modeIndex+1:m));
            end
            CAVs(i).Green_phases = [CAVs(i).Green_phases; t0 ,tf];
            flag=0;
        end

    elseif ismember(path_number, maxLengthSets{m}) && m == length(maxLengthSets)

        tf = sum(modeDurations);
       CAVs(i).Green_phases = [CAVs(i).Green_phases; t0 ,tf];
        flag=0;

    end

end


% If you put adaptive signals, remove the following code

flag1=0;
extra_phases=[];
for m=1:length(modeDurations)
    if ismember(path_number,maxLengthSets{m})
        if flag1==0
            t0 = sum(modeDurations(1:m)) - modeDurations(m);
            flag1=1;
        end
    end

    if ismember(path_number, maxLengthSets{m}) && m < length(maxLengthSets)

        if ismember(path_number, maxLengthSets{m}) && ~ismember(path_number, maxLengthSets{m+1})
            tf=sum(modeDurations(1:m));
            extra_phases=[extra_phases; t0, tf];
            flag1=0;
        end

    elseif ismember(path_number, maxLengthSets{m}) && m == length(maxLengthSets)

        tf = sum(modeDurations);
        extra_phases=[extra_phases; t0, tf];
        flag1=0;        


    end

end



for c=1:Cycles_look_ahead

    CAVs(i).Green_phases = [CAVs(i).Green_phases; extra_phases+c*sum(modeDurations)];

end


CAVs(i).Green_phases = unique(CAVs(i).Green_phases, 'rows', 'stable');








