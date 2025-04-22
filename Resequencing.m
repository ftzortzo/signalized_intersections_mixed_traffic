CAVs_CZ = []; %Initiaalize the vecotr that tracks the vehicles inside the control zone

number_of_paths=12;

clear chain
for c=1:number_of_paths
chain(c) = struct('IDs',[],'pos',[],'p_factor',[],'p_max',-1); % Initialize the chains.
end


%Update feasible_time_set and Feasible_Interval for each CAV
for i=1:number_of_CAVs
    distance_from_coordinator = sqrt(CAVs(i).x^2 + CAVs(i).y^2 );
    if distance_from_coordinator < range_of_coordinator
        CAVs_CZ = [CAVs_CZ i]
        S = abs(-100-(CAVs(i).p))

        [tif_min tif_max] = Find_Feasible_Time_set(CAVs(i).v,t0,S);

        CAVs(i).Feasible_Time_Set = [tif_min tif_max];
        CAVs(i).Feasible_Interval = tif_max - tif_min;
    end
end

%Get in array A the values of the Feasile Intervals of the vehicles that
%are inside the control  zone
A = [CAVs(CAVs_CZ).Feasible_Interval];


%Sort the vehiicles according to ther Feasible Interval. I should the
%substitutions need to be made. For exaple if I(2)=4, iteams that the
%vehicle with index 4 in CAVs has the second smaller feasible time set
[B,I] = sort(A);

%Change CAV_CZ with the shorted indeces
CAVs_CZ = CAVs_CZ(I);

%Create a vector that starts from the number of CAV_CZ and goes towards 1.
priorFactor  = numel(CAVs_CZ):-1:1;

for pp=1:length(CAVs_CZ)
    %Find the ID of the vehicle with index pp inside the CAV_CZ
    index_CZ = CAVs_CZ(pp)
    %add the corresponding weight
    CAVs(index_CZ).last_weight = priorFactor(pp)/length(CAVs_CZ);
    %add the corresponding processing_time
    CAVs(index_CZ).Processing_Time = CAVs(index_CZ).Feasible_Time_Set(1);
end

%access all the vehicles inside the CAVs_CZ
for ii=1:length(CAVs_CZ)
    %id2 is the ID of the vehicle at the index ii inside the CAV_CZ
    id2 =  CAVs_CZ(ii)
    %access all the paths
    for c=1:number_of_paths
        % if this vehicle is not a member of the list chain(c).IDs, add it
        % and also add its position
        if ~ismember(id2,chain(c).IDs)
            if CAVs(id2).path == c
                chain(c).IDs = [chain(c).IDs , id2]
                chain(c).pos = [chain(c).pos, CAVs(id2).p]
            end
        end
    end 
end

% sort the chains according to the positions such that they reflect the
% chain constraint
for i = 1:12
    % Sort the positions 'p' and get the sorting order
    [sortedPositions, sortIdx] = sort(chain(i).pos, 'ascend');
    
    % Use the sorting order to rearrange 'IDs'
    chain(i).pos = sortedPositions;
    chain(i).IDs = chain(i).IDs(sortIdx);
end

%Access all the paths
for ii = 1:number_of_paths
        %access the length of the corresponding chain and then the IDS in
        %it
    for jj=1:length(chain(ii).IDs)
        % id1 is the index in the current chain ii
        id1=jj;
        weight_sum = 0;
        pros_sum=0;
        % we start caluclating p_factor, from k=1 up to k=jj
        for k=1:id1
            id2 = chain(ii).IDs(id1);
            weight_sum = weight_sum + CAVs(id2).last_weight;
            pros_sum = pros_sum + CAVs(id2).Processing_Time;
        end
        if ~isempty(weight_sum)
        chain(ii).p_factor(id1) = weight_sum/pros_sum;
        end
    end
end

for ii = 1 : number_of_paths
        pmax=0;
        for jj=1:length(chain(ii).IDs)
            if chain(ii).p_factor(jj)>pmax
                pmax = chain(ii).p_factor(jj);
                chain(ii).p_max = chain(ii).p_factor(jj);
                chain(ii).p_index = jj;
                chain(ii).p_ID = chain(ii).IDs(jj);
            end
        end
end



sequence=[];

while sum([chain.IDs])>0

    [p_max, l_max] = max([chain.p_max])
    Id_max = chain(l_max).p_ID

    subsequence = [];

    for pp = 1:chain(l_max).p_index
        subsequence = [subsequence, chain(l_max).IDs(pp)] ;
    end

    delete_index = chain(l_max).p_index

    chain(l_max).IDs = chain(l_max).IDs(delete_index+1:end);
    chain(l_max).pos = chain(l_max).pos(delete_index+1:end);
    chain(l_max).p_factor = chain(l_max).p_factor(delete_index+1:end);
    chain(l_max).p_max=-1;
    chain(l_max).p_index=[];
    chain(l_max).p_ID=[];

    sequence = [sequence subsequence];

    for ii = 1 : number_of_paths
        pmax=0;
        for jj=1:length(chain(ii).IDs)
            if chain(ii).p_factor(jj)>pmax
                pmax = chain(ii).p_factor(jj);
                chain(ii).p_max = chain(ii).p_factor(jj);
                chain(ii).p_index = jj;
                chain(ii).p_ID = chain(ii).IDs(jj);
            end
        end
    end

end
















