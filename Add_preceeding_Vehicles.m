% if Replanning_flag==1;
%     i=id;
% end

min_diff = 1000;
for j=1:number_of_CAVs
    if i~=j
        if CAVs(i).path == CAVs(j).path

            if CAVs(i).p > CAVs(j).p

                if CAVs(i).p - CAVs(j).p < min_diff
                    min_diff = CAVs(i).p - CAVs(j).p;
                    CAVs(i).Preceeding=j;
                end

            end
        end
    end
end