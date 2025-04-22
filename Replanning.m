Copy_of_InitializeCoordinator



for se=1:length(sequence)
    id=sequence(se);

    [phis, CAVs] = Find_reference_plan_for_replanning(['Path', num2str(CAVs(id).path)], CAVs(id).v, t,CAVs,id);
    CAVs(id).phis = phis;

    Update_arrival_times_at_conflict_points;

    Add_preceeding_Vehicles

    if CAVs(id).Passed_control_zone==0
        CAVs(id).Passed_control_zone=1;
    end

end


