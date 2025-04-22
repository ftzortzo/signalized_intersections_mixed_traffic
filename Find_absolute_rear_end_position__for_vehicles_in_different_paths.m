if (CAVs(i)==2 && CAVs(CAVs(i).Preceding_CAV)==11) || (CAVs(i)==11 && CAVs(CAVs(i).Preceding_CAV)==2)
    s = CAVs(CAVs(i).Preceding_CAV).x - CAVs(i).x;
elseif (CAVs(i)==2 && CAVs(CAVs(i).Preceding_CAV)==5) || (CAVs(i)==5 && CAVs(CAVs(i).Preceding_CAV)==2)
    s = CAVs(CAVs(i).Preceding_CAV).y - CAVs(i).y;
elseif (CAVs(i)==5 && CAVs(CAVs(i).Preceding_CAV)==8) || (CAVs(i)==8 && CAVs(CAVs(i).Preceding_CAV)==5)
    s = CAVs(i).x - CAVs(CAVs(i).Preceding_CAV).x ;
elseif (CAVs(i)==8 && CAVs(CAVs(i).Preceding_CAV)==11) || (CAVs(i)==11 && CAVs(CAVs(i).Preceding_CAV)==8)
    s = CAVs(i).x - CAVs(CAVs(i).Preceding_CAV).x ;
end

