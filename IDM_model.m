% IDM Parameters
a = 4;          % max acceleration (m/s^2)
b = 2;            % comfortable deceleration (m/s^2)
v0 = 15;          % desired speed (m/s)
delta = 4;        % acceleration exponent
T = 1;          % desired time headway (s)
s0 = 5;           % minimmum distance


% Check if I need to stop due to red light
if ((~ismember(CAVs(i).path,currentMode) || (ismember(CAVs(i).path,currentMode) && time_until_next_node < 1 && abs(CAVs(i).p - (+40))>(CAVs(i).v^2)/(2*3.5) )) ) && (CAVs(i).p - (+40))>0

        s_tr_light = abs(CAVs(i).p - (+40));

        Dv_tr_light = CAVs(i).v;

        s_star_tr_light = s0 + CAVs(i).v * T + CAVs(i).v * Dv_tr_light / (2*sqrt(a*b));

        u_tr_light =  a*(1-(CAVs(i).v/v0)^delta - (s_star_tr_light/s_tr_light)^2) ;

else

    u_tr_light = a;

end

if CAVs(i).Preceding_CAV==-1

    u_preced = a*(1-(CAVs(i).v/v0)^delta);

else

    % we have strong issue here because .p is not correct when the
    % preceding vehicle comes from a different path.

    if CAVs(i).path ~= CAVs(CAVs(i).Preceding_CAV).path

        if (CAVs(i).path==2 && CAVs(CAVs(i).Preceding_CAV).path==11) || (CAVs(i).path==11 && CAVs(CAVs(i).Preceding_CAV).path==2)
            s = CAVs(CAVs(i).Preceding_CAV).x - CAVs(i).x;
        elseif (CAVs(i).path==2 && CAVs(CAVs(i).Preceding_CAV).path==5) || (CAVs(i).path==5 && CAVs(CAVs(i).Preceding_CAV).path==2)
            s = CAVs(CAVs(i).Preceding_CAV).y - CAVs(i).y;
        elseif (CAVs(i).path==5 && CAVs(CAVs(i).Preceding_CAV).path==8) || (CAVs(i).path==8 && CAVs(CAVs(i).Preceding_CAV).path==5)
            s = CAVs(i).x - CAVs(CAVs(i).Preceding_CAV).x ;
        elseif (CAVs(i).path==8 && CAVs(CAVs(i).Preceding_CAV).path==11) || (CAVs(i).path==11 && CAVs(CAVs(i).Preceding_CAV).path==8)
            s = CAVs(i).y - CAVs(CAVs(i).Preceding_CAV).y ;
        end
    else

    s = abs (CAVs(i).p- CAVs(CAVs(i).Preceding_CAV).p);

    end

    Dv = CAVs(i).v- CAVs(CAVs(i).Preceding_CAV).v;

    s_star = s0 + CAVs(i).v*T + (CAVs(i).v*Dv)/(2*sqrt(a*b));

    u_preced = a*(1-(CAVs(i).v/v0)^delta - (s_star/s)^2 );

end


CAVs(i).u = min(u_preced,u_tr_light);

if CAVs(i).u < -5
    CAVs(i).u = -5;   % clamp to -5
elseif CAVs(i).u > 6
    CAVs(i).u = 6;    % clamp to +6
end



function x = desired_gap(v,Dv,b,s0,T,a)

x = s0 + v*T + v*Dv/2*sqrt(a*b);

end


