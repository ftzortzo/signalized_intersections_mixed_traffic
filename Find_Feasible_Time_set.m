function [tif_min tif_max] = Find_Feasible_Time_set(v_start,t0,S)

global umin umax vmin vmax

tf_u = 6 * S / (3*v_start + sqrt(3)*sqrt(3*v_start^2 + 4*S*umax));
tf_v = 3 * S / (v_start + 2 * vmax);
tif=t0+max(tf_u,tf_v);
tif_min=tif;


if (9*v_start^2 + 12*S*umin > 0)
    tf_u = (sqrt(9*v_start^2 + 12*S*umin) - 3*v_start)/(2*umin);
else
    tf_u = -1;
end
tf_v = 3 * S / (v_start + 2 * vmin);
tif_max = t0 + max(tf_v, tf_u);

