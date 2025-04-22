
if CAVs(i).path == 1
    if CAVs(i).y >= -17.375 && CAVs(i).x > -17.375
        % TURNING REGION:
        k = 1 / radius;  % (As in your original code; adjust sign if needed.)
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        % Retrieve current state:
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        % Compute average velocity (accounting for acceleration over dt)
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        % Compute heading change (using average velocity)
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        % Use the closed–form integration for position update:
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            % Fallback for straight–line motion:
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        % Final velocity update:
        
    elseif CAVs(i).y >= -17.375 && CAVs(i).x <= -17.375
        % STRAIGHT MOVEMENT (moving primarily in -x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old - v_avg * dt;  % x decreases
        y_new = y_old;               % y unchanged
        theta_new = theta_old;
        
    else
        % STRAIGHT MOVEMENT (moving primarily in +y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;               % x unchanged
        y_new = y_old + v_avg * dt;    % y increases
        theta_new = theta_old;
    end

elseif CAVs(i).path == 2
    if CAVs(i).y >= -27.875 && CAVs(i).x < 27.875 && strcmp(CAVs(i).Turn, 'right')
        % TURNING REGION:
        k = -1 / radius;
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        
    elseif CAVs(i).y >= -27.875 && CAVs(i).x >= 27.875
        % STRAIGHT MOVEMENT (moving primarily in +x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old + v_avg * dt;
        y_new = y_old;
        theta_new = theta_old;
        
    else
        % STRAIGHT MOVEMENT (moving primarily in +y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;
        y_new = y_old + v_avg * dt;
        theta_new = theta_old;
    end

elseif CAVs(i).path == 4
    if CAVs(i).y >= -17.375 && CAVs(i).x <= 17.375
        % TURNING REGION:
        k = 1 / radius;
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        
    elseif CAVs(i).y <= -17.375 && CAVs(i).x <= 17.375
        % STRAIGHT MOVEMENT (moving primarily in -y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;
        y_new = y_old - v_avg * dt;
        theta_new = theta_old;
        
    else
        
        % STRAIGHT MOVEMENT (moving primarily in -x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old - v_avg * dt;
        y_new = y_old;
        theta_new = theta_old;
    end

elseif CAVs(i).path == 5
    if CAVs(i).y <= 27.875 && CAVs(i).x <= 27.875 && strcmp(CAVs(i).Turn, 'right')
        % TURNING REGION:
        k = -1 / radius;
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        
    elseif CAVs(i).y >= 27.875 && CAVs(i).x <= 27.875
        % STRAIGHT MOVEMENT (moving primarily in +y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;
        y_new = y_old + v_avg * dt;
        theta_new = theta_old;
        
    else
        % STRAIGHT MOVEMENT (moving primarily in -x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old - v_avg * dt;
        y_new = y_old;
        theta_new = theta_old;
    end

elseif CAVs(i).path == 7
    if CAVs(i).y <= 17.375 && CAVs(i).x <= 17.375
        % TURNING REGION:
        k = 1 / radius;
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        
    elseif CAVs(i).y <= 17.375 && CAVs(i).x >= 17.375
        % STRAIGHT MOVEMENT (moving primarily in +x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old + v_avg * dt;
        y_new = y_old;
        theta_new = theta_old;
        
    else
        % STRAIGHT MOVEMENT (moving primarily in -y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;
        y_new = y_old - v_avg * dt;
        theta_new = theta_old;
    end

elseif CAVs(i).path == 8
    if CAVs(i).y <= 27.875 && CAVs(i).x >= -27.875 && strcmp(CAVs(i).Turn, 'right')
        % TURNING REGION:
        k = -1 / radius;
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        
    elseif CAVs(i).y <= 27.875 && CAVs(i).x <= -27.875
        % STRAIGHT MOVEMENT (moving primarily in -x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old - v_avg * dt;
        y_new = y_old;
        theta_new = theta_old;
        
    else
        % STRAIGHT MOVEMENT (moving primarily in -y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;
        y_new = y_old - v_avg * dt;
        theta_new = theta_old;
    end

elseif CAVs(i).path == 10
    if CAVs(i).y <= 17.375 && CAVs(i).x > -17.375
        % TURNING REGION:
        k = 1 / radius;
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        
    elseif CAVs(i).y >= 17.375 && CAVs(i).x >= -17.375
        % STRAIGHT MOVEMENT (moving primarily in +y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;
        y_new = y_old + v_avg * dt;
        theta_new = theta_old;
        
    else
        % STRAIGHT MOVEMENT (moving primarily in +x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old + v_avg * dt;
        y_new = y_old;
        theta_new = theta_old;
    end

elseif CAVs(i).path == 11
    if CAVs(i).y >= -27.875 && CAVs(i).x >= -27.875 && strcmp(CAVs(i).Turn, 'right')
        % TURNING REGION:
        k = -1 / radius;
        delta = atan(k * L);
        omega = (1 / L) * tan(delta);
        
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        Delta_theta = v_avg * dt * omega;
        theta_new = theta_old + Delta_theta;
        
        if abs(omega) > 1e-12
            x_new = x_old + (1 / omega) * ( sin(theta_old + Delta_theta) - sin(theta_old) );
            y_new = y_old + (1 / omega) * ( -cos(theta_old + Delta_theta) + cos(theta_old) );
        else
            x_new = x_old + v_avg * dt * cos(theta_old);
            y_new = y_old + v_avg * dt * sin(theta_old);
        end
        
        
    elseif CAVs(i).y <= -27.875 && CAVs(i).x >= -27.875
        % STRAIGHT MOVEMENT (moving primarily in -y direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old;
        y_new = y_old - v_avg * dt;
        theta_new = theta_old;
        
    else
        % STRAIGHT MOVEMENT (moving primarily in +x direction)
        v_old = CAVs(i).v;
        theta_old = CAVs(i).theta;
        x_old = CAVs(i).x;
        y_old = CAVs(i).y;
        
        v_avg = v_old + 0.5 * dt * CAVs(i).u;
        x_new = x_old + v_avg * dt;
        y_new = y_old;
        theta_new = theta_old;
    end
end

