

for i = 1:number_of_CAVs


    distance_from_coordinator = sqrt(CAVs(i).x^2 + CAVs(i).y^2 );

    if ( max(abs(CAVs(i).x),abs(CAVs(i).y))>300 ) && CAVs(i).Passed_control_zone == 0

        % Update vehicle positions and orientations based on the path
        if CAVs(i).path == 1
            xnew = CAVs(i).x;
            ynew = CAVs(i).y + dt * CAVs(i).v;
            thetanew = CAVs(i).theta;
        elseif CAVs(i).path == 2

            if CAVs(i).p>0
                xnew = CAVs(i).x;
                ynew = CAVs(i).y + dt * CAVs(i).v;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).y<0
                xnew = CAVs(i).x + dt * CAVs(i).v;
                ynew = CAVs(i).y;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).path == 2 && CAVs(i).y>0
                xnew = CAVs(i).x ;
                ynew = CAVs(i).y + dt * CAVs(i).v;
                thetanew = CAVs(i).theta;
            end

        elseif CAVs(i).path == 4
            xnew = CAVs(i).x - dt * CAVs(i).v;
            ynew = CAVs(i).y ;
            thetanew = CAVs(i).theta;
        elseif CAVs(i).path == 5

            if CAVs(i).p>0
                xnew = CAVs(i).x - dt * CAVs(i).v;
                ynew = CAVs(i).y ;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).x>0
                xnew = CAVs(i).x ;
                ynew = CAVs(i).y + dt * CAVs(i).v;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).x<0
                xnew = CAVs(i).x - dt * CAVs(i).v ;
                ynew = CAVs(i).y;
                thetanew = CAVs(i).theta;
            end

        elseif CAVs(i).path == 7
            xnew = CAVs(i).x;
            ynew = CAVs(i).y - dt * CAVs(i).v;
            thetanew = CAVs(i).theta;
        elseif CAVs(i).path == 8
            if CAVs(i).p>0
                xnew = CAVs(i).x;
                ynew = CAVs(i).y - dt * CAVs(i).v;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).y>0
                xnew = CAVs(i).x-dt * CAVs(i).v;
                ynew = CAVs(i).y;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).y<0
                xnew = CAVs(i).x;
                ynew = CAVs(i).y - dt * CAVs(i).v;
                thetanew = CAVs(i).theta;
            end

        elseif CAVs(i).path == 10
            xnew = CAVs(i).x + dt * CAVs(i).v;
            ynew = CAVs(i).y ;
            thetanew = CAVs(i).theta;
        elseif CAVs(i).path == 11
            if CAVs(i).p>0
                xnew = CAVs(i).x + dt * CAVs(i).v;
                ynew = CAVs(i).y ;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).x > 0
                xnew = CAVs(i).x + dt * CAVs(i).v;
                ynew = CAVs(i).y ;
                thetanew = CAVs(i).theta;
            elseif CAVs(i).x < 0
                xnew = CAVs(i).x ;
                ynew = CAVs(i).y - dt * CAVs(i).v ;
                thetanew = CAVs(i).theta;
            end
        end
        CAVs(i).direction = [xnew-CAVs(i).x, ynew-CAVs(i).y];
        CAVs(i).direction = CAVs(i).direction/norm(CAVs(i).direction);
        CAVs(i).x = xnew;
        CAVs(i).y = ynew;
        CAVs(i).theta = thetanew;
        CAVs(i).p = CAVs(i).p - dt*CAVs(i).v;

    else

        Find_preceding_vehicle

        % pause(10)
        if CAVs(i).Type=='HDV' 
            HDVs_prediction
            CAVs(i).Passed_control_zone=1;
            IDM_model

            
                [~, idx] = min(abs(t - CAVs(i).t));
                abs((300-CAVs(i).p)-CAVs(i).p1(idx));
                if abs((300-CAVs(i).p)-CAVs(i).p1(idx))>1
                     CAVs(i).prediction=-1;
                end
        else

        if CAVs(i).Passed_control_zone == 0

            CAVs(i).Entrance_time=t;
            CAVs(i).Exit=-1;
            Find_green_times    
            v_start = CAVs(i).v;
            x = CAVs(i).x;
            y = CAVs(i).y;
            [phis, CAVs] =find_reference_path(['Path', num2str(CAVs(i).path)], v_start, t,x,y,CAVs,i);
            CAVs(i).phis = phis;
            CAVs(i).Passed_control_zone=1;
            Find_preceding_vehicle

        end


        if traffic_light_change_flag==1            
                Find_green_times ;               
        end

 
        if (300-CAVs(i).p) > 262 && CAVs(i).Passed_traffic_light==0
            
            Atomic_Replanning_after_light;
            CAVs(i).cant_cross_traffic_light=0;
        end
       

        if CAVs(i).cant_cross_traffic_light==1 

            if CAVs(i).Preceding_CAV>-1

                if CAVs(CAVs(i).Preceding_CAV).cant_cross_traffic_light==0
                    
                     Atomic_Replanning_before_light

                end
            else
                Atomic_Replanning_before_light
            end

        end


        u_ref = 6*CAVs(i).phis(1)*t + 2*CAVs(i).phis(2);

        % Next we add a certificate to guarantee that our vehicle has a
        % robust safety framework and cannot crash with the preceding
        % vehicle under whatever conditions
        if CAVs(i).Preceding_CAV>0  

            rear_constraint = (CAVs(CAVs(i).Preceding_CAV).v - CAVs(i).v + CAVs(i).p - CAVs(CAVs(i).Preceding_CAV).p  - gamma - reaction_time*CAVs(i).v)/reaction_time;
        else
            rear_constraint =1000;
        end
        
        
        CAVs(i).u = min(u_ref,rear_constraint);

        end


        % Update vehicle positions and orientations based on the path
       
        update_position_in_control_zone

        % Update CAV state
        CAVs(i).direction = [x_new-CAVs(i).x, y_new-CAVs(i).y];
        CAVs(i).direction = CAVs(i).direction/norm(CAVs(i).direction);
        CAVs(i).x = x_new;
        CAVs(i).y = y_new;
        CAVs(i).theta = theta_new;
        CAVs(i).p = CAVs(i).p - dt*CAVs(i).v - (1/2)*(dt^2*CAVs(i).u);
        CAVs(i).v = CAVs(i).v + dt*CAVs(i).u;

        if CAVs(i).v<0
            CAVs(i).v=0;
        end
        %- (1/2)*(dt^2)*CAVs(i).u;
    end

    % Define the vertices of the rectangle (vehicle)
    vertices = [-rect_length/2, -rect_width/2;
        rect_length/2, -rect_width/2;
        rect_length/2,  rect_width/2;
        -rect_length/2,  rect_width/2]';

    % Rotation matrix based on the updated theta
    R = [cos(CAVs(i).theta), -sin(CAVs(i).theta);
        sin(CAVs(i).theta),  cos(CAVs(i).theta)];

    % Rotate and translate the vertices to new position
    rotated_vertices = R * vertices;
    rotated_vertices(1, :) = rotated_vertices(1, :) + CAVs(i).x;
    rotated_vertices(2, :) = rotated_vertices(2, :) + CAVs(i).y;


    % Plot the vehicle as a rotated rectangle
    if CAVs(i).Type == 'CAV'
        fill(rotated_vertices(1, :), rotated_vertices(2, :), 'r'); % Plot the vehicle
    else
        fill(rotated_vertices(1, :), rotated_vertices(2, :), 'b'); % Plot the vehicle
    end
    axis([-320, 320, -320, 320]);

CAVs(i).P(plot_counter-1) = 300-CAVs(i).p;    

end
