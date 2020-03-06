function status = sl_op_fx(t,y,flag,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,c,u,uz,dxy,dz)

persistent u_xy
persistent u_z
persistent d_xy
persistent d_z

global u_xy_arr;
global u_z_arr;
global d_xy_arr;
global d_z_arr;

switch flag
    case 'init'
        %fprintf("Inside init");
        u_xy = u;
        u_z = uz;
        d_xy = dxy;
        d_z = dz;
    case ''
        v = 25;
        si_w = 3.14; % wind direction
        vw = c*v; 
        Rmin = 75;

        igain_xy = 0.0001;
        igain_z = 0.1;
        i_xy = y(8);
        i_z = y(9);

        si_z = y(6);

        %% XY Controller 

        % Computing the position error
        % Distance of point to line (UAV Position - Desired path)

        pt = [y(1) y(2) 0]; % UAV Positon vector
        a1 = [w1_x w1_y 0]; % Waypoint 1 Vector
        a2 = [w2_x w2_y 0]; % Waypoint 2 Vector

        tmp = (y(1) - w1_x)*(w2_y - w1_y) - (y(2) - w1_y)*(w2_x - w1_x);
        % To check whether the point is left or right of the desired path
        if(tmp < 0)
            dxy = point_to_line(pt,a1,a2);
        else
            dxy = -point_to_line(pt,a1,a2);
        end    

        % LQR Formualtion

        q2 = 2.1;
        % -------------- Exponential Gain -------------
        k = 0.0001;
        q1 = sqrt(exp(k*(abs(dxy))));

        si = y(3);

        si_p = atan2((w2_y - w1_y),(w2_x - w1_x)); % si desired

        v_x = v*cos(si) + vw*cos(si_w);
        v_y = v*sin(si) + vw*sin(si_w);

        course_angle = atan2(v_y,v_x);
        si_p = atan2((w2_y - w1_y),(w2_x - w1_x)); % si desired
        % vd = v*sin(si - si_p) +  vw*sin(si_w - si_p); % d_dot
        vd = v*sin(course_angle - si_p) +  vw*sin(si_w - si_p); % d_dot
        
%         vd = v*sin(si - si_p) +  vw*sin(si_w - si_p); % d_dot

        u = -(q1*dxy + sqrt(2*q1 + q2^2)*vd + igain_xy*i_xy);

        % Constraining the control input
        if(abs(u) > (v^2)/Rmin)
            if (u > 0)
                u = (v^2)/Rmin;
            else
                u = -(v^2)/Rmin;
            end
        end

        %% Z Controller

        % Computing position error in z 

        pt = [y(1) y(2) y(5)]; % UAV Vector 
        a1 = [w1_x w1_y w1_z]; % Waypoint 1 Vector
        a2 = [w2_x w2_y w2_z]; % Waypoint 2 Vector

        if(w1_z < w2_z)
            line_vect = (a2 - a1)/norm(a2 - a1);
            pt_vect = (pt - a1)/norm(pt - a1);
        else
            line_vect = (a1 - a2)/norm(a1 - a2);
            pt_vect = (pt - a2)/norm(pt - a2);
        end

        % To check whether the point is above or below of the desired path
        % Taking zcopoinent of resultant vectors

        d = point_to_line(pt,a1,a2); % Error in Z

        dz = sqrt(abs(d^2 - dxy^2));
        if(pt_vect(3) - line_vect(3) < 0)
            dz = -dz; % Error in Z
        else
            dz = dz;
        end    

        % LQR Formulation
        % ------------ Exponential Gain -----------
        k = 0.01;
        q1_z = sqrt(exp(k*(abs(dz))));

        q2 = 1;

        v1 = [(w2_x - w1_x) (w2_y - w1_y) (w2_z - w1_z)]; % direction vector b/w two waypoints
        v2 = [0 0 1]; % direction vector normal to xy plane

        si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2))));
        vd_z = v*sin(si_z - si_z_p);
        uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z + igain_z*i_z);

        Rminz = 75;
        if(abs(uz) > (v^2)/Rminz)
            if (uz > 0)
                uz = (v^2)/Rminz;
            else
                uz = -(v^2)/Rminz;
            end
        end
        
        u_xy = [u_xy;u];
        d_xy = [d_xy;dxy];
        
        u_z = [u_z;uz];
        d_z = [d_z;dz];
        
    case 'done'
        
        %fprintf("Inside done");
        u_xy_arr = [u_xy_arr;u_xy];
        d_xy_arr = [d_xy_arr;d_xy];
        u_z_arr = [u_z_arr;u_z];
        d_z_arr = [d_z_arr;d_z];
        
        assignin('base','u_xy',u_xy);
        assignin('base','d_xy',d_xy);
        assignin('base','u_z',u_z);
        assignin('base','d_z',d_z);        
        
end

status = 0;