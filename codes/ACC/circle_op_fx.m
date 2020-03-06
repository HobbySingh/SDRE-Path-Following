function status = circle_op_fx(t,y,flag,r,center_x,center_y,speedx,speedz,c,u,uz,dxy,dz)

persistent u_xy
persistent u_z
persistent d_xy
persistent d_z

switch flag
    case 'init'
        %fprintf("Inside init");
        u_xy = u;
        u_z = uz;
        d_xy = dxy;
        d_z = dz;
    case ''
%         fprintf("Inside Middle\n");

        if(0 < t(end) && t(end) < 50)
            speedz = 1;
        end
        if(50 < t(end) && t(end) < 125)
            speedz = 0;
        end
        if(125 < t(end) && t(end) < 175)
            speedz = -1;
        end
        v = 25;
        % c = 0.2;
        si_w = 3.14; % wind direction
        vw = c*v; 
        v_wx = c*v*cos(si_w);
        v_wy = c*v*sin(si_w);
        si = y(3);
        x_dot = v*cos(si) + v_wx;
        y_dot = v*sin(si) + v_wy;

        Rmin = 75;

        igain_xy = 0.0001;
        igain_z = 0.001;
        i_xy = y(9);
        i_z = y(10);

        % r = 100;
        lambda = 0.1;
        uav_x = y(1); uav_y = y(2); uav_z = y(6);
        center_x_moving = y(5);

        curr_r = sqrt((uav_x - center_x_moving)^2 + (uav_y - center_y)^2);

        y_ = uav_y - center_y;
        x_ = uav_x - center_x_moving;
        y_by_x = y_/x_;
        y_by_x_dot = (y_dot*x_ - y_*(x_dot - speedx))/(x_)^2; 
        theta = atan2((y_),(x_));

        theta_dot = (y_by_x_dot)*(1/(1 + (y_by_x)^2));

        x_t = center_x_moving + r*cos(theta + lambda);
        y_t = center_y + r*sin(theta + lambda);
        % si_p = atan2((y_t - uav_y),(x_t - uav_x));
        si_p = theta + 1.57;
        si_p_dot = theta_dot;

        db = 4;
        q2 = 1;

        d1 = y(4);
        d = r - curr_r;
        % q1 = sqrt(abs(db/(db - d)));
        k = 0.25;
        q1 = sqrt(exp(k*(abs(d))));
        si = y(3);
        vd = v*sin(si - si_p) + vw*sin(si_w - si_p); % d_dot
        u = -(q1*d + sqrt(2*q1 + q2^2)*vd + igain_xy*i_xy);
        if(abs(u) > (v^2)/Rmin)
            if (u > 0)
                u = (v^2)/Rmin;
            else
                u = -(v^2)/Rmin;
            end
        end
        si_dot = u/v;
        si_dot = (u + v*si_p_dot)/v;

        z_t = y(9);
        v1 = [(x_t - uav_x) (y_t - uav_y) (z_t - uav_z)];
        v2 = [0 0 1];
        dz1 = y(8);
        dz = (uav_z - z_t);

        k = 0.01;
        q1_z = sqrt(exp(k*(abs(dz))));
        si_z = y(7);
        si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2)))); 
        vd_z = v*sin(si_z - si_z_p);
        uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z +  + igain_z*i_z);
        Rminz = 75;
        if(abs(uz) > (v^2)/Rminz)
            if (uz > 0)
                uz = (v^2)/Rminz;
            else
                uz = -(v^2)/Rminz;
            end
        end
        si_z_dot = uz/v;v = 25;
        % c = 0.2;
        si_w = 3.14; % wind direction
        vw = c*v; 
        v_wx = c*v*cos(si_w);
        v_wy = c*v*sin(si_w);
        si = y(3);
        x_dot = v*cos(si) + v_wx;
        y_dot = v*sin(si) + v_wy;

        Rmin = 75;

        igain_xy = 0.0001;
        igain_z = 0.0001;
        i_xy = y(9);
        i_z = y(10);

        % r = 100;
        lambda = 0.1;
        uav_x = y(1); uav_y = y(2); uav_z = y(6);
        center_x_moving = y(5);

        curr_r = sqrt((uav_x - center_x_moving)^2 + (uav_y - center_y)^2);

        y_ = uav_y - center_y;
        x_ = uav_x - center_x_moving;
        y_by_x = y_/x_;
        y_by_x_dot = (y_dot*x_ - y_*(x_dot - speedx))/(x_)^2; 
        theta = atan2((y_),(x_));

        theta_dot = (y_by_x_dot)*(1/(1 + (y_by_x)^2));

        x_t = center_x_moving + r*cos(theta + lambda);
        y_t = center_y + r*sin(theta + lambda);
        % si_p = atan2((y_t - uav_y),(x_t - uav_x));
        si_p = theta + 1.57;
        si_p_dot = theta_dot;

        db = 4;
        q2 = 1;

        d = r - curr_r;
        % q1 = sqrt(abs(db/(db - d)));
        k = 0.01;
        q1 = sqrt(exp(k*(abs(d))));
        si = y(3);
        vd = v*sin(si - si_p) + vw*sin(si_w - si_p); % d_dot
        u = -(q1*d + sqrt(2*q1 + q2^2)*vd + igain_xy*i_xy);
        if(abs(u) > (v^2)/Rmin)
            if (u > 0)
                u = (v^2)/Rmin;
            else
                u = -(v^2)/Rmin;
            end
        end

        % z_t = 2*(theta+lambda);
        z_t = y(9);
        v1 = [(x_t - uav_x) (y_t - uav_y) (z_t - uav_z)];
        v2 = [0 0 1];
        dz = (uav_z - z_t);
        % q1_z = sqrt(abs(db/(db - dz)));
        k = 0.01;
        q1_z = sqrt(exp(k*(abs(dz))));
        si_z = y(7);
        si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2)))); 
        vd_z = v*sin(si_z - si_z_p);
        uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z +  + igain_z*i_z);
        Rminz = 75;
        if(abs(uz) > (v^2)/Rminz)
            if (uz > 0)
                uz = (v^2)/Rminz;
            else
                uz = -(v^2)/Rminz;
            end
        end
        
        u_xy = [u_xy;u];
        d_xy = [d_xy;d];
        
        u_z = [u_z;uz];
        d_z = [d_z;dz];
        
    case 'done'
        
        fprintf("Inside done\n");
        
        assignin('base','u_xy',u_xy);
        assignin('base','d_xy',d_xy);
        assignin('base','u_z',u_z);
        assignin('base','d_z',d_z);        
        status = 0;
end

status = 0;