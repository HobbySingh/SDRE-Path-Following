function [xt,yt,zt] = sphere_intersection(w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,p_x,p_y,p_z,L)

    x1 = w1_x;y1 = w1_y;z1 = w1_z;
    x2 = w2_x;y2 = w2_y;z2 = w2_z;
    x3 = p_x; y3 = p_y;z3 = p_z;
    r = L;

    a = (x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2;
    b = 2*((x2 - x1)*(x1 -x3) + (y2-y1)*(y1-y3) + (z2-z1)*(z1-z3));
    c = x3^2 + y3^2 + z3^2 + x1^2 + y1^2 + z1^2 - 2*(x3*x1 + y3*y1 + z3*z1) - r^2;

    u1 = (-b + sqrt(b^2 - 4*a*c))/(2*a);
    u2 = (-b - sqrt(b^2 - 4*a*c))/(2*a);

    vtp_x1 = x1 + u1*(x2-x1);
    vtp_y1 = y1 + u1*(y2-y1);
    vtp_z1 = z1 + u1*(z2-z1);

    vtp_x2 = x1 + u2*(x2-x1);
    vtp_y2 = y1 + u2*(y2-y1);
    vtp_z2 = z1 + u2*(z2-z1);

    %% choosing point in forward direction by calculating distance from
    % target waypoint
    dist1 = sqrt((vtp_x1-w2_x)^2 + (vtp_y1-w2_y)^2 + (vtp_z1-w2_z)^2);
    dist2 = sqrt((vtp_x2-w2_x)^2 + (vtp_y2-w2_y)^2 + (vtp_z2-w2_z)^2);
    
    if dist1 < dist2
        xt = vtp_x1;
        yt = vtp_y1;
        zt = vtp_z1;
    else
        xt = vtp_x2;
        yt = vtp_y2;
        zt = vtp_z2;
    end