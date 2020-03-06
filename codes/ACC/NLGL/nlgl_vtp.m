function [xt,yt,zt] = nlgl_vtp(uav_x,uav_y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,L)

    %% equation of line to be followed
    slope = ((w2_y - w1_y)/(w2_x - w1_x));
    if (slope == inf) || (slope == -inf)
        intercept = w1_x;
    else
        intercept = slope*-w1_x + w1_y;
    end
    
    %% circle with center as uav and line as path to be followed
    % finding intersection points
    [x,y] = linecirc(slope,intercept,uav_x,uav_y,L);
    pt1 = [(x(1)),(y(1))];
    pt2 = [(x(2)),(y(2))];
    
    %% choosing point in forward direction by calculating distance from
    % target waypoint
    dist1 = sqrt((pt1(1)-w2_x)^2 + (pt1(2)-w2_y)^2);
    dist2 = sqrt((pt2(1)-w2_x)^2 + (pt2(2)-w2_y)^2);

    if dist1 < dist2
        xt = pt1(1);
        yt = pt1(2);
    else
        xt = pt2(1);
        yt = pt2(2);
    end
    
    dist = sqrt((xt - w2_x)^2 + (yt - w2_y)^2);
    if dist < L
        xt = w2_x;
        yt = w2_y;
    end
    
    %% solving for z given x,y coordinates
    % direction vector
    v = [w2_x - w1_x, w2_y - w1_y, w2_z - w1_z];
    if(v(2) == 0)
        zt = ((xt - w1_x)/v(1))*(v(3)) + w1_z;
    else
        zt = ((yt - w1_y)/v(2))*(v(3)) + w1_z;
    end
    
    
    