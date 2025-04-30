function [dx_new, dy_new,zero_deg] = rotate_cartesian_with_custom_polar(x0, y0, dx, dy, zero_deg, delta_theta)

    r = hypot(dx, dy); 
    theta = rad2deg(atan2(dy, dx)); 

    theta_adjusted = theta - zero_deg;

    theta_rad = deg2rad(theta_adjusted);

    dx_new = r * cos(theta_rad) + x0;
    dy_new = r * sin(theta_rad) + y0;
    zero_deg=zero_deg+delta_theta;
end
