function cornersRotated = drawRotatedRectangle(center, width, height, angleDeg)

    theta = deg2rad(-angleDeg);

    corners = [ -width/2, -height/2;
                 width/2, -height/2;
                 width/2,  height/2;
                -width/2,  height/2 ];

    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    cornersRotated = (R * corners')';
    cornersRotated = cornersRotated + center;

    plot([cornersRotated(:,1); cornersRotated(1,1)], ...
         [cornersRotated(:,2); cornersRotated(1,2)], ...
         'Color', 'k', 'LineWidth', 2);
    axis equal;
end

