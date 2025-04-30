function angle = computeRelativeAngle(xpoint, ypoint, prevPoint, nextPoint)

    v1 = [xpoint(end),ypoint(end)] - [xpoint(1),ypoint(1)];
    v1 = v1 / norm(v1);

    v2 = nextPoint - prevPoint;
    v2 = v2 / norm(v2);


    raw_angle = atan2d(det([v1; v2]), dot(v1, v2)); 

    if mod(360-raw_angle, 360)<180
        angle =mod(360-raw_angle, 360);
    else
        angle =mod(360-raw_angle, 360)-360;
    end
end
