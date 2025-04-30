function isInside = isPointInPolygon(point)

    theta = linspace(pi/2, pi/2*3, 500);
    theta2 = linspace(pi/2*3, pi*2.5, 500);
    r1 = 34;
    r2 = 50;
    L = 132;
    x11 = r1 * cos(theta) + r1;
    y11 = r2 * sin(theta);
    x21 = r1 * cos(theta2) + L + r1;
    y21 = r2 * sin(theta2);
    x1= [x11, x21, x11(1)];
    y1 = [y11, y21, y11(1)];

    px = point(1);
    py = point(2);

    isInside = inpolygon(px, py, x1, y1);
end
