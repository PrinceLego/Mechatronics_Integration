function isInside = Detection(polygon)

    theta  = linspace(pi/2, pi*1.5, 500);
    theta2 = linspace(pi*1.5, pi*2.5, 500);
    r1 = 34;
    r2 = 50;
    L  = 132;

    x12 = (r1+15) * cos(theta)  + r1;
    y12 = (r2+15) * sin(theta);
    x22 = (r1+15) * cos(theta2) + L + r1;
    y22 = (r2+15) * sin(theta2);
    x_outer = [x12, x22, x12(1)];
    y_outer = [y12, y22, y12(1)];

    x13 = (r1-15) * cos(theta)  + r1;
    y13 = (r2-15) * sin(theta);
    x23 = (r1-15) * cos(theta2) + L + r1;
    y23 = (r2-15) * sin(theta2);
    x_inner = [x13, x23, x13(1)];
    y_inner = [y13, y23, y13(1)];

    x = polygon(:,1);
    y = polygon(:,2);

    isInsideOuter = inpolygon(x, y, x_outer, y_outer);
    isOutsideInner = ~inpolygon(x, y, x_inner, y_inner);

    isInside = all(isInsideOuter & isOutsideInner);
end
