function [targetPoints,x111,y111] = pointAlongAngle(P, theta, L, targetDists)

    theta_adjusted = mod(90 - theta, 360);  

    thetaRad = deg2rad(theta_adjusted);

    dir = [cos(thetaRad), sin(thetaRad)];

    lineEnd = P + L * dir;

    targetPoints = P + targetDists(:) * dir;
    x111 = linspace(P(1),lineEnd(1), 500);
    y111 = linspace(P(2),lineEnd(2), 500);
end
