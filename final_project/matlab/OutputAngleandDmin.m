function [angle,dmin]=OutputAngleandDmin(O,a,LL,in)

    num_inputs = length(in);
    dmin = zeros(1, num_inputs);
    angle = zeros(1, num_inputs);

    for i=1:num_inputs
        [targetPoints,xpoint,ypoint] = pointAlongAngle(O,a,LL,in);
        P1 = [targetPoints(i,1), targetPoints(i,2)];
        [n]= computeNormalAtPoint(xpoint,ypoint, P1);
        [dmin(i), closestPoint, idx_min, prevPoint, nextPoint] = findDistanceToCurve(P1, n);
        angle(i) = computeRelativeAngle(xpoint, ypoint, nextPoint, prevPoint);
            %plot(xpoint,ypoint, 'g-', 'LineWidth', 2);
    hold on
    %plot(P1(1), P1(2), 'ro', 'MarkerSize', 8, 'DisplayName','起始點')
    %plot(closestPoint(1), closestPoint(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName','最近交d點')
    %plot(P1(1), P1(2), 'ro', 'DisplayName','起點')
    %plot(xpoint(1), ypoint(2), 'ro', 'DisplayName','起點')
    end



end