function [dmin, closestPoint, idx, prevPoint, nextPoint] = findDistanceToCurve(P, n)

    theta = linspace(pi/2, pi/2*3, 500);
    theta2 = linspace(pi/2*3,pi*2.5, 500);
    r1=34;
    r2=50;
    L=132;
    x11 = r1 * cos(theta)+r1;
    y11 = r2 * sin(theta);
    x21 = r1 * cos(theta2)+L+r1;
    y21 = r2 * sin(theta2);
    x = [x11,x21,x11(1)];
    y = [y11,y21,y11(1)];
    

    minDist = inf;
    closestPoint = [NaN, NaN];
    idx = -1;
    prevPoint = [NaN, NaN];
    nextPoint = [NaN, NaN];

    for i = 1:length(x)-1
        A = [x(i), y(i)];
        B = [x(i+1), y(i+1)];
        AB = B - A;

        M = [n(:), -AB(:)];
        b = (A - P)';

        if rank(M) < 2
            continue;
        end

        ts = M \ b;
        t = ts(1);
        s = ts(2);

        if s >= 0 && s <= 1
            intersectPt = P + t * n;
            dist = norm(intersectPt - P);

            if dist < minDist
                minDist = dist;
                closestPoint = intersectPt;

                if norm(intersectPt - A) < 1e-6 && i > 1 && i < length(x)
                    idx = i;
                    prevPoint = [x(i-1), y(i-1)];
                    nextPoint = [x(i+1), y(i+1)];
                elseif norm(intersectPt - B) < 1e-6 && i+1 > 1 && i+1 < length(x)
                    idx = i+1;
                    prevPoint = [x(i), y(i)];
                    nextPoint = [x(i+2), y(i+2)];
                else
                    idx = -1;
                    prevPoint = A;
                    nextPoint = B;
                end
            end
        end
    end
    inside = isPointInPolygon(P);
    if inside==0
        dmin = minDist;
    else
        dmin = -minDist;
    end
end
