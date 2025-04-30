function [n1] = computeNormalAtPoint(x, y, P)



    distances = hypot(x - P(1), y - P(2));
    [~, i] = min(distances);

    if i == 1
        tangent = [x(i+1) - x(i), y(i+1) - y(i)];
    elseif i == length(x)
        tangent = [x(i) - x(i-1), y(i) - y(i-1)];
    else
        tangent = [x(i+1) - x(i-1), y(i+1) - y(i-1)];
    end


    tangent = tangent / norm(tangent);

 
    n1 = [ -tangent(2),  tangent(1) ]; 
end
