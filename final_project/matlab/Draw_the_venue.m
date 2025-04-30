function Draw_the_venue(r1,r2,L)

    theta = linspace(pi/2, pi/2*3, 500);
    theta2 = linspace(pi/2*3,pi*2.5, 500);
    
    x11 = r1 * cos(theta)+r1;
    y11 = r2 * sin(theta);
    x21 = r1 * cos(theta2)+L+r1;
    y21 = r2 * sin(theta2);
    
    x1 = [x11,x21,x11(1)];
    y1 = [y11,y21,y11(1)];
    
    
    x12 = (r1+15) * cos(theta)+r1;
    y12 = (r2+15) * sin(theta);
    x22 = (r1+15) * cos(theta2)+L+r1;
    y22 = (r2+15) * sin(theta2);
    
    x2 = [x12,x22,x12(1)];
    y2 = [y12,y22,y12(1)];
    
    x13 = (r1-15) * cos(theta)+r1;
    y13 = (r2-15) * sin(theta);
    x23 = (r1-15) * cos(theta2)+L+r1;
    y23 = (r2-15) * sin(theta2);
    
    x3 = [x13,x23,x13(1)];
    y3 = [y13,y23,y13(1)];
    
    x4 =linspace(100,100, 500);
    y4 =linspace(-35,-65, 500);
    
    
    
    % 畫圖
    plot(x1, y1, 'b-', 'LineWidth', 2);
    hold on
    plot(x2, y2, 'r-', 'LineWidth', 2);
    hold on
    plot(x3, y3, 'r-', 'LineWidth', 2);
    hold on
    plot(x4, y4, 'r-', 'LineWidth', 2);
    axis equal;
    grid on;
    title('競賽場地');
    xlabel('X 軸（公分）');
    ylabel('Y 軸（公分）');

end
