close all;clear;clc;

figure; 
fps=0.5;

x0=80;
y0=-50;
zero_deg=-100;

all_time=500;
time=0;

while time<all_time

    [angle,dmin]=OutputAngleandDmin([x0,y0],zero_deg,30,[10,20,30]);
    corners =drawRotatedRectangle([x0,y0], 16, 20,zero_deg);
    isInside = Detection(corners);
    isInside_final = isPointInPolygon_final([x0,y0],110,-40,100,-60,120,-60,120,-40);
    [Vx_out, Vy_out, Omega_out]  = fuzzy_control_system(angle,dmin);
    
    Vx_out=Vx_out/(10*fps);
    Vy_out=Vy_out/(10*fps);
    Omega_out=Omega_out/(fps);
    
    [dx_new, dy_new,zero_deg] = rotate_cartesian_with_custom_polar(x0, y0,Vy_out,Vx_out, zero_deg,Omega_out);
    
    x0=dx_new;
    y0=dy_new;
    
    time=time+1;
        
    if isInside==1 & ~isInside_final==1
        fprintf('安全 %f 秒\n',time/fps);
    elseif isInside==1 & isInside_final==1
        fprintf('抵達 %f 秒\n',time/fps);
        break;
    else
        fprintf('出局 %f 秒\n',time/fps);
        break;
    end

end


% 繪圖
hold on; axis equal

Draw_the_venue(34,50,132)