close all;clear;clc;

figure; 
fps=1;

initial_x0=80;
initial_y0=0;
initial_zero_deg=0;

all_time=500;
time=1;


x0=initial_x0;
y0=initial_y0-50;
zero_deg=initial_zero_deg-90;
[angle,dmin]=OutputAngleandDmin([x0,y0],zero_deg,30,[10,20,30]);
[angle_now,dmin_now]=OutputAngleandDmin([x0,y0],zero_deg,30,0);
Vx_out=0;
Vy_out=0;
Omega_out=0;
V=sqrt((Vx_out*Vx_out)+(Vy_out*Vy_out));

log(time,1)=(time-1)/fps;
log(time,2)=angle_now;
log(time,3)=dmin_now;
log(time,4)=angle(1);
log(time,5)=dmin(1);
log(time,6)=angle(2);
log(time,7)=dmin(2);
log(time,8)=angle(3);
log(time,9)=dmin(3);
log(time,10)=V;
log(time,11)=Vx_out;
log(time,12)=Vy_out;
log(time,13)=Omega_out;


while time<all_time

    time=time+1;
    

    %log(時間,即時角度差1,即時距離差1,
    % ,偵測角度差1,偵測距離差1,偵測角度差2,偵測距離差2,偵測角度差3,偵測距離差3
    % ,v,vx,vy,omega)
    [angle,dmin]=OutputAngleandDmin([x0,y0],zero_deg,30,[10,20,30]);
    [angle_now,dmin_now]=OutputAngleandDmin([x0,y0],zero_deg,30,0);
    corners =drawRotatedRectangle([x0,y0], 16, 20,zero_deg);
    isInside = Detection(corners);
    isInside_final = isPointInPolygon_final([x0,y0],110,-40,100,-60,120,-60,120,-40);
    [Vx_out, Vy_out, Omega_out]  = fuzzy_control_system(angle,dmin);
    



    V=sqrt((Vx_out*Vx_out)+(Vy_out*Vy_out));

    log(time,1)=(time-1)/fps;
    log(time,2)=angle_now;
    log(time,3)=dmin_now;
    log(time,4)=angle(1);
    log(time,5)=dmin(1);
    log(time,6)=angle(2);
    log(time,7)=dmin(2);
    log(time,8)=angle(3);
    log(time,9)=dmin(3);
    log(time,10)=V/10;
    log(time,11)=Vx_out/10;
    log(time,12)=Vy_out/10;
    log(time,13)=Omega_out;


    Vx_out=Vx_out/(10*fps);
    Vy_out=Vy_out/(10*fps);
    Omega_out=Omega_out/(fps);
    
    [dx_new, dy_new,zero_deg] = rotate_cartesian_with_custom_polar(x0, y0,Vy_out,Vx_out, zero_deg,Omega_out);
    
    x0=dx_new;
    y0=dy_new;
    


        
    if isInside==1 && ~isInside_final==1
        fprintf('%f 秒 安全\n',(time-1)/fps);
    elseif isInside==1 && isInside_final==1
        fprintf('%f 秒 抵達\n',(time-1)/fps);
        break;
    else
        fprintf('%f 秒 出局\n',(time-1)/fps);
        break;
    end

end


% 繪圖
hold on; axis equal

Draw_the_venue(34,50,132)

figure;
plot(log(:,1),log(:,10),"LineWidth",1.5)
hold on;
plot(log(:,1),log(:,11),"LineWidth",1.5)
title(sprintf("初始距離差:%.0fcm，初始角度差:%.0fdeg，每%.0f秒執行一次，即時速度對應時間",initial_y0,initial_zero_deg,1/fps))
xlabel("時間(s)")
ylabel("速度(cm/s)")
xlim([0 time-1]); 
legend('V',"Vx", 'FontSize', 14,'Location', 'eastoutside')
grid on;

figure;
plot(log(:,1),log(:,12),"LineWidth",1.5)
title(sprintf("初始距離差:%.0fcm，初始角度差:%.0fdeg，每%.0f秒執行一次，即時速度對應時間",initial_y0,initial_zero_deg,1/fps))
xlabel("時間(s)")
ylabel("速度(cm/s)")
xlim([0 time-1]); 
legend("Vy", 'FontSize', 14,'Location', 'eastoutside')
grid on;

figure;
plot(log(:,1),log(:,13),"LineWidth",1.5)
title(sprintf("初始距離差:%.0fcm，初始角度差:%.0fdeg，每%.0f秒執行一次，即時角速度對應時間",initial_y0,initial_zero_deg,1/fps))
xlabel("時間(s)")
ylabel("角速度(deg/s)")
xlim([0 time-1]); 
legend("Omega", 'FontSize', 14,'Location', 'eastoutside')
grid on;

figure;
plot(log(:,1),log(:,5),"LineWidth",1.5)
hold on;
plot(log(:,1),log(:,7),"LineWidth",1.5)
hold on;
plot(log(:,1),log(:,9),"LineWidth",1.5)
title(sprintf("初始距離差:%.0fcm，初始角度差:%.0fdeg，每%.0f秒執行一次，距離差對應時間",initial_y0,initial_zero_deg,1/fps))
xlabel("時間(s)")
ylabel("距離(mm)")
xlim([0 time-1]); 
legend('距離10cm','距離20cm','距離30cm', 'FontSize', 14,'Location', 'eastoutside')
grid on;

figure;
plot(log(:,1),log(:,6),"LineWidth",1.5)
hold on;
plot(log(:,1),log(:,8),"LineWidth",1.5)
hold on;
plot(log(:,1),log(:,10),"LineWidth",1.5)
title(sprintf("初始距離差:%.0fcm，初始角度差:%.0fdeg，每%.0f秒執行一次，角度差差對應時間",initial_y0,initial_zero_deg,1/fps))
xlabel("時間(s)")
ylabel("角度差(deg)")
xlim([0 time-1]); 
legend('距離10cm','距離20cm','距離30cm', 'FontSize', 14,'Location', 'eastoutside')
grid on;

figure;
plot(log(:,1),log(:,2),"LineWidth",1.5)
title(sprintf("初始距離差:%.0fcm，初始角度差:%.0fdeg，每%.0f秒執行一次，即時角度差對應時間",initial_y0,initial_zero_deg,1/fps))
xlabel("時間(s)")
ylabel("角度差(deg)")
xlim([0 time-1]); 
legend("角度差", 'FontSize', 14,'Location', 'eastoutside')
grid on;

figure;
plot(log(:,1),log(:,3),"LineWidth",1.5)
title(sprintf("初始距離差:%.0fcm，初始角度差:%.0fdeg，每%.0f秒執行一次，即時距離差對應時間",initial_y0,initial_zero_deg,1/fps))
xlabel("時間(s)")
ylabel("距離(mm)")
xlim([0 time-1]); 
legend("距離差", 'FontSize', 14,'Location', 'eastoutside')
grid on;

