% 濾波分析主程式
clear; clc;
opts = detectImportOptions('rssi3.txt', 'Delimiter', ',');
opts = setvartype(opts, {'Var2','Var3','Var4'}, 'double'); 
opts = setvaropts(opts, {'Var2','Var3','Var4'}, 'TreatAsMissing', 'None');

data1 = readtable('rssi3.txt', opts);



% === 載入資料 ===
rssi_raw1 = data1{:, 2};   % 假設每列是一個 RSSI 數值
rssi_raw2 = data1{:, 3};   % 假設每列是一個 RSSI 數值
rssi_raw3 = data1{:, 4};   % 假設每列是一個 RSSI 數值
time_str = data1{:, 1};


% === 濾波參數 ===
alpha = 0.2;                   % EMA 係數
window_size = 5;              % 移動平均與中位數窗口大小
max_delta = 3;                % Clipping 最大跳動限制
Q = 1e-3; R = 1;              % Kalman 濾波器參數

% === 初始化結果 ===
N = length(rssi_raw1);

kalman_result = zeros(N, 1);



% Kalman 狀態
x1 = rssi_raw1(1);   % 初始估計值
x2 = rssi_raw2(1);   % 初始估計值
x3 = rssi_raw3(1);   % 初始估計值
P = 1;             % 誤差協方差

% === 濾波迴圈 ===
for t = 2:N
    new_val3 = rssi_raw3(t);

    % === Kalman ===
    % 預測步驟
    P = P + Q;

    % 更新步驟
    K = P / (P + R);
    x3 = x3 + K * (new_val3 - x3);
    P = (1 - K) * P;

    kalman_result3(t) = x3;
end

for t = 2:N
    new_val1 = rssi_raw1(t);

    % === Kalman ===
    % 預測步驟
    P = P + Q;

    % 更新步驟
    K = P / (P + R);
    x1 = x1 + K * (new_val1 - x1);
    P = (1 - K) * P;

    kalman_result1(t) = x1;
end

for t = 2:N
    new_val2 = rssi_raw2(t);

    % === Kalman ===
    % 預測步驟
    P = P + Q;

    % 更新步驟
    K = P / (P + R);
    x2 = x2 + K * (new_val2 - x2);
    P = (1 - K) * P;

    kalman_result2(t) = x2;
end


% === 畫圖 ===
time_dt = datetime(time_str, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
time_dt.Format = 'mm:ss'; 
% 繪圖時用時間軸作為 x
figure;

subplot(1,2,1);
% 原始
plot(time_dt(3:end), rssi_raw1(3:end), 'g');hold on
plot(time_dt(3:end), rssi_raw2(3:end), 'b');hold on
plot(time_dt(3:end), rssi_raw3(3:end), 'r');
ylabel('dBm');
title('原始 RSSI');
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
ylim([-90, -45]);
xlabel('分:秒');
grid on;

% Kalman
subplot(1,2,2);
plot(time_dt(3:end), kalman_result1(3:end), 'g');hold on
plot(time_dt(3:end), kalman_result2(3:end), 'b');hold on
plot(time_dt(3:end), kalman_result3(3:end), 'r');hold on
ylabel('dBm');
title('卡曼濾波 (Kalman)');
ylim([-90, -45]);
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
xlabel('分:秒');
grid on;