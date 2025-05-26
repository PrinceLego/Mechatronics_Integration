% 濾波分析主程式
clear; clc;
opts = detectImportOptions('rssi_log.txt', 'Delimiter', ',');
opts = setvartype(opts, {'Var2','Var3','Var4'}, 'double'); 
opts = setvaropts(opts, {'Var2','Var3','Var4'}, 'TreatAsMissing', 'None');

data = readtable('rssi_log.txt', opts);



% === 載入資料 ===
rssi_raw = data{:, 2};   % 假設每列是一個 RSSI 數值
time_str = data{:, 1};

% === 濾波參數 ===
alpha = 0.2;                   % EMA 係數
window_size = 5;              % 移動平均與中位數窗口大小
max_delta = 3;                % Clipping 最大跳動限制
Q = 1e-3; R = 1;              % Kalman 濾波器參數

% === 初始化結果 ===
N = length(rssi_raw);
ema_result = zeros(N, 1);
sma_result = zeros(N, 1);
median_result = zeros(N, 1);
clip_result = zeros(N, 1);
kalman_result = zeros(N, 1);

% === 初始化狀態 ===
ema_result(1) = rssi_raw(1);
sma_window = rssi_raw(1);
median_window = rssi_raw(1);
clip_result(1) = rssi_raw(1);

% Kalman 狀態
x = rssi_raw(1);   % 初始估計值
P = 1;             % 誤差協方差

% === 濾波迴圈 ===
for t = 2:N
    new_val = rssi_raw(t);

    % === EMA ===
    ema_result(t) = alpha * new_val + (1 - alpha) * ema_result(t-1);

    % === SMA ===
    if t <= window_size
        sma_result(t) = mean(rssi_raw(1:t));
    else
        sma_result(t) = mean(rssi_raw(t-window_size+1:t));
    end

    % === Median ===
    if t <= window_size
        median_result(t) = median(rssi_raw(1:t));
    else
        median_result(t) = median(rssi_raw(t-window_size+1:t));
    end

    % === Clipping ===
    delta = new_val - clip_result(t-1);
    if abs(delta) > max_delta
        clip_result(t) = clip_result(t-1) + sign(delta) * max_delta;
    else
        clip_result(t) = new_val;
    end

    % === Kalman ===
    % 預測步驟
    P = P + Q;

    % 更新步驟
    K = P / (P + R);
    x = x + K * (new_val - x);
    P = (1 - K) * P;

    kalman_result(t) = x;
end

% === 畫圖 ===
time_dt = datetime(time_str, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
time_dt.Format = 'mm:ss'; 
% 繪圖時用時間軸作為 x
figure;

% 原始
subplot(2,3,1);
plot(time_dt, rssi_raw, 'k');
ylabel('dBm');
title('原始 RSSI');
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
ylim([-90, -45]);
xlabel('分:秒');
grid on;

% EMA
subplot(2,3,2);
plot(time_dt(3:end), ema_result(3:end), 'r');
hold on
plot(time_dt(3:end), rssi_raw(3:end), 'k');
ylabel('dBm');
title('指數移動平均 (EMA)');
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
ylim([-90, -45]);
xlabel('分:秒');
grid on;

% SMA
subplot(2,3,3);
plot(time_dt(3:end), sma_result(3:end), 'g');hold on
plot(time_dt(3:end), rssi_raw(3:end), 'k');
ylabel('dBm');
title('移動平均 (SMA)');
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
ylim([-90, -45]);
xlabel('分:秒');
grid on;

% Median
subplot(2,3,4);
plot(time_dt(3:end), median_result(3:end), 'c');hold on
plot(time_dt(3:end), rssi_raw(3:end), 'k');
ylabel('dBm');
title('中位數濾波 (Median)');
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
ylim([-90, -45]);xlabel('分:秒');
grid on;

% Clipping
subplot(2,3,5);
plot(time_dt(3:end), clip_result(3:end), 'm');hold on
plot(time_dt(3:end), rssi_raw(3:end), 'k');
ylabel('dBm');
title('限幅濾波 (Clipping)');
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
ylim([-90, -45]);
xlabel('分:秒');
grid on;

% Kalman
subplot(2,3,6);
plot(time_dt(3:end), kalman_result(3:end), 'b');hold on
plot(time_dt(3:end), rssi_raw(3:end), 'k');
ylabel('dBm');
title('卡曼濾波 (Kalman)');
ylim([-90, -45]);
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
xlabel('分:秒');
grid on;

