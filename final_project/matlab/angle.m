% 濾波分析主程式
clear; clc;
opts = detectImportOptions('/Users/prince_lego/Desktop/progarm/LeTaX/Mechatronics_Integration/final_project/Python/angle.txt', 'Delimiter', ',');
opts = setvartype(opts, {'Var2','Var3'}, 'double'); 
opts = setvaropts(opts, {'Var2','Var3'}, 'TreatAsMissing', 'None');

data = readtable('/Users/prince_lego/Desktop/progarm/LeTaX/Mechatronics_Integration/final_project/Python/angle.txt', opts);



% === 載入資料 ===
rssi_raw = data{:, 3};   % 假設每列是一個 RSSI 數值
time_str = data{:, 1};

% === 畫圖 ===
time_dt = datetime(time_str, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
time_dt.Format = 'mm:ss'; 
% 繪圖時用時間軸作為 x
figure;

% 原始
plot(time_dt, rssi_raw, 'k');
ylabel('度');
title('角度');
datetick('x', 'MM:SS');  % 強制 x 軸只顯示分秒
xlabel('分:秒');
grid on;
