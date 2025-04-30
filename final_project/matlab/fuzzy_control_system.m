function [Vx_out, Vy_out, Omega_out]  = fuzzy_control_system(angles, positions)

    % 建立模糊控制器
    fis = mamfis('Name', 'SimpleFuzzyControl');
    fis.DefuzzificationMethod = 'centroid';

    %% === 輸入 ===
    % 角度輸入 [-30, 30]
    fis = addInput(fis, [-60 60], 'Name', 'angle');
    fis = addMF(fis, 'angle', 'trapmf', [-60, -60, -25, -15], 'Name', 'BL');
    fis = addMF(fis, 'angle', 'trimf',[-25, -15, 0], 'Name', 'SL');
    fis = addMF(fis, 'angle', 'trimf', [-5, 0, 5], 'Name', 'Z');
    fis = addMF(fis, 'angle', 'trimf', [0, 15, 25], 'Name', 'SR');
    fis = addMF(fis, 'angle', 'trapmf', [15, 25, 60, 60], 'Name', 'BR');

    % 偏移輸入 [-50, 50]
    fis = addInput(fis, [-50 50], 'Name', 'position');
    fis = addMF(fis, 'position', 'trapmf', [-50, -50, -35, -20], 'Name', 'BL');
    fis = addMF(fis, 'position', 'trimf',  [-35, -20, 0], 'Name', 'SL');
    fis = addMF(fis, 'position', 'trimf',[-10, 0, 10], 'Name', 'Z');
    fis = addMF(fis, 'position', 'trimf',[0, 20, 35], 'Name', 'SR');
    fis = addMF(fis, 'position', 'trapmf', [25 50 50 50], 'Name', 'BR');

    %% === 輸出 ===
    fis = addOutput(fis, [0 150], 'Name', 'Vx'); % index 1
    fis = addMF(fis, 'Vx', 'trapmf', [0 0 50 80], 'Name', 'S');
    fis = addMF(fis, 'Vx', 'trimf', [50 80 100], 'Name', 'M');
    fis = addMF(fis, 'Vx', 'trapmf', [80 100 150 150], 'Name', 'F');

    fis = addOutput(fis, [-30 30], 'Name', 'Vy'); % index 2
    fis = addMF(fis, 'Vy', 'trapmf', [-30 -30 -20 -16], 'Name', 'LL');
    fis = addMF(fis, 'Vy', 'trimf', [-16 -12 0], 'Name', 'L');
    fis = addMF(fis, 'Vy', 'trimf', [-14 0 14], 'Name', 'Z');
    fis = addMF(fis, 'Vy', 'trimf', [0 12 16], 'Name', 'R');
    fis = addMF(fis, 'Vy', 'trapmf', [16 20 30 30], 'Name', 'RR');

    fis = addOutput(fis, [-20 20], 'Name', 'Omega'); % index 3
    fis = addMF(fis, 'Omega', 'trapmf', [-20 -20 -10 -8], 'Name', 'CCW2');
    fis = addMF(fis, 'Omega', 'trimf', [-10 -4 0], 'Name', 'CCW');
    fis = addMF(fis, 'Omega', 'trimf', [-4 0 4], 'Name', 'Z');
    fis = addMF(fis, 'Omega', 'trimf', [0 4 10], 'Name', 'CW');
    fis = addMF(fis, 'Omega', 'trapmf', [8 10 20 20], 'Name', 'CW2');
    
    %% === 規則 ===
     ruleList = [
        1 1 1 1 1 1 1;
        1 2 1 2 1 1 1;
        1 3 2 3 1 1 1;
        1 4 1 4 1 1 1;
        1 5 1 5 1 1 1;

        2 1 1 1 2 1 1;
        2 2 2 2 2 1 1;
        2 3 3 3 2 1 1;
        2 4 2 4 2 1 1;
        2 5 1 5 2 1 1;

        3 1 2 1 3 1 1;
        3 2 3 2 3 1 1;
        3 3 3 3 3 1 1;
        3 4 3 4 3 1 1;
        3 5 2 5 3 1 1;

        4 1 1 1 4 1 1;
        4 2 2 2 4 1 1;
        4 3 3 3 4 1 1;
        4 4 2 4 4 1 1;
        4 5 1 5 4 1 1;

        5 1 1 1 5 1 1;
        5 2 1 2 5 1 1;
        5 3 2 3 5 1 1;
        5 4 1 4 5 1 1;
        5 5 1 5 5 1 1;
    ];
     fis = addRule(fis, ruleList);

    %% === 輸入格式處理 ===
    if isscalar(angles), angles = angles(:)'; end
    if isscalar(positions), positions = positions(:)'; end

    % 強制輸入限制在範圍內，避免 NaN
    angles = max(min(angles, 60), -60);
    positions = max(min(positions, 50), -50);

    % 初始化結果
    num_inputs = length(angles);
    Vx_results = zeros(1, num_inputs);
    Vy_results = zeros(1, num_inputs);
    Omega_results = zeros(1, num_inputs);

    % 模糊推論
    for i = 1:num_inputs
        input_pair = [angles(i), positions(i)];
        out = evalfis(fis, input_pair);
        if any(isnan(out))
            warning('evalfis 回傳 NaN：angle=%.2f, position=%.2f', angles(i), positions(i));
        end
        Vx_results(i) = out(1);
        Vy_results(i) = out(2);
        Omega_results(i) = out(3);
    end

    keep_ratio = 0.8;
    cutoff = max(1, floor(length(Vx_results) * keep_ratio));
    sorted_Vx = sort(Vx_results); % 升冪排序
    sorted_Vy = sort(Vy_results); % 升冪排序
    sorted_Omega = sort(Omega_results); % 升冪排序
    % 計算平均值（前 cutoff 筆），若資料為空則輸出 0
    if ~isempty(sorted_Vx)
        Vx_out = mean(sorted_Vx(1:cutoff));
    else
        Vx_out = 0;
    end
    
    if ~isempty(sorted_Vy)
        Vy_out = mean(sorted_Vy(1:cutoff));
    else
        Vy_out = 0;
    end
    
    if ~isempty(sorted_Omega)
        Omega_out = mean(sorted_Omega(1:cutoff));
    else
        Omega_out = 0;
    end

end
