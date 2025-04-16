import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def calculate_mecanum_wheel_speeds(angle_input: list[float], position_input: list[float]) -> dict:
    """
    根據角度偏差與位置偏移進行模糊控制，輸出麥克納姆輪車的移動參數

    輸入:
        angle_input: List of float, 各位置的角度差 (-30° ~ 30°)
        position_input: List of float, 各位置的標線偏移 (-10 ~ 10)

    輸出:
        dict: {'Vx': 前進速度, 'Vy': 橫移速度, 'omega': 角速度}
    """

    if len(angle_input) != len(position_input):
        raise ValueError("angle_input 和 position_input 長度必須一致")

    # 定義模糊輸入變數
    angle = ctrl.Antecedent(np.arange(-30, 30.1, 0.1), 'angle')
    position = ctrl.Antecedent(np.arange(-10, 10.1, 0.1), 'position')

    # 定義輸出變數
    Vx = ctrl.Consequent(np.arange(0, 10, 0.1), 'Vx')
    Vy = ctrl.Consequent(np.arange(-10, 10, 0.1), 'Vy')
    omega = ctrl.Consequent(np.arange(-3, 3, 0.1), 'omega')

    # 隸屬函數定義
    angle.automf(5, names=['BL', 'SL', 'Z', 'SR', 'BR'])
    position.automf(5, names=['BL', 'SL', 'Z', 'SR', 'BR'])

    Vx['S'] = fuzz.trapmf(Vx.universe, [0, 0, 2.5, 5])
    Vx['M'] = fuzz.trimf(Vx.universe, [2.5, 5, 7.5])
    Vx['F'] = fuzz.trapmf(Vx.universe, [5, 7.5, 10, 10])

    Vy['LL'] = fuzz.trapmf(Vy.universe, [-10, -10, -7, -4])
    Vy['L'] = fuzz.trimf(Vy.universe, [-6, -3, 0])
    Vy['Z'] = fuzz.trimf(Vy.universe, [-4, 0, 4])
    Vy['R'] = fuzz.trimf(Vy.universe, [0, 3, 6])
    Vy['RR'] = fuzz.trapmf(Vy.universe, [4, 7, 10, 10])

    omega['CCW2'] = fuzz.trapmf(omega.universe, [-3, -3, -2, -1])
    omega['CCW'] = fuzz.trimf(omega.universe, [-2, -1, 0])
    omega['Z'] = fuzz.trimf(omega.universe, [-1, 0, 1])
    omega['CW'] = fuzz.trimf(omega.universe, [0, 1, 2])
    omega['CW2'] = fuzz.trapmf(omega.universe, [1, 2, 3, 3])

    # 規則定義（你已有的可維持不變）
    rules = [
        ctrl.Rule((angle['BL'] & position['BL']), (Vx['S'], Vy['RR'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['BL']), (Vx['S'], Vy['RR'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['BL']), (Vx['M'], Vy['RR'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['BL']), (Vx['S'], Vy['RR'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['BL']), (Vx['S'], Vy['RR'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['SL']), (Vx['S'], Vy['R'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['SL']), (Vx['M'], Vy['R'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['SL']), (Vx['F'], Vy['R'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['SL']), (Vx['M'], Vy['R'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['SL']), (Vx['S'], Vy['R'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['Z']), (Vx['S'], Vy['Z'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['Z']), (Vx['F'], Vy['Z'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['Z']), (Vx['F'], Vy['Z'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['Z']), (Vx['F'], Vy['Z'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['Z']), (Vx['S'], Vy['Z'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['SR']), (Vx['S'], Vy['L'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['SR']), (Vx['M'], Vy['L'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['SR']), (Vx['F'], Vy['L'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['SR']), (Vx['M'], Vy['L'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['SR']), (Vx['S'], Vy['L'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['BR']), (Vx['S'], Vy['LL'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['BR']), (Vx['S'], Vy['LL'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['BR']), (Vx['M'], Vy['LL'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['BR']), (Vx['S'], Vy['LL'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['BR']), (Vx['S'], Vy['LL'], omega['CCW2'])),
    ]

    control_system = ctrl.ControlSystem(rules)

    Vx_results, Vy_results, omega_results = [], [], []

    for a, p in zip(angle_input, position_input):
        sim = ctrl.ControlSystemSimulation(control_system)
        sim.input['angle'] = a
        sim.input['position'] = p
        sim.compute()
        Vx_results.append(sim.output['Vx'])
        Vy_results.append(sim.output['Vy'])
        omega_results.append(sim.output['omega'])

    if len(Vx_results) == 1:
        return {"Vx": Vx_results[0], "Vy": Vy_results[0], "omega": omega_results[0]}

    # 使用 alpha-cut 去除極端值 (去掉最大 20%)
    keep_ratio = 0.8
    cutoff = int(len(Vx_results) * keep_ratio)

    Vx_out = np.mean(sorted(Vx_results)[:cutoff]) if Vx_results else 0
    Vy_out = np.mean(sorted(Vy_results)[:cutoff]) if Vy_results else 0
    omega_out = np.mean(sorted(omega_results)[-cutoff:]) if omega_results else 0



    #"""
    #角度的隸屬函數
    plt.figure()
    plt.plot(angle.universe, angle['BL'].mf, label='BL')
    plt.plot(angle.universe, angle['SL'].mf, label='SL')
    plt.plot(angle.universe, angle['Z'].mf, label='Z')
    plt.plot(angle.universe, angle['SR'].mf, label='SR')
    plt.plot(angle.universe, angle['BR'].mf, label='BR')
    plt.title('Angle Membership Functions')
    plt.legend()
    plt.show()

    #位置的隸屬函數
    plt.figure()
    plt.plot(position.universe, position['BL'].mf, label='BL')
    plt.plot(position.universe, position['SL'].mf, label='SL')
    plt.plot(position.universe, position['Z'].mf, label='Z')
    plt.plot(position.universe, position['SR'].mf, label='SR')
    plt.plot(position.universe, position['BR'].mf, label='BR')
    plt.title('Position Membership Functions')
    plt.legend()
    plt.show()

    #Vx 的隸屬函數
    plt.figure()
    plt.plot(Vx.universe, Vx['S'].mf, label='S')
    plt.plot(Vx.universe, Vx['M'].mf, label='M')
    plt.plot(Vx.universe, Vx['F'].mf, label='F')
    plt.title('Vx Membership Functions')
    plt.legend()
    plt.show()

    #Vy 的隸屬函數
    plt.figure()
    plt.plot(Vy.universe, Vy['LL'].mf, label='LL')
    plt.plot(Vy.universe, Vy['L'].mf, label='L')
    plt.plot(Vy.universe, Vy['Z'].mf, label='Z')
    plt.plot(Vy.universe, Vy['R'].mf, label='R')
    plt.plot(Vy.universe, Vy['RR'].mf, label='RR')
    plt.title('Vy Membership Functions')
    plt.legend()
    plt.show()

    #Omega 的隸屬函數
    plt.figure()
    plt.plot(omega.universe, omega['CCW2'].mf, label='CCW2')
    plt.plot(omega.universe, omega['CCW'].mf, label='CCW')
    plt.plot(omega.universe, omega['Z'].mf, label='Z')
    plt.plot(omega.universe, omega['CW'].mf, label='CW')
    plt.plot(omega.universe, omega['CW2'].mf, label='CW2')
    plt.title('Omega Membership Functions')
    plt.legend()
    plt.show()
    #"""

    return {"Vx": Vx_out, "Vy": Vy_out, "omega": omega_out}



angles = [5, -2, 0, 3]
positions = [1, -4, 0, 2]

output = calculate_mecanum_wheel_speeds(angles, positions)
print(f"Vx: {output['Vx']:.2f}, Vy: {output['Vy']:.2f}, Omega: {output['omega']:.2f}")

#"""
angle_range = np.linspace(-30, 30, 5)
position_range = np.linspace(-10, 10, 5)

# 建立網格
angle_grid, position_grid = np.meshgrid(angle_range, position_range)
Vx_grid = np.zeros_like(angle_grid)
Vy_grid = np.zeros_like(angle_grid)
omega_grid = np.zeros_like(angle_grid)


# 計算每一組輸入對應的 Vx
for i in range(angle_grid.shape[0]):
    for j in range(angle_grid.shape[1]):
        angle_val = angle_grid[i, j]
        position_val = position_grid[i, j]
        result = calculate_mecanum_wheel_speeds([angle_val], [position_val])
        Vx_grid[i, j] = result['Vx']
        Vy_grid[i, j] = result['Vy']
        omega_grid[i, j] = result['omega']
        
        # 打印進度
        if (i * angle_grid.shape[1] + j) % 10 == 0:  # 每10次顯示一次進度
            print(f"Processing {i * angle_grid.shape[1] + j}/{angle_grid.size}...")


# 顯示 Vx 的顏色圖
plt.figure(figsize=(8, 6))
plt.imshow(Vx_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis')
plt.title('Vx Output')
plt.xlabel('Angle (°)')
plt.ylabel('Position Offset')
plt.colorbar(label='Vx')
plt.show()

# 顯示 Vy 的顏色圖
plt.figure(figsize=(8, 6))
plt.imshow(Vy_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis')
plt.title('Vy Output')
plt.xlabel('Angle (°)')
plt.ylabel('Position Offset')
plt.colorbar(label='Vy')
plt.show()

# 顯示 omega 的顏色圖
plt.figure(figsize=(8, 6))
plt.imshow(omega_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis')
plt.title('Omega Output')
plt.xlabel('Angle (°)')
plt.ylabel('Position Offset')
plt.colorbar(label='Omega')
plt.show()

V_sum_grid = Vx_grid + Vy_grid+omega_grid

plt.figure(figsize=(8, 6))
plt.imshow(V_sum_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis')
plt.colorbar(label='Vx + Vy')
plt.title('Sum of Vx and Vy')
plt.xlabel('Position Offset')
plt.ylabel('Angle (°)')
plt.tight_layout()
plt.show()

# 畫出 Vx、Vy 和 omega 的 3D 圖
fig = plt.figure(figsize=(18, 7))

# Vx 圖
ax1 = fig.add_subplot(131, projection='3d')
surf_vx = ax1.plot_surface(angle_grid, position_grid, Vx_grid, cmap='viridis')
ax1.set_title('Vx Output Surface')
ax1.set_xlabel('Angle (°)')
ax1.set_ylabel('Position Offset')
ax1.set_zlabel('Vx (Forward Speed)')
fig.colorbar(surf_vx, shrink=0.5, aspect=10)

# Vy 圖
ax2 = fig.add_subplot(132, projection='3d')
surf_vy = ax2.plot_surface(angle_grid, position_grid, Vy_grid, cmap='viridis')
ax2.set_title('Vy Output Surface')
ax2.set_xlabel('Angle (°)')
ax2.set_ylabel('Position Offset')
ax2.set_zlabel('Vy (Sideways Speed)')
fig.colorbar(surf_vy, shrink=0.5, aspect=10)

# omega 圖
ax3 = fig.add_subplot(133, projection='3d')
surf_omega = ax3.plot_surface(angle_grid, position_grid, omega_grid, cmap='viridis')
ax3.set_title('Omega Output Surface')
ax3.set_xlabel('Angle (°)')
ax3.set_ylabel('Position Offset')
ax3.set_zlabel('Omega (Angular Speed)')
fig.colorbar(surf_omega, shrink=0.5, aspect=10)

# 顯示圖表
plt.tight_layout()
plt.show()


# 畫出 3D 圖
fig = plt.figure(figsize=(12, 7))
ax = fig.add_subplot(111, projection='3d')

# 使用 plot_surface 畫出 3D 表面
surf = ax.plot_surface(angle_grid, position_grid, V_sum_grid, cmap='viridis')

# 設定標題和軸標籤
ax.set_title('Sum of Vx and Vy (3D Surface)')
ax.set_xlabel('Angle (°)')
ax.set_ylabel('Position Offset')
ax.set_zlabel('Vx + Vy')

# 加入 colorbar 來顯示顏色對應的數值
fig.colorbar(surf, shrink=0.5, aspect=10)

# 顯示圖
plt.tight_layout()
plt.show()
