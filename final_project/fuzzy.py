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
    angle = ctrl.Antecedent(np.arange(-30, 30, 0.01), 'angle')
    position = ctrl.Antecedent(np.arange(-50, 50, 0.01), 'position')

    # 定義輸出變數
    Vx = ctrl.Consequent(np.arange(0, 200, 0.01), 'Vx')
    Vy = ctrl.Consequent(np.arange(-50, 50, 0.01), 'Vy')
    omega = ctrl.Consequent(np.arange(-20, 20, 0.01), 'omega')

    # 隸屬函數定義
    # 角度範圍 [-30, 30]
    angle['BL'] = fuzz.trapmf(angle.universe, [-30, -30, -25, -15])
    angle['SL'] = fuzz.trimf(angle.universe, [-25, -15, 0])
    angle['Z']  = fuzz.trimf(angle.universe, [-5, 0, 5])
    angle['SR'] = fuzz.trimf(angle.universe, [0, 15, 25])
    angle['BR'] = fuzz.trapmf(angle.universe, [15, 25, 30, 30])

    # 位置範圍 [-50, 50]
    position['BL'] = fuzz.trapmf(position.universe, [-50, -50, -35, -20])
    position['SL'] = fuzz.trimf(position.universe, [-35, -20, 0])
    position['Z']  = fuzz.trimf(position.universe, [-10, 0, 10])
    position['SR'] = fuzz.trimf(position.universe, [0, 20, 35])
    position['BR'] = fuzz.trapmf(position.universe, [20, 35, 50, 50])

    Vx['S'] = fuzz.trapmf(Vx.universe, [0, 0, 50, 100])
    Vx['M'] = fuzz.trimf(Vx.universe, [50, 100, 150])
    Vx['F'] = fuzz.trapmf(Vx.universe, [100, 150, 200, 200])

    Vy['LL'] = fuzz.trapmf(Vy.universe, [-50, -50, -40, -20])
    Vy['L'] = fuzz.trimf(Vy.universe, [-30, -15, 0])
    Vy['Z'] = fuzz.trimf(Vy.universe, [-15, 0, 15])
    Vy['R'] = fuzz.trimf(Vy.universe, [0, 15, 30])
    Vy['RR'] = fuzz.trapmf(Vy.universe, [20, 40, 50, 50])

    omega['CCW2'] = fuzz.trapmf(omega.universe, [-20, -20, -15, -10])
    omega['CCW'] = fuzz.trimf(omega.universe, [-15, -10, 0])
    omega['Z'] = fuzz.trimf(omega.universe, [-10, 0, 10])
    omega['CW'] = fuzz.trimf(omega.universe, [0, 10, 15])
    omega['CW2'] = fuzz.trapmf(omega.universe, [10, 15, 20, 20])

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
    omega_out = np.mean(sorted(omega_results)[:cutoff])if omega_results else 0


    """


    #角度的隸屬函數
    plt.figure()
    plt.plot(angle.universe, angle['BL'].mf, label='BL')
    plt.plot(angle.universe, angle['SL'].mf, label='SL')
    plt.plot(angle.universe, angle['Z'].mf, label='Z')
    plt.plot(angle.universe, angle['SR'].mf, label='SR')
    plt.plot(angle.universe, angle['BR'].mf, label='BR')
    plt.title('Angle Membership Functions')
    plt.xlabel('Angle Offset (deg)')                           
    plt.ylabel('Membership Degree')                                                            
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()                                      
    plt.show()

    #位置的隸屬函數
    plt.figure()
    plt.plot(position.universe, position['BL'].mf, label='BL')
    plt.plot(position.universe, position['SL'].mf, label='SL')
    plt.plot(position.universe, position['Z'].mf, label='Z')
    plt.plot(position.universe, position['SR'].mf, label='SR')
    plt.plot(position.universe, position['BR'].mf, label='BR')
    plt.title('Distance Membership Functions')
    plt.xlabel('Distance Offset (mm)')                           
    plt.ylabel('Membership Degree')                                                            
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()                                      
    plt.show()

    #Vx 的隸屬函數
    plt.figure()
    plt.plot(Vx.universe, Vx['S'].mf, label='S')
    plt.plot(Vx.universe, Vx['M'].mf, label='M')
    plt.plot(Vx.universe, Vx['F'].mf, label='F')
    plt.title('Vx Membership Functions')
    plt.xlabel('Vx (mm/s)')                           
    plt.ylabel('Membership Degree')                                                            
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()                                      
    plt.show()

    #Vy 的隸屬函數
    plt.figure()
    plt.plot(Vy.universe, Vy['LL'].mf, label='LL')
    plt.plot(Vy.universe, Vy['L'].mf, label='L')
    plt.plot(Vy.universe, Vy['Z'].mf, label='Z')
    plt.plot(Vy.universe, Vy['R'].mf, label='R')
    plt.plot(Vy.universe, Vy['RR'].mf, label='RR')
    plt.title('Vy Membership Functions')
    plt.xlabel('Vy (mm/s)')                           
    plt.ylabel('Membership Degree')                                                            
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()                                      
    plt.show()

    #Omega 的隸屬函數
    plt.figure()
    plt.plot(omega.universe, omega['CCW2'].mf, label='CCW2')
    plt.plot(omega.universe, omega['CCW'].mf, label='CCW')
    plt.plot(omega.universe, omega['Z'].mf, label='Z')
    plt.plot(omega.universe, omega['CW'].mf, label='CW')
    plt.plot(omega.universe, omega['CW2'].mf, label='CW2')
    plt.title('Omega Membership Functions')
    plt.xlabel('Omega (deg/s)')                           
    plt.ylabel('Membership Degree')                                                            
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()                                      
    plt.show()

    """
    return {"Vx": Vx_out, "Vy": Vy_out, "omega": omega_out}



angles = [15,5,20]
positions = [30,40,10]

output = calculate_mecanum_wheel_speeds(angles, positions)
print(f"Vx: {output['Vx']:.2f}, Vy: {output['Vy']:.2f}, Omega: {output['omega']:.2f}")

"""
angle_range = np.linspace(-30, 30, 50)
position_range = np.linspace(-50, 50, 50)

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
plt.imshow(Vx_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis',aspect='auto')
plt.title('Vx Output')
plt.xlabel('Angle (deg)')
plt.ylabel('Distance Offset (mm)')
plt.colorbar(label='Vx (mm/s)')
plt.tight_layout()
plt.show()

# 顯示 Vy 的顏色圖
plt.figure(figsize=(8, 6))
plt.imshow(Vy_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis',aspect='auto')
plt.title('Vy Output')
plt.xlabel('Angle (deg)')
plt.ylabel('Distance Offset (mm)')
plt.colorbar(label='Vy (mm/s)')
plt.show()

# 顯示 omega 的顏色圖
plt.figure(figsize=(8, 6))
plt.imshow(omega_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis',aspect='auto')
plt.title('Omega Output')
plt.xlabel('Angle (deg)')
plt.ylabel('Distance Offset (mm)')
plt.colorbar(label='Omega (deg/s)')
plt.show()







def mecanum_v(vx, vy, omega,R,r):
    omega=np.pi*omega/180
    return (vx - vy - omega*R)/r, (vx + vy + omega*R)/r, (vx + vy - omega*R)/r, (vx - vy + omega*R)/r

FL_grid,FR_grid,RL_grid,RR_grid=mecanum_v(Vx_grid, Vy_grid,omega_grid,60,55)


plt.figure(figsize=(8, 6))
plt.imshow(FL_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis',aspect='auto')
plt.colorbar(label='FL(rad/s)')
plt.title('Distance offset and angle corresponding to FL')
plt.ylabel('Distance Offset (mm)')
plt.xlabel('Angle (deg)')
plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(FR_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis',aspect='auto')
plt.colorbar(label='FR(rad/s)')
plt.title('Distance offset and angle corresponding to FR')
plt.ylabel('Distance Offset (mm)')
plt.xlabel('Angle (deg)')
plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(RL_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis',aspect='auto')
plt.colorbar(label='FL(rad/s)')
plt.title('Distance offset and angle corresponding to RL')
plt.ylabel('Distance Offset (mm)')
plt.xlabel('Angle (deg)')
plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(RR_grid, extent=[angle_range.min(), angle_range.max(), position_range.min(), position_range.max()], origin='lower', cmap='viridis',aspect='auto')
plt.colorbar(label='RR(rad/s)')
plt.title('Distance offset and angle corresponding to RR')
plt.ylabel('Distance Offset (mm)')
plt.xlabel('Angle (deg)')
plt.tight_layout()
plt.show()






# 畫出 Vx、Vy 和 omega 的 3D 圖
fig = plt.figure(figsize=(18, 7))

# Vx 圖
ax1 = fig.add_subplot(131, projection='3d')
surf_vx = ax1.plot_surface(angle_grid, position_grid, Vx_grid, cmap='viridis')
ax1.set_title('Distance offset and angle corresponding to Vx Output')
ax1.set_xlabel('Angle (deg)')
ax1.set_ylabel('Distance Offset (mm)')
ax1.set_zlabel('Vx (Forward Speed)')
fig.colorbar(surf_vx, shrink=0.5, aspect=10)

# Vy 圖
ax2 = fig.add_subplot(132, projection='3d')
surf_vy = ax2.plot_surface(angle_grid, position_grid, Vy_grid, cmap='viridis')
ax2.set_title('Distance offset and angle corresponding to Vy Output')
ax2.set_xlabel('Angle (deg)')
ax2.set_ylabel('Distance Offset (mm)')
ax2.set_zlabel('Vy (Sideways Speed)')
fig.colorbar(surf_vy, shrink=0.5, aspect=10)

# omega 圖
ax3 = fig.add_subplot(133, projection='3d')
surf_omega = ax3.plot_surface(angle_grid, position_grid, omega_grid, cmap='viridis')
ax3.set_title('Distance offset and angle corresponding to Omega Output')
ax3.set_xlabel('Angle (deg)')
ax3.set_ylabel('Distance Offset (mm)')
ax3.set_zlabel('Omega (Angular Speed)')
fig.colorbar(surf_omega, shrink=0.5, aspect=10)

plt.tight_layout()
plt.show()




fig = plt.figure(figsize=(12, 7))
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(angle_grid, position_grid, FL_grid, cmap='viridis')
ax.set_title('Distance offset and angle corresponding to FL')
ax.set_xlabel('Angle (deg)')
ax.set_ylabel('Distance Offset (mm)')
ax.set_zlabel('FL(rad/s)')
# 加入 colorbar 來顯示顏色對應的數值
fig.colorbar(surf, shrink=0.5, aspect=10)
# 顯示圖
plt.tight_layout()
plt.show()

fig = plt.figure(figsize=(12, 7))
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(angle_grid, position_grid, FR_grid, cmap='viridis')
ax.set_title('Distance offset and angle corresponding to FR')
ax.set_xlabel('Angle (deg)')
ax.set_ylabel('Distance Offset (mm)')
ax.set_zlabel('FR(rad/s)')
# 加入 colorbar 來顯示顏色對應的數值
fig.colorbar(surf, shrink=0.5, aspect=10)
# 顯示圖
plt.tight_layout()
plt.show()


fig = plt.figure(figsize=(12, 7))
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(angle_grid, position_grid, RR_grid, cmap='viridis')
ax.set_title('Distance offset and angle corresponding to RR')
ax.set_xlabel('Angle (deg)')
ax.set_ylabel('Distance Offset (mm)')
ax.set_zlabel('RR(rad/s)')
# 加入 colorbar 來顯示顏色對應的數值
fig.colorbar(surf, shrink=0.5, aspect=10)
# 顯示圖
plt.tight_layout()
plt.show()


fig = plt.figure(figsize=(12, 7))
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(angle_grid, position_grid, RL_grid, cmap='viridis')
ax.set_title('Distance offset and angle corresponding to RL')
ax.set_xlabel('Angle (deg)')
ax.set_ylabel('Distance Offset (mm)')
ax.set_zlabel('RL(rad/s)')
# 加入 colorbar 來顯示顏色對應的數值
fig.colorbar(surf, shrink=0.5, aspect=10)
# 顯示圖
plt.tight_layout()
plt.show()
"""