import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# 1. 定義輸入變數（角度誤差 Angle, 位置誤差 Position）
angle = ctrl.Antecedent(np.arange(-30, 31, 1), 'angle')  # 角度範圍：-30° 到 30°


# 2. 定義輸出變數（前進速度 Vx, 側向速度 Vy, 旋轉速度 ω）
Vx = ctrl.Consequent(np.arange(-100, 101, 1), 'Vx')  # 前進速度 (-100~100)
Vy = ctrl.Consequent(np.arange(-100, 101, 1), 'Vy')  # 側向速度 (-100~100)
omega = ctrl.Consequent(np.arange(-50, 51, 1), 'omega')  # 旋轉速度 (-100~100)

# 3. 定義隸屬函數
angle['BL'] = fuzz.trimf(angle.universe, [-30, -30, -15])  # 大偏左
angle['SL'] = fuzz.trimf(angle.universe, [-20, -10, 0])   # 小偏左
angle['Z'] = fuzz.trimf(angle.universe, [-5, 0, 5])       # 對準
angle['SR'] = fuzz.trimf(angle.universe, [0, 10, 20])     # 小偏右
angle['BR'] = fuzz.trimf(angle.universe, [15, 30, 30])    # 大偏右

Vx['S'] = fuzz.trimf(Vx.universe, [-100, -50, 0])  # 慢速
Vx['M'] = fuzz.trimf(Vx.universe, [-50, 0, 50])    # 中速
Vx['F'] = fuzz.trimf(Vx.universe, [0, 50, 100])    # 快速


Vy['L'] = fuzz.trimf(Vy.universe, [-100, -50, 0])  # 向左
Vy['Z'] = fuzz.trimf(Vy.universe, [-50, 0, 50])       # 中間
Vy['R'] = fuzz.trimf(Vy.universe, [0, 50, 100])     # 向右

omega['CCW'] = fuzz.trimf(omega.universe, [-50, -50, -25])  # 逆時針旋轉
omega['Z'] = fuzz.trimf(omega.universe, [-25, 0, 25])        # 無旋轉
omega['CW'] = fuzz.trimf(omega.universe, [25, 50, 50])      # 順時針旋轉

# 4. 定義模糊規則
rule1 = ctrl.Rule(angle['BL'], (Vx['S'], Vy['L'], omega['CCW']))
rule2 = ctrl.Rule(angle['SL'], (Vx['M'], Vy['L'], omega['Z']))
rule3 = ctrl.Rule(angle['Z'], (Vx['F'], Vy['Z'], omega['Z']))
rule4 = ctrl.Rule(angle['SR'], (Vx['M'], Vy['R'], omega['Z']))
rule5 = ctrl.Rule(angle['BR'], (Vx['S'], Vy['R'], omega['CW']))


# 5. 建立模糊控制系統
control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])  # Include all your rules

simulator = ctrl.ControlSystemSimulation(control_system)

# 6. 測試模糊控制系統
simulator.input['angle'] = -20  # 測試：角度偏左 20°
simulator.compute()

# 7. 計算四顆麥克納姆輪速度
Vx_out = simulator.output['Vx']
Vy_out = simulator.output['Vy']
omega_out = simulator.output['omega']
'''
V_FL = np.clip(Vx_out - Vy_out - omega_out, -100, 100)
V_FR = np.clip(Vx_out + Vy_out + omega_out, -100, 100)
V_RL = np.clip(Vx_out + Vy_out - omega_out, -100, 100)
V_RR = np.clip(Vx_out - Vy_out + omega_out, -100, 100)
'''
# 對輪速進行限制，保證輪速在合理範圍內
V_FL = Vx_out - Vy_out - omega_out
V_FR = Vx_out + Vy_out + omega_out
V_RL = Vx_out + Vy_out - omega_out
V_RR = Vx_out - Vy_out + omega_out

# 輸出結果
print(f"Front Left Wheel Speed: {V_FL:.2f}")
print(f"Front Right Wheel Speed: {V_FR:.2f}")
print(f"Rear Left Wheel Speed: {V_RL:.2f}")
print(f"Rear Right Wheel Speed: {V_RR:.2f}")


plt.figure()
plt.plot(angle.universe, angle['BL'].mf, label='BL')
plt.plot(angle.universe, angle['SL'].mf, label='SL')
plt.plot(angle.universe, angle['Z'].mf, label='Z')
plt.plot(angle.universe, angle['SR'].mf, label='SR')
plt.plot(angle.universe, angle['BR'].mf, label='BR')
plt.title('Position Membership Functions')
plt.legend()
plt.show()

# 繪製 Vx 的隸屬函數
plt.figure()
plt.plot(Vx.universe, Vx['S'].mf, label='S')
plt.plot(Vx.universe, Vx['M'].mf, label='M')
plt.plot(Vx.universe, Vx['F'].mf, label='F')
plt.title('Vx Membership Functions')
plt.legend()
plt.show()

# 繪製 Omega 的隸屬函數
plt.figure()
plt.plot(omega.universe, omega['CCW'].mf, label='CCW')
plt.plot(omega.universe, omega['Z'].mf, label='Z')
plt.plot(omega.universe, omega['CW'].mf, label='CW')
plt.title('Omega Membership Functions')
plt.legend()
plt.show()