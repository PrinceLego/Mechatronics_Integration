import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# 1. 定義輸入變數（位置偏移量 Position）
position = ctrl.Antecedent(np.arange(-10, 11, 1), 'position')  # 位置偏移範圍：-10 到 10

# 2. 定義輸出變數（前進速度 Vx, 側向速度 Vy, 旋轉速度 ω）
Vx = ctrl.Consequent(np.arange(-100, 101, 1), 'Vx')  # 前進速度 (-100~100)
Vy = ctrl.Consequent(np.arange(-100, 101, 1), 'Vy')  # 側向速度 (-100~100)
omega = ctrl.Consequent(np.arange(-50, 51, 1), 'omega')  # 旋轉速度 (-50~50)

# 3. 定義隸屬函數
position['BL'] = fuzz.trimf(position.universe, [-10, -10, -5])  # 大偏左
position['SL'] = fuzz.trimf(position.universe, [-7, -3, 0])    # 小偏左
position['Z'] = fuzz.trimf(position.universe, [-3, 0, 3])      # 對準
position['SR'] = fuzz.trimf(position.universe, [0, 3, 7])      # 小偏右
position['BR'] = fuzz.trimf(position.universe, [5, 10, 10])    # 大偏右

Vx['S'] = fuzz.trimf(Vx.universe, [-100, -50, 0])  # 慢速
Vx['M'] = fuzz.trimf(Vx.universe, [-50, 0, 50])    # 中速
Vx['F'] = fuzz.trimf(Vx.universe, [0, 50, 100])    # 快速

Vy['L'] = fuzz.trimf(Vy.universe, [-100, -50, 0])  # 向左
Vy['Z'] = fuzz.trimf(Vy.universe, [-50, 0, 50])    # 中間
Vy['R'] = fuzz.trimf(Vy.universe, [0, 50, 100])    # 向右

omega['CCW'] = fuzz.trimf(omega.universe, [-50, -50, -25])  # 逆時針旋轉
omega['Z'] = fuzz.trimf(omega.universe, [-25, 0, 25])        # 無旋轉
omega['CW'] = fuzz.trimf(omega.universe, [25, 50, 50])       # 順時針旋轉

# 4. 定義模糊規則
rule1 = ctrl.Rule(position['BL'], (Vx['S'], Vy['L'], omega['CCW']))
rule2 = ctrl.Rule(position['SL'], (Vx['M'], Vy['L'], omega['Z']))
rule3 = ctrl.Rule(position['Z'], (Vx['F'], Vy['Z'], omega['Z']))
rule4 = ctrl.Rule(position['SR'], (Vx['M'], Vy['R'], omega['Z']))
rule5 = ctrl.Rule(position['BR'], (Vx['S'], Vy['R'], omega['CW']))

# 5. 建立模糊控制系統
control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])  # Include all your rules

simulator = ctrl.ControlSystemSimulation(control_system)

# 6. 測試模糊控制系統
simulator.input['position'] = -3  # 測試：位置偏左 5
simulator.compute()

# 7. 計算四顆麥克納姆輪速度
Vx_out = simulator.output['Vx']
Vy_out = simulator.output['Vy']
omega_out = simulator.output['omega']

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
plt.plot(position.universe, position['BL'].mf, label='BL')
plt.plot(position.universe, position['SL'].mf, label='SL')
plt.plot(position.universe, position['Z'].mf, label='Z')
plt.plot(position.universe, position['SR'].mf, label='SR')
plt.plot(position.universe, position['BR'].mf, label='BR')
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