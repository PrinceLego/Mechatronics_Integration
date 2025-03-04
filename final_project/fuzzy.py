import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

def calculate_mecanum_wheel_speeds(angle_input, position_input, w_a=0.6, w_p=0.4):
    # 1. 定義輸入變數
    angle = ctrl.Antecedent(np.arange(-30, 31, 1), 'angle')  # 角度偏差 (-30° ~ 30°)
    position = ctrl.Antecedent(np.arange(-10, 11, 1), 'position')  # 位置偏移 (-10 ~ 10)

    # 2. 定義輸出變數
    Vx = ctrl.Consequent(np.arange(-100, 101, 1), 'Vx')
    Vy = ctrl.Consequent(np.arange(-100, 101, 1), 'Vy')
    omega = ctrl.Consequent(np.arange(-50, 51, 1), 'omega')

    # 3. 定義隸屬函數
    angle['BL'] = fuzz.trimf(angle.universe, [-30, -30, -15])
    angle['SL'] = fuzz.trimf(angle.universe, [-20, -10, 0])
    angle['Z'] = fuzz.trimf(angle.universe, [-5, 0, 5])
    angle['SR'] = fuzz.trimf(angle.universe, [0, 10, 20])
    angle['BR'] = fuzz.trimf(angle.universe, [15, 30, 30])

    position['BL'] = fuzz.trimf(position.universe, [-10, -10, -5])
    position['SL'] = fuzz.trimf(position.universe, [-7, -3, 0])
    position['Z'] = fuzz.trimf(position.universe, [-3, 0, 3])
    position['SR'] = fuzz.trimf(position.universe, [0, 3, 7])
    position['BR'] = fuzz.trimf(position.universe, [5, 10, 10])

    Vx['S'] = fuzz.trimf(Vx.universe, [-100, -50, 0])
    Vx['M'] = fuzz.trimf(Vx.universe, [-50, 0, 50])
    Vx['F'] = fuzz.trimf(Vx.universe, [0, 50, 100])

    Vy['L'] = fuzz.trimf(Vy.universe, [-100, -50, 0])
    Vy['Z'] = fuzz.trimf(Vy.universe, [-50, 0, 50])
    Vy['R'] = fuzz.trimf(Vy.universe, [0, 50, 100])

    omega['CCW'] = fuzz.trimf(omega.universe, [-50, -50, -25])
    omega['Z'] = fuzz.trimf(omega.universe, [-25, 0, 25])
    omega['CW'] = fuzz.trimf(omega.universe, [25, 50, 50])

    # 4. 定義角度控制規則
    rules_angle = [
        ctrl.Rule(angle['BL'], (Vx['S'], Vy['L'], omega['CCW'])),
        ctrl.Rule(angle['SL'], (Vx['M'], Vy['L'], omega['Z'])),
        ctrl.Rule(angle['Z'], (Vx['F'], Vy['Z'], omega['Z'])),
        ctrl.Rule(angle['SR'], (Vx['M'], Vy['R'], omega['Z'])),
        ctrl.Rule(angle['BR'], (Vx['S'], Vy['R'], omega['CW']))
    ]

    # 5. 定義位置控制規則
    rules_position = [
        ctrl.Rule(position['BL'], (Vx['S'], Vy['L'], omega['CCW'])),
        ctrl.Rule(position['SL'], (Vx['M'], Vy['L'], omega['Z'])),
        ctrl.Rule(position['Z'], (Vx['F'], Vy['Z'], omega['Z'])),
        ctrl.Rule(position['SR'], (Vx['M'], Vy['R'], omega['Z'])),
        ctrl.Rule(position['BR'], (Vx['S'], Vy['R'], omega['CW']))
    ]

    # 6. 建立模糊控制系統
    control_system_angle = ctrl.ControlSystem(rules_angle)
    control_system_position = ctrl.ControlSystem(rules_position)

    sim_angle = ctrl.ControlSystemSimulation(control_system_angle)
    sim_position = ctrl.ControlSystemSimulation(control_system_position)

    # 7. 設定輸入
    sim_angle.input['angle'] = angle_input  # 角度
    sim_position.input['position'] = position_input  # 位置

    sim_angle.compute()
    sim_position.compute()

    # 8. 加權合併輸出
    Vx_out = w_a * sim_angle.output['Vx'] + w_p * sim_position.output['Vx']
    Vy_out = w_a * sim_angle.output['Vy'] + w_p * sim_position.output['Vy']
    omega_out = w_a * sim_angle.output['omega'] + w_p * sim_position.output['omega']

    # 9. 計算麥克納姆輪速度
    V_FL = Vx_out - Vy_out - omega_out
    V_FR = Vx_out + Vy_out + omega_out
    V_RL = Vx_out + Vy_out - omega_out
    V_RR = Vx_out - Vy_out + omega_out
    """
    # 畫出角度的隸屬函數
    plt.figure()
    plt.plot(angle.universe, angle['BL'].mf, label='BL')
    plt.plot(angle.universe, angle['SL'].mf, label='SL')
    plt.plot(angle.universe, angle['Z'].mf, label='Z')
    plt.plot(angle.universe, angle['SR'].mf, label='SR')
    plt.plot(angle.universe, angle['BR'].mf, label='BR')
    plt.title('Angle Membership Functions')
    plt.legend()
    plt.show()

    # 畫出位置的隸屬函數
    plt.figure()
    plt.plot(position.universe, position['BL'].mf, label='BL')
    plt.plot(position.universe, position['SL'].mf, label='SL')
    plt.plot(position.universe, position['Z'].mf, label='Z')
    plt.plot(position.universe, position['SR'].mf, label='SR')
    plt.plot(position.universe, position['BR'].mf, label='BR')
    plt.title('Position Membership Functions')
    plt.legend()
    plt.show()

    # 畫出 Vx 的隸屬函數
    plt.figure()
    plt.plot(Vx.universe, Vx['S'].mf, label='S')
    plt.plot(Vx.universe, Vx['M'].mf, label='M')
    plt.plot(Vx.universe, Vx['F'].mf, label='F')
    plt.title('Vx Membership Functions')
    plt.legend()
    plt.show()

    # 畫出 Vy 的隸屬函數
    plt.figure()
    plt.plot(Vy.universe, Vy['L'].mf, label='L')
    plt.plot(Vy.universe, Vy['Z'].mf, label='Z')
    plt.plot(Vy.universe, Vy['R'].mf, label='R')
    plt.title('Vy Membership Functions')
    plt.legend()
    plt.show()

    # 畫出 Omega 的隸屬函數
    plt.figure()
    plt.plot(omega.universe, omega['CCW'].mf, label='CCW')
    plt.plot(omega.universe, omega['Z'].mf, label='Z')
    plt.plot(omega.universe, omega['CW'].mf, label='CW')
    plt.title('Omega Membership Functions')
    plt.legend()
    plt.show()

    simulator_position = ctrl.ControlSystemSimulation(control_system_position)  # 正確
    simulator_angle = ctrl.ControlSystemSimulation(control_system_angle)  # 正確


    position_values = np.arange(-10, 11, 1)
    Vx_values, Vy_values, omega_values = [], [], []

    for p in position_values:
        simulator_position.input['position'] = p
        simulator_position.compute()
        Vx_values.append(simulator_position.output['Vx'])
        Vy_values.append(simulator_position.output['Vy'])
        omega_values.append(simulator_position.output['omega'])

    plt.figure()
    plt.plot(position_values, Vx_values, label='Vx')
    plt.plot(position_values, Vy_values, label='Vy')
    plt.plot(position_values, omega_values, label='Omega')
    plt.title("Fuzzy Control Output vs Position")
    plt.xlabel("Position Offset")
    plt.ylabel("Output Value")
    plt.legend()
    plt.grid()
    plt.show()


    # 掃描不同的角度輸入，繪製 Vx, Vy, omega 的變化曲線
    angle_values = np.arange(-30, 31, 1)
    Vx_values, Vy_values, omega_values = [], [], []

    for a in angle_values:
        simulator_angle.input['angle'] = a
        simulator_angle.compute()
        Vx_values.append(simulator_angle.output['Vx'])
        Vy_values.append(simulator_angle.output['Vy'])
        omega_values.append(simulator_angle.output['omega'])

    plt.figure()
    plt.plot(angle_values, Vx_values, label='Vx')
    plt.plot(angle_values, Vy_values, label='Vy')
    plt.plot(angle_values, omega_values, label='Omega')
    plt.title("Fuzzy Control Output vs Angle")
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Output Value")
    plt.legend()
    plt.grid()
    plt.show()

    """
    # 10. 返回結果
    return V_FL, V_FR, V_RL, V_RR

# 測試函式
angle_input = -20  # 角度偏左 20°
position_input = -3  # 位置偏左 3
V_FL, V_FR, V_RL, V_RR = calculate_mecanum_wheel_speeds(angle_input, position_input)

print(f"Front Left Wheel Speed: {V_FL:.2f}")
print(f"Front Right Wheel Speed: {V_FR:.2f}")
print(f"Rear Left Wheel Speed: {V_RL:.2f}")
print(f"Rear Right Wheel Speed: {V_RR:.2f}")


