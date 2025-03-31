import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

def calculate_mecanum_wheel_speeds(angle_input, position_input):
    #定義輸入變數
    angle = ctrl.Antecedent(np.arange(-30, 31, 1), 'angle')  # 角度偏差 (-30° ~ 30°)
    position = ctrl.Antecedent(np.arange(-10, 11, 1), 'position')  # 位置偏移 (-10 ~ 10)

    #定義輸出變數
    Vx = ctrl.Consequent(np.arange(0, 101, 1), 'Vx')
    Vy = ctrl.Consequent(np.arange(-50, 50, 1), 'Vy')
    omega = ctrl.Consequent(np.arange(-50, 51, 1), 'omega')

    #定義隸屬函數
    Vx = ctrl.Consequent(np.arange(0, 10, 0.1), 'Vx')
    Vy = ctrl.Consequent(np.arange(-10, 10, 0.1), 'Vy')
    omega = ctrl.Consequent(np.arange(-3, 3, 0.1), 'omega')

    #定義隸屬函數
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

    Vx['S'] = fuzz.trapmf(Vx.universe, [0, 0, 2.5, 5])          #慢速
    Vx['M'] = fuzz.trimf(Vx.universe, [2.5, 5, 7.5])                 #中速
    Vx['F'] = fuzz.trapmf(Vx.universe, [5, 7.5, 10, 10])             #快速

    Vy['LL'] = fuzz.trapmf(Vy.universe, [-10, -10, -7,-4])             #最左
    Vy['L'] = fuzz.trimf(Vy.universe, [-6, -3, 0])              #左
    Vy['Z'] = fuzz.trimf(Vy.universe, [-4, 0, 4])                 #中間
    Vy['R'] = fuzz.trimf(Vy.universe, [0, 3, 6])                 #右
    Vy['RR'] = fuzz.trapmf(Vy.universe, [4, 7, 10,10])                #最右


    omega['CCW2'] = fuzz.trapmf(omega.universe, [-3, -3, -2,-1])     #強烈逆時針
    omega['CCW'] = fuzz.trimf(omega.universe, [-2, -1, 0])      #輕微逆時針
    omega['Z'] = fuzz.trimf(omega.universe, [-1, 0, 1])           #中間
    omega['CW'] = fuzz.trimf(omega.universe, [0, 1, 2])          #輕微順時針
    omega['CW2'] = fuzz.trapmf(omega.universe, [1, 2, 3, 3])         #強烈順時

    #定義位置控制規則
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


    #建立模糊控制系統
    control_system = ctrl.ControlSystem(rules)
    simulator = ctrl.ControlSystemSimulation(control_system)
    
    # 設定輸入並計算
    simulator.input['angle'] = angle_input
    simulator.input['position'] = position_input
    simulator.compute()

    #加權合併輸出
    Vx_out = simulator.output['Vx']
    Vy_out = simulator.output['Vy']
    omega_out = simulator.output['omega']
    #計算麥克納姆輪速度
    V_FL = Vx_out - Vy_out - omega_out
    V_FR = Vx_out + Vy_out + omega_out
    V_RL = Vx_out + Vy_out - omega_out
    V_RR = Vx_out - Vy_out + omega_out

    print(f"Vx: {Vx_out}, Vy: {Vy_out}, Omega: {omega_out}")
    
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


