import pygame

# 初始化 pygame
pygame.init()

# 初始化搖桿
pygame.joystick.init()

# 檢查是否有搖桿連接
if pygame.joystick.get_count() == 0:
    print("未偵測到 PS5 搖桿，請插入 USB 或透過藍牙連線")
    exit()

# 取得搖桿
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"偵測到搖桿：{joystick.get_name()}\n")

# **PS5 DualSense 按鍵索引對應**
BUTTON_NAMES = {
    0: "Cross",      # 叉叉
    1: "Circle",     # 圈圈
    2: "Square",     # 方形
    3: "Triangle",   # 三角形
    4: "Share",      # Share
    5: "PS",         # PS
    6: "Options",    # Options
    7: "L3",         # L3
    8: "R3",         # R3
    9: "L1",         # L1
    10: "R1",        # R1
    11: "Up",        # 上
    12: "Down",      # 下
    13: "Left",      # 左
    14: "Right",     # 右
    15: "Touchpad",  # 觸控板按鍵
    16: "Button1",  # 新按鍵3
    17: "Button2",  # 新按鍵4
}

AXIS_NAMES = {
    0: "Left_X",   # 左搖桿 X
    1: "Left_Y",   # 左搖桿 Y
    2: "Right_X",  # 右搖桿 X
    3: "Right_Y",  # 右搖桿 Y
    4: "L2_Trigger",     # L2 扳機
    5: "R2_Trigger"      # R2 扳機
}

# 儲存每個按鈕的狀態為變數
def update_button_and_axis_states(button_states, axis_states):
    # 按鈕狀態
    button_state_variables = {}
    for i, name in BUTTON_NAMES.items():
        button_state_variables[name] = button_states.get(name, 0)

    # 搖桿狀態
    axis_state_variables = {}
    for i, name in AXIS_NAMES.items():
        axis_state_variables[name] = round(axis_states.get(name, 0), 2)

    return button_state_variables, axis_state_variables

# 清除畫面函數
def clear_screen():
    print("\033[H\033[J", end="")

# 顯示按鈕狀態函數
def print_button_states(button_states):
    print("**按鍵狀態**:")
    pressed_buttons = [name for name, state in button_states.items() if state]
    if pressed_buttons:
        for name in pressed_buttons:
            print(f"{name}")
    else:
        print("沒有按鍵被按下")

# 顯示搖桿狀態函數
def print_axis_states(axis_states):
    print("\n**搖桿狀態**:")
    for name, value in axis_states.items():
        print(f"  {name}: {value}")

# 主程式迴圈
running = True
while running:
    pygame.event.pump()  # 更新事件

    # 讀取所有按鍵和搖桿軸
    button_states = {BUTTON_NAMES[i]: joystick.get_button(i) for i in range(joystick.get_numbuttons())}
    axis_states = {AXIS_NAMES[i]: round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())}

    # 清除畫面
    clear_screen()

    # 輸出 PS5 搖桿狀態
    print("**PS5 搖桿狀態**")
    print("-" * 40)

    button_state_variables, axis_state_variables = update_button_and_axis_states(button_states, axis_states)

    Cross, Circle, Square, Triangle, Share, PS, Options, L3, R3, L1, R1, Up, Down, Left, Right, Touchpad, New_Button_3, New_Button_4 = \
            button_state_variables["Cross"], button_state_variables["Circle"], button_state_variables["Square"], button_state_variables["Triangle"], \
            button_state_variables["Share"], button_state_variables["PS"], button_state_variables["Options"], button_state_variables["L3"], \
            button_state_variables["R3"], button_state_variables["L1"], button_state_variables["R1"], button_state_variables["Up"], \
            button_state_variables["Down"], button_state_variables["Left"], button_state_variables["Right"], button_state_variables["Touchpad"], \
            button_state_variables["Button1"], button_state_variables["Button2"]
    Left_X, Left_Y, Right_X, Right_Y, L2_Trigger, R2_Trigger = \
        axis_state_variables["Left_X"], axis_state_variables["Left_Y"], axis_state_variables["Right_X"], axis_state_variables["Right_Y"], \
        axis_state_variables["L2_Trigger"], axis_state_variables["R2_Trigger"]

    # 輸出按鍵和搖桿狀態
    print_button_states(button_states)
    print_axis_states(axis_states)
    print()

    print(Cross, Circle, Square, Triangle)
    print(f"\nL X: {Left_X}, L Y: {Left_Y}")
    print(f"R X: {Right_X}, R Y: {Right_Y}")
    print(f"L2: {L2_Trigger}, R2: {R2_Trigger}")


    if Cross == 1:  # 如果 "叉叉" 按鍵被按下
        print("'叉叉' 按鍵被按下！")

    if Left_X > 0.3: 
        print("!!!!!!!!!")




    pygame.time.wait(100)
    # 退出條件
    if Share == 1:  # 如果 "Share" 按鈕被按下
        print("檢測到 'Share' 按鍵被按下，程式即將退出...")
        running = False
