#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from circular_saw_controller.msg import CircularSawCmd, CircularSawState

import tkinter as tk
from tkinter import ttk


class CircularSawUI(object):
    def __init__(self):
        rospy.init_node("circular_saw_test_ui", anonymous=True)

        # ◆ Topic 設定
        self.cmd_topic = rospy.get_param("~cmd_topic", "/circular_saw_cmd")
        self.state_topic = rospy.get_param("~state_topic", "/circular_saw_state")

        self.pub = rospy.Publisher(self.cmd_topic, CircularSawCmd, queue_size=10)
        rospy.Subscriber(self.state_topic, CircularSawState, self.stateCallback)

        # ◆ Tkinter UI
        self.root = tk.Tk()
        self.root.title("Circular Saw Test UI")

        # ======== 控制指令變數 ========
        self.x_pos_var = tk.StringVar(value="0")
        self.z_pos_var = tk.StringVar(value="0")
        self.angle_var = tk.StringVar(value="0")

        self.x_speed_var = tk.StringVar(value="3000")
        self.z_speed_var = tk.StringVar(value="2000")
        self.saw_speed_var = tk.StringVar(value="0")

        self.status_var = tk.StringVar(value=f"Publish to: {self.cmd_topic}")

        # ======== 回報狀態變數 (State Topic) ========
        self.fb_x_var = tk.StringVar(value="---")
        self.fb_z_var = tk.StringVar(value="---")
        self.fb_angle_var = tk.StringVar(value="---")
        self.fb_en1_var = tk.StringVar(value="---")
        self.fb_en2_var = tk.StringVar(value="---")
        self.fb_en3_var = tk.StringVar(value="---")
        self.fb_voltage_var = tk.StringVar(value="---")
        self.fb_frame_cnt = tk.StringVar(value="---")
        self.fb_saw_speed_var = tk.StringVar(value="---")

        # 建立 UI
        self.build_layout()

        # 定期執行 ros spin（避免 UI 卡住）
        self.update_ros()

        self.root.mainloop()

    # UI 元件
    def build_layout(self):
        pad = 5

        # 位置設定
        frame_pos = ttk.LabelFrame(self.root, text="位置設定 (mm)  ※X越大=往前, Z越大=往下")
        frame_pos.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(frame_pos, text="X (forward+) 0 ~ 400：").grid(row=0, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_pos, textvariable=self.x_pos_var, width=10).grid(row=0, column=1)

        ttk.Label(frame_pos, text="Z (down+) 0 ~ 350：").grid(row=1, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_pos, textvariable=self.z_pos_var, width=10).grid(row=1, column=1)

        # 鋸片角度 / 轉速
        frame_angle = ttk.LabelFrame(self.root, text="鋸片角度 / 轉速")
        frame_angle.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(frame_angle, text="角度 (-90 ~ 90)：").grid(row=0, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_angle, textvariable=self.angle_var, width=10).grid(row=0, column=1)

        ttk.Label(frame_angle, text="鋸片轉速 (0 ~ 6000 RPM)：").grid(row=1, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_angle, textvariable=self.saw_speed_var, width=10).grid(row=1, column=1)

        # 速度設定
        frame_speed = ttk.LabelFrame(self.root, text="各軸移動速度")
        frame_speed.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(frame_speed, text="X 軸速度 (500~5000)：").grid(row=0, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_speed, textvariable=self.x_speed_var, width=10).grid(row=0, column=1)

        ttk.Label(frame_speed, text="Z 軸速度 (500~5000)：").grid(row=1, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_speed, textvariable=self.z_speed_var, width=10).grid(row=1, column=1)

        # 控制按鈕
        frame_btn = ttk.Frame(self.root)
        frame_btn.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Button(frame_btn, text="送出指令 (RUN)", command=self.send_run).grid(row=0, column=0, padx=10, pady=5)
        ttk.Button(frame_btn, text="停止 (STOP)", command=self.send_stop).grid(row=0, column=1, padx=10, pady=5)

        ttk.Label(self.root, textvariable=self.status_var).grid(row=4, column=0, padx=10, pady=5, sticky="w")

        # 下位機狀態回報區塊
        frame_fb = ttk.LabelFrame(self.root, text="下位機回報狀態（Feedback）")
        frame_fb.grid(row=0, column=1, rowspan=5, padx=10, pady=10, sticky="nsew")

        labels = [
            ("X 現在位置 (mm)：", self.fb_x_var),
            ("Z 現在位置 (mm)：", self.fb_z_var),
            ("鋸片角度 (deg)：", self.fb_angle_var),
            ("鋸片轉速 (RPM)：", self.fb_saw_speed_var),
            ("EN1：", self.fb_en1_var),
            ("EN2：", self.fb_en2_var),
            ("EN3：", self.fb_en3_var),
            ("電壓 (V)：", self.fb_voltage_var),
            ("Frame Count：", self.fb_frame_cnt),
        ]

        for i, (text, var) in enumerate(labels):
            ttk.Label(frame_fb, text=text).grid(row=i, column=0, padx=pad, pady=pad, sticky="e")
            ttk.Label(frame_fb, textvariable=var, foreground="blue").grid(row=i, column=1, padx=pad, pady=pad, sticky="w")

    # ROS Subscriber Callback —— 更新 UI 顯示
    def stateCallback(self, msg: CircularSawState):
        self.fb_x_var.set(str(msg.x_pos))
        self.fb_z_var.set(str(msg.z_pos))
        self.fb_angle_var.set(f"{msg.angle:.2f}")
        self.fb_saw_speed_var.set(str(msg.saw_speed))
        self.fb_en1_var.set("ON" if msg.en1 else "OFF")
        self.fb_en2_var.set("ON" if msg.en2 else "OFF")
        self.fb_en3_var.set("ON" if msg.en3 else "OFF")
        self.fb_voltage_var.set(f"{msg.voltage:.2f}")
        self.fb_frame_cnt.set(str(msg.frame_count))

    # RUN 與 STOP 指令
    def send_run(self):
        msg = CircularSawCmd()

        try:
            x_pos   = float(self.x_pos_var.get())
            z_pos   = float(self.z_pos_var.get())
            angle   = float(self.angle_var.get())
            x_speed = float(self.x_speed_var.get())
            z_speed = float(self.z_speed_var.get())
            saw_spd = float(self.saw_speed_var.get())
        except ValueError:
            self.status_var.set("輸入格式錯誤，請確認都是數字")
            return

        # clamp：以 ROS 語意 (X forward+, Z down+)
        x_pos = max(0, min(400, x_pos))
        z_pos = max(0, min(350, z_pos))
        angle = max(-90, min(90, angle))
        x_speed = max(500, min(5000, x_speed))
        z_speed = max(500, min(5000, z_speed))
        saw_spd = max(0, min(6000, saw_spd))

        msg.x_pos = int(x_pos)
        msg.z_pos = int(z_pos)
        msg.angle = float(angle)
        msg.x_speed = int(x_speed)
        msg.z_speed = int(z_speed)
        msg.saw_speed = int(saw_spd)
        msg.stop = False

        self.pub.publish(msg)
        self.status_var.set(
            f"RUN: X={msg.x_pos}, Z={msg.z_pos}, angle={msg.angle}, "
            f"Vx={msg.x_speed}, Vz={msg.z_speed}, Saw={msg.saw_speed}"
        )

    def send_stop(self):
        msg = CircularSawCmd()
        msg.stop = True
        self.pub.publish(msg)
        self.status_var.set("已送出 STOP 指令（driver 會鎖當前位置）")

    # UI / ROS 更新
    def update_ros(self):
        if not rospy.is_shutdown():
            rospy.sleep(0.001)
            self.root.after(50, self.update_ros)  # 20Hz 更新 UI 與 ROS
        else:
            self.root.destroy()


if __name__ == "__main__":
    CircularSawUI()
