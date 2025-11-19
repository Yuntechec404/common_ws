#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from circular_saw_controller.msg import CircularSaw

import tkinter as tk
from tkinter import ttk


class CircularSawUI(object):
    def __init__(self):
        rospy.init_node("circular_saw_test_ui", anonymous=True)

        # 從參數讀取 topic（預設 /circular_saw_cmd）
        self.cmd_topic = rospy.get_param("~cmd_topic", "/circular_saw_cmd")
        self.pub = rospy.Publisher(self.cmd_topic, CircularSaw, queue_size=10)

        self.root = tk.Tk()
        self.root.title("Circular Saw Test UI")

        # 預設值：給一個明顯會動的目標
        self.x_pos_var = tk.StringVar(value="-200")  # X 往前 200mm
        self.z_pos_var = tk.StringVar(value="-150")  # Z 往下 150mm
        self.angle_var = tk.StringVar(value="0")     # 鋸片角度

        self.x_speed_var   = tk.StringVar(value="3000")
        self.z_speed_var   = tk.StringVar(value="2000")
        self.saw_speed_var = tk.StringVar(value="0")

        self.status_var = tk.StringVar(value=f"Publish to: {self.cmd_topic}")

        self.build_layout()
        self.update_ros()

        self.root.mainloop()

    def build_layout(self):
        pad = 5

        # 位置設定
        frame_pos = ttk.LabelFrame(self.root, text="位置設定 (mm)")
        frame_pos.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(
            frame_pos,
            text="X 軸長度 (mm)\n範圍: 0 ~ -400（往前請輸入負值）"
        ).grid(row=0, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_pos, textvariable=self.x_pos_var, width=10).grid(
            row=0, column=1, padx=pad, pady=pad
        )

        ttk.Label(
            frame_pos,
            text="Z 軸高度 (mm)\n範圍: 0 ~ -350（往下請輸入負值）"
        ).grid(row=1, column=0, padx=pad, pady=pad, sticky="e")
        ttk.Entry(frame_pos, textvariable=self.z_pos_var, width=10).grid(
            row=1, column=1, padx=pad, pady=pad
        )

        # 鋸片角度
        frame_angle = ttk.LabelFrame(self.root, text="鋸片角度 / 轉速")
        frame_angle.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(frame_angle, text="角度 (-90 ~ 90 deg)：").grid(
            row=0, column=0, padx=pad, pady=pad, sticky="e"
        )
        ttk.Entry(frame_angle, textvariable=self.angle_var, width=10).grid(
            row=0, column=1, padx=pad, pady=pad
        )

        ttk.Label(frame_angle, text="鋸片轉速 (0 ~ 6000 RPM)：").grid(
            row=1, column=0, padx=pad, pady=pad, sticky="e"
        )
        ttk.Entry(frame_angle, textvariable=self.saw_speed_var, width=10).grid(
            row=1, column=1, padx=pad, pady=pad
        )

        # 速度設定
        frame_speed = ttk.LabelFrame(self.root, text="各軸移動速度")
        frame_speed.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(frame_speed, text="X 軸速度 (500~5000)：").grid(
            row=0, column=0, padx=pad, pady=pad, sticky="e"
        )
        ttk.Entry(frame_speed, textvariable=self.x_speed_var, width=10).grid(
            row=0, column=1, padx=pad, pady=pad
        )

        ttk.Label(frame_speed, text="Z 軸速度 (500~5000)：").grid(
            row=1, column=0, padx=pad, pady=pad, sticky="e"
        )
        ttk.Entry(frame_speed, textvariable=self.z_speed_var, width=10).grid(
            row=1, column=1, padx=pad, pady=pad
        )

        # 按鈕列
        frame_btn = ttk.Frame(self.root)
        frame_btn.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Button(frame_btn, text="送出指令 (RUN)", command=self.send_run).grid(
            row=0, column=0, padx=10, pady=5
        )
        ttk.Button(frame_btn, text="停止 (STOP)", command=self.send_stop).grid(
            row=0, column=1, padx=10, pady=5
        )

        ttk.Label(self.root, textvariable=self.status_var).grid(
            row=4, column=0, padx=10, pady=5, sticky="w"
        )

    def send_run(self):
        msg = CircularSaw()

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

        # ----- clamp 到 STM32 支援範圍 -----
        # X : -400 ~ 0
        if x_pos > 0:
            x_pos = 0
        if x_pos < -400:
            x_pos = -400

        # Z : -350 ~ 0
        if z_pos > 0:
            z_pos = 0
        if z_pos < -350:
            z_pos = -350

        # angle : -90 ~ 90
        if angle > 90:
            angle = 90
        if angle < -90:
            angle = -90

        # speeds
        if x_speed < 500:
            x_speed = 500
        if x_speed > 5000:
            x_speed = 5000

        if z_speed < 500:
            z_speed = 500
        if z_speed > 5000:
            z_speed = 5000

        if saw_spd < 0:
            saw_spd = 0
        if saw_spd > 6000:
            saw_spd = 6000

        # ----- 填 msg -----
        msg.x_pos     = int(x_pos)
        msg.z_pos     = int(z_pos)
        msg.angle     = float(angle)
        msg.x_speed   = int(x_speed)
        msg.z_speed   = int(z_speed)
        msg.saw_speed = int(saw_spd)
        msg.stop      = False

        self.pub.publish(msg)
        self.status_var.set(
            f"RUN: X={msg.x_pos}mm, Z={msg.z_pos}mm, angle={msg.angle:.1f}deg, "
            f"Vx={msg.x_speed}, Vz={msg.z_speed}, Saw={msg.saw_speed}"
        )

    def send_stop(self):
        msg = CircularSaw()
        msg.stop = True
        self.pub.publish(msg)
        self.status_var.set("已送出 STOP 指令（停機）")

    def update_ros(self):
        if not rospy.is_shutdown():
            self.root.after(100, self.update_ros)
        else:
            self.root.destroy()


if __name__ == "__main__":
    CircularSawUI()
