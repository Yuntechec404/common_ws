#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk

class RobotController:
    def __init__(self):
        rospy.init_node('car_test', anonymous=True)

        # 初始化速度參數
        self.robot_speed_factor = 1.0  # 機器人速度倍率 (0.1x ~ 1.0x)
        # 最低測試參數
        self.min_forward = 0.05  # 最低前進速度
        self.min_rotate = 0.1   # 最低自轉速度

        # /cmd_vel 控制機器人移動
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 建立主視窗
        self.window = tk.Tk()
        self.window.title("Robot Control Panel")
        self.window.geometry("500x500")

        # 3x3 移動按鈕
        button_config = [
            ("\u2196", 1, 2), ("\u2191", 1, 0), ("\u2197", 1, -2),
            ("\u21ba", 0, 2), ("Stop", 0, 0), ("\u21bb", 0, -2),
            ("\u2199", -1, -2), ("\u2193", -1, 0), ("\u2198", -1, 2)
        ]
        for i, (label, x, z) in enumerate(button_config):
            btn = tk.Button(self.window, text=label, font=("Arial", 14), width=5, height=2,
                            command=lambda x=x, z=z: self.send_command(x, z))
            btn.grid(row=i // 3, column=i % 3, padx=5, pady=5)

        row_offset = 4
        # 機器人速度滑桿 (0.1x ~ 1.0x) + 鍵盤輸入
        tk.Label(self.window, text="Robot Speed", font=("Arial", 12)).grid(row=row_offset, column=0, sticky="w", padx=5)
        self.robot_speed_label = tk.Label(self.window, text=f"{self.robot_speed_factor:.2f}x", font=("Arial", 12))
        self.robot_speed_label.grid(row=row_offset, column=2, sticky="e", padx=5)
        self.robot_speed_slider = ttk.Scale(self.window, from_=0.1, to=1.0, orient="horizontal",
                                            command=self.update_robot_speed)
        self.robot_speed_slider.set(self.robot_speed_factor)
        self.robot_speed_slider.grid(row=row_offset+1, column=0, columnspan=3, pady=5, sticky="ew")
        # Entry for keyboard輸入
        self.robot_speed_entry = tk.Entry(self.window, width=5)
        self.robot_speed_entry.insert(0, f"{self.robot_speed_factor:.2f}")
        self.robot_speed_entry.grid(row=row_offset+1, column=3)
        self.robot_speed_entry.bind('<Return>', self.entry_robot_speed)

        # 最低前進速度測試滑桿 + 鍵盤輸入
        tk.Label(self.window, text="Min Forward Speed", font=("Arial", 12)).grid(row=row_offset+2, column=0, sticky="w", padx=5)
        self.min_forward_label = tk.Label(self.window, text=f"{self.min_forward:.2f}", font=("Arial", 12))
        self.min_forward_label.grid(row=row_offset+2, column=2, sticky="e", padx=5)
        self.min_forward_slider = ttk.Scale(self.window, from_=0.0, to=1.0, orient="horizontal",
                                            command=self.update_min_forward)
        self.min_forward_slider.set(self.min_forward)
        self.min_forward_slider.grid(row=row_offset+3, column=0, columnspan=3, pady=5, sticky="ew")
        # Entry for keyboard輸入
        self.min_forward_entry = tk.Entry(self.window, width=5)
        self.min_forward_entry.insert(0, f"{self.min_forward:.2f}")
        self.min_forward_entry.grid(row=row_offset+3, column=3)
        self.min_forward_entry.bind('<Return>', self.entry_min_forward)

        # 最低自轉速度測試滑桿 + 鍵盤輸入
        tk.Label(self.window, text="Min Rotate Speed", font=("Arial", 12)).grid(row=row_offset+4, column=0, sticky="w", padx=5)
        self.min_rotate_label = tk.Label(self.window, text=f"{self.min_rotate:.2f}", font=("Arial", 12))
        self.min_rotate_label.grid(row=row_offset+4, column=2, sticky="e", padx=5)
        self.min_rotate_slider = ttk.Scale(self.window, from_=0.0, to=1.0, orient="horizontal",
                                            command=self.update_min_rotate)
        self.min_rotate_slider.set(self.min_rotate)
        self.min_rotate_slider.grid(row=row_offset+5, column=0, columnspan=3, pady=5, sticky="ew")
        # Entry for keyboard輸入
        self.min_rotate_entry = tk.Entry(self.window, width=5)
        self.min_rotate_entry.insert(0, f"{self.min_rotate:.2f}")
        self.min_rotate_entry.grid(row=row_offset+5, column=3)
        self.min_rotate_entry.bind('<Return>', self.entry_min_rotate)

        # 測試最低參數按鈕
        test_btn = tk.Button(self.window, text="Test Min Values", font=("Arial", 12),
                             command=self.test_min_values)
        test_btn.grid(row=row_offset+6, column=0, columnspan=4, pady=10)

        self.window.mainloop()

    def send_command(self, dir_x, dir_z):
        """ 發送機器人移動指令 """
        twist = Twist()
        twist.linear.x = 0.25 * self.robot_speed_factor * dir_x
        twist.angular.z = 0.25 * self.robot_speed_factor * dir_z
        self.pub_cmd_vel.publish(twist)

    def update_robot_speed(self, value):
        """ 更新機器人移動速度倍率 (滑桿) """
        self.robot_speed_factor = float(value)
        self.robot_speed_label.config(text=f"{self.robot_speed_factor:.2f}x")
        self.robot_speed_entry.delete(0, tk.END)
        self.robot_speed_entry.insert(0, f"{self.robot_speed_factor:.2f}")
        rospy.loginfo(f"Robot Speed Factor: {self.robot_speed_factor:.2f}x")

    def entry_robot_speed(self, event):
        """ 更新機器人移動速度倍率 (鍵盤) """
        try:
            val = float(self.robot_speed_entry.get())
            self.robot_speed_factor = max(0.1, min(val, 1.0))
            self.robot_speed_slider.set(self.robot_speed_factor)
            self.robot_speed_label.config(text=f"{self.robot_speed_factor:.2f}x")
            rospy.loginfo(f"Robot Speed Factor (entry): {self.robot_speed_factor:.2f}x")
        except ValueError:
            pass

    def update_min_forward(self, value):
        """ 更新最低前進速度參數 (滑桿) """
        self.min_forward = float(value)
        self.min_forward_label.config(text=f"{self.min_forward:.2f}")
        self.min_forward_entry.delete(0, tk.END)
        self.min_forward_entry.insert(0, f"{self.min_forward:.2f}")
        rospy.loginfo(f"Min Forward Speed: {self.min_forward:.2f}")

    def entry_min_forward(self, event):
        """ 更新最低前進速度參數 (鍵盤) """
        try:
            val = float(self.min_forward_entry.get())
            self.min_forward = max(0.0, min(val, 1.0))
            self.min_forward_slider.set(self.min_forward)
            self.min_forward_label.config(text=f"{self.min_forward:.2f}")
            rospy.loginfo(f"Min Forward Speed (entry): {self.min_forward:.2f}")
        except ValueError:
            pass

    def update_min_rotate(self, value):
        """ 更新最低自轉速度參數 (滑桿) """
        self.min_rotate = float(value)
        self.min_rotate_label.config(text=f"{self.min_rotate:.2f}")
        self.min_rotate_entry.delete(0, tk.END)
        self.min_rotate_entry.insert(0, f"{self.min_rotate:.2f}")
        rospy.loginfo(f"Min Rotate Speed: {self.min_rotate:.2f}")

    def entry_min_rotate(self, event):
        """ 更新最低自轉速度參數 (鍵盤) """
        try:
            val = float(self.min_rotate_entry.get())
            self.min_rotate = max(0.0, min(val, 1.0))
            self.min_rotate_slider.set(self.min_rotate)
            self.min_rotate_label.config(text=f"{self.min_rotate:.2f}")
            rospy.loginfo(f"Min Rotate Speed (entry): {self.min_rotate:.2f}")
        except ValueError:
            pass

    def test_min_values(self):
        """ 測試並發送最低前進與自轉指令 """
        twist = Twist()
        # 只前進
        rospy.loginfo(f"Testing Min Forward {self.min_forward}")
        twist.linear.x = self.min_forward
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(1.0)
        # 原地自轉
        rospy.loginfo(f"Testing Min Rotate {self.min_rotate}")
        twist.linear.x = 0.0
        twist.angular.z = self.min_rotate
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(1.0)
        # 停止
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
