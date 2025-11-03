#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import tkinter as tk
from tkinter import ttk, messagebox
import rospy
from cut_pliers_controller.msg import CmdCutPliers

NODE_NAME = "cut_pliers_ui"
CMD_TOPIC = "/cmd_cut_pliers_1"
STATUS_TOPIC = "/arm_current_status"

H_MIN, H_MAX = 0, 280
L_MIN, L_MAX = 10, 440

class CutPliersUI:
    def __init__(self, root, pub, ready_event):
        self.root = root
        root.title("Cut Pliers Controller UI (Absolute Move)")

        self.pub = pub
        self.ready = ready_event

        # 目前狀態（由 /arm_current_status 回來）
        self.current_height = 0
        self.current_length = 0
        self.current_claw   = False

        # 目標（輸入框）
        self.target_height = tk.IntVar(value=0)
        self.target_length = tk.IntVar(value=10)

        frm = ttk.Frame(root, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        # === 顯示目前 & 目標 ===
        ttk.Label(frm, text="Current Height (mm):", font=("Arial", 11)).grid(row=0, column=0, sticky="w", padx=2, pady=2)
        self.lbl_h_cur = ttk.Label(frm, text="0", font=("Arial", 12, "bold"))
        self.lbl_h_cur.grid(row=0, column=1, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Target Height (mm):",  font=("Arial", 11)).grid(row=0, column=2, sticky="e", padx=2, pady=2)
        self.ent_h = ttk.Entry(frm, textvariable=self.target_height, width=8, justify="right")
        self.ent_h.grid(row=0, column=3, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Current Length (mm):", font=("Arial", 11)).grid(row=1, column=0, sticky="w", padx=2, pady=2)
        self.lbl_l_cur = ttk.Label(frm, text="10", font=("Arial", 12, "bold"))
        self.lbl_l_cur.grid(row=1, column=1, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Target Length (mm):",  font=("Arial", 11)).grid(row=1, column=2, sticky="e", padx=2, pady=2)
        self.ent_l = ttk.Entry(frm, textvariable=self.target_length, width=8, justify="right")
        self.ent_l.grid(row=1, column=3, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Claw:", font=("Arial", 11)).grid(row=2, column=0, sticky="w", padx=2, pady=2)
        self.lbl_claw = ttk.Label(frm, text="CLOSED", font=("Arial", 12, "bold"))
        self.lbl_claw.grid(row=2, column=1, sticky="w", padx=2, pady=2)

        self.allow_retract = tk.BooleanVar(value=False)
        ttk.Checkbutton(frm, text="Allow Retract（允許後退）", variable=self.allow_retract).grid(row=2, column=2, columnspan=2, sticky="w", padx=2, pady=2)

        # === 操作按鈕 ===
        btn_row = 3
        ttk.Button(frm, text="Send HEIGHT (ABS)", command=self.send_height).grid(row=btn_row, column=0, columnspan=2, sticky="ew", padx=2, pady=8)
        ttk.Button(frm, text="Send LENGTH (ABS)", command=self.send_length).grid(row=btn_row, column=2, columnspan=2, sticky="ew", padx=2, pady=8)

        btn_row += 1
        ttk.Button(frm, text="Open Claw",  command=lambda: self.send_claw(False)).grid(row=btn_row, column=0, sticky="ew", padx=2, pady=8)
        ttk.Button(frm, text="Close Claw", command=lambda: self.send_claw(True)).grid(row=btn_row, column=1, sticky="ew", padx=2, pady=8)
        ttk.Button(frm, text="Set Target = Current", command=self.copy_current_to_target).grid(row=btn_row, column=2, columnspan=2, sticky="ew", padx=2, pady=8)

        # 提示
        tip = ("※ 本介面為『絕對位置』控制：\n"
               "  • 高度/前伸長度會直接設成目標值\n"
               "  • 若長度變小且未勾選 Allow Retract，指令會被取消")
        ttk.Label(frm, text=tip, foreground="#666").grid(row=btn_row+1, column=0, columnspan=4, sticky="w", padx=2, pady=4)

        # 每 100ms 刷新 UI
        self.root.after(100, self._refresh_labels)

        # Enter 快捷：在高度框按 Enter 送高度、在長度框按 Enter 送長度
        self.ent_h.bind("<Return>", lambda _e: self.send_height())
        self.ent_l.bind("<Return>", lambda _e: self.send_length())

    # ---- 內部：發訊共用 ----
    def _publish(self, *, height=None, length=None, claw=None):
        if not self.ready.is_set():
            rospy.logwarn_throttle(1.0, "ROS 尚未初始化完成，略過 publish")
            return

        msg = CmdCutPliers()

        # 絕對高度
        if height is not None:
            h = int(max(H_MIN, min(H_MAX, int(height))))
            msg.height1 = h
        else:
            msg.height1 = -1

        # 絕對長度（含後退檢查）
        if length is not None:
            l = int(max(L_MIN, min(L_MAX, int(length))))
            msg.length1 = l

            # 模式設定（你的下位機若不需要可忽略，但保留語意）
            if l >= self.current_length:
                msg.mode = 0  # forward
            else:
                msg.mode = 1  # reverse
                if not self.allow_retract.get():
                    rospy.logwarn("UI: 未勾選 Allow Retract，拒絕後退到 %d mm（目前 %d mm）", l, self.current_length)
                    messagebox.showwarning("Blocked", f"未勾選 Allow Retract，拒絕後退到 {l} mm（目前 {self.current_length} mm）")
                    return
        else:
            msg.length1 = -1

        if claw is not None:
            msg.claw1 = bool(claw)

        self.pub.publish(msg)

    # ---- 事件：送出 ----
    def send_height(self):
        try:
            self._publish(height=int(self.ent_h.get()))
        except ValueError:
            messagebox.showerror("Invalid", "Height 必須是整數")

    def send_length(self):
        try:
            self._publish(length=int(self.ent_l.get()))
        except ValueError:
            messagebox.showerror("Invalid", "Length 必須是整數")

    def send_claw(self, state):
        self._publish(claw=state)

    def copy_current_to_target(self):
        self.target_height.set(int(self.current_height))
        self.target_length.set(int(self.current_length))

    # ---- UI 刷新 ----
    def _refresh_labels(self):
        self.lbl_h_cur.config(text=str(int(self.current_height)))
        self.lbl_l_cur.config(text=str(int(self.current_length)))
        self.lbl_claw.config(text=("OPEN" if self.current_claw else "CLOSED"))
        self.root.after(100, self._refresh_labels)

# ROS 訂閱執行緒
def ros_sub_thread(ui: CutPliersUI):
    def _status_cb(msg: CmdCutPliers):
        # 依你的 /arm_current_status 內容填入
        ui.current_height = msg.height1
        ui.current_length = msg.length1
        ui.current_claw   = bool(getattr(msg, "claw1", False))

        # 一開始收到狀態就把 target 預設成 current，比較直覺
        if not hasattr(ui, "_synced"):
            ui._synced = True
            ui.target_height.set(int(ui.current_height))
            ui.target_length.set(int(ui.current_length))

    rospy.Subscriber(STATUS_TOPIC, CmdCutPliers, _status_cb)
    rospy.spin()

if __name__ == "__main__":
    # 在主執行緒初始化 ROS，避免 Tk/Signal 衝突
    rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)

    pub = rospy.Publisher(CMD_TOPIC, CmdCutPliers, queue_size=10)

    ready_event = threading.Event()
    ready_event.set()

    root = tk.Tk()
    ui = CutPliersUI(root, pub, ready_event)

    t = threading.Thread(target=ros_sub_thread, args=(ui,), daemon=True)
    t.start()

    root.mainloop()
