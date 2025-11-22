#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import tkinter as tk
from tkinter import ttk, messagebox
import rospy
from cut_pliers_controller.msg import CmdCutPliers

NODE_NAME = "cut_pliers_ui"

H_MIN, H_MAX = 0, 280
L_MIN, L_MAX = 10, 440


class CutPliersUI:
    def __init__(self, root, pub, ready_event,
                 arm_id: int,
                 status_topic: str,
                 height_field: str,
                 length_field: str,
                 claw_field: str):
        self.root = root
        root.title(f"Cut Pliers Controller UI (ABS) - Arm {arm_id}")

        self.pub = pub
        self.ready = ready_event

        # 這三個字串決定要用 height1/2, length1/2, claw1/2
        self.height_field = height_field
        self.length_field = length_field
        self.claw_field = claw_field

        # 目前狀態（由 /arm_current_status 回來）
        self.current_height = 0
        self.current_length = 10
        self.current_claw = False

        # 目標值（輸入框）
        self.target_height = tk.IntVar(value=0)
        self.target_length = tk.IntVar(value=10)

        frm = ttk.Frame(root, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        # === 顯示目前 & 目標 ===
        ttk.Label(frm, text="Current Height (mm):", font=("Arial", 11)).grid(
            row=0, column=0, sticky="w", padx=2, pady=2
        )
        self.lbl_h_cur = ttk.Label(frm, text="0", font=("Arial", 12, "bold"))
        self.lbl_h_cur.grid(row=0, column=1, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Target Height (mm):", font=("Arial", 11)).grid(
            row=0, column=2, sticky="e", padx=2, pady=2
        )
        self.ent_h = ttk.Entry(frm, textvariable=self.target_height, width=8, justify="right")
        self.ent_h.grid(row=0, column=3, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Current Length (mm):", font=("Arial", 11)).grid(
            row=1, column=0, sticky="w", padx=2, pady=2
        )
        self.lbl_l_cur = ttk.Label(frm, text="10", font=("Arial", 12, "bold"))
        self.lbl_l_cur.grid(row=1, column=1, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Target Length (mm):", font=("Arial", 11)).grid(
            row=1, column=2, sticky="e", padx=2, pady=2
        )
        self.ent_l = ttk.Entry(frm, textvariable=self.target_length, width=8, justify="right")
        self.ent_l.grid(row=1, column=3, sticky="w", padx=2, pady=2)

        ttk.Label(frm, text="Claw:", font=("Arial", 11)).grid(
            row=2, column=0, sticky="w", padx=2, pady=2
        )
        self.lbl_claw = ttk.Label(frm, text="CLOSED", font=("Arial", 12, "bold"))
        self.lbl_claw.grid(row=2, column=1, sticky="w", padx=2, pady=2)

        self.allow_retract = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            frm,
            text="Allow Retract（允許後退）",
            variable=self.allow_retract
        ).grid(row=2, column=2, columnspan=2, sticky="w", padx=2, pady=2)

        # === 操作按鈕 ===
        btn_row = 3
        ttk.Button(
            frm,
            text="Send HEIGHT & LENGTH (ABS)",
            command=self.send_height_length
        ).grid(
            row=btn_row, column=0, columnspan=4,
            sticky="ew", padx=2, pady=8
        )

        btn_row += 1
        ttk.Button(frm, text="Open Claw", command=lambda: self.send_claw(False)).grid(
            row=btn_row, column=0, sticky="ew", padx=2, pady=8
        )
        ttk.Button(frm, text="Close Claw", command=lambda: self.send_claw(True)).grid(
            row=btn_row, column=1, sticky="ew", padx=2, pady=8
        )
        ttk.Button(frm, text="Set Target = Current", command=self.copy_current_to_target).grid(
            row=btn_row, column=2, columnspan=2, sticky="ew", padx=2, pady=8
        )

        tip = (
            f"※ 本介面為 Arm {arm_id}『絕對位置』控制：\n"
            "  • 高度/前伸長度會直接設成目標值\n"
            "  • 若長度變小且未勾選 Allow Retract，指令會被取消"
        )
        ttk.Label(frm, text=tip, foreground="#666").grid(
            row=btn_row+1, column=0, columnspan=4, sticky="w", padx=2, pady=4
        )

        # 每 100ms 刷新 UI
        self.root.after(100, self._refresh_labels)

        # Enter 快捷：在任一輸入框按 Enter 都一起送高度+長度
        self.ent_h.bind("<Return>", lambda _e: self.send_height_length())
        self.ent_l.bind("<Return>", lambda _e: self.send_height_length())

    # ---- 內部：發訊共用 ----
    def _publish(self, *, height=None, length=None, claw=None):
        if not self.ready.is_set():
            rospy.logwarn_throttle(1.0, "ROS 尚未初始化完成，略過 publish")
            return

        msg = CmdCutPliers()

        # 高度
        if height is not None:
            h = int(max(H_MIN, min(H_MAX, int(height))))
            setattr(msg, self.height_field, h)   # height1 or height2
        else:
            setattr(msg, self.height_field, -1)  # -1 表示不上傳此軸（C++ 那邊可以選擇忽略）

        # 長度（含後退檢查）
        if length is not None:
            l = int(max(L_MIN, min(L_MAX, int(length))))
            setattr(msg, self.length_field, l)   # length1 or length2

            # mode for forward/reverse
            if l >= self.current_length:
                msg.mode = 0  # forward
            else:
                msg.mode = 1  # reverse
                if not self.allow_retract.get():
                    rospy.logwarn(
                        "UI: 未勾選 Allow Retract，拒絕後退到 %d mm（目前 %d mm）",
                        l, self.current_length
                    )
                    messagebox.showwarning(
                        "Blocked",
                        f"未勾選 Allow Retract，拒絕後退到 {l} mm（目前 {self.current_length} mm）"
                    )
                    return
        else:
            setattr(msg, self.length_field, -1)

        # 爪子：若沒特別指定，就沿用 current_claw，避免不小心關掉
        claw_val = self.current_claw if claw is None else bool(claw)
        setattr(msg, self.claw_field, claw_val)

        self.pub.publish(msg)

    # ---- 事件：送出 ----
    def send_height_length(self):
        """同時送高度 + 長度（絕對值）"""
        try:
            h = int(self.ent_h.get())
        except ValueError:
            messagebox.showerror("Invalid", "Height 必須是整數")
            return

        try:
            l = int(self.ent_l.get())
        except ValueError:
            messagebox.showerror("Invalid", "Length 必須是整數")
            return

        self._publish(height=h, length=l)

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


# ROS 訂閱執行緒：依 arm_id 選不同欄位
def ros_sub_thread(ui: CutPliersUI, status_topic: str):
    def _status_cb(msg: CmdCutPliers):
        h = getattr(msg, ui.height_field, 0)
        l = getattr(msg, ui.length_field, 10)
        claw = bool(getattr(msg, ui.claw_field, False))

        ui.current_height = h
        ui.current_length = l
        ui.current_claw = claw

        # 一開始收到狀態就把 target 預設成 current，比較直覺
        if not hasattr(ui, "_synced"):
            ui._synced = True
            ui.target_height.set(int(ui.current_height))
            ui.target_length.set(int(ui.current_length))

    rospy.Subscriber(status_topic, CmdCutPliers, _status_cb)
    rospy.spin()


if __name__ == "__main__":
    # 在主執行緒初始化 ROS，避免 Tk/Signal 衝突
    rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)

    # 從參數讀要控制哪一支手臂 & topics
    arm_id = int(rospy.get_param("~arm_id", 2))          # 預設控制 Arm2
    cmd_topic = rospy.get_param("~cmd_topic", "/cmd_cut_pliers_123")
    status_topic = rospy.get_param("~status_topic", "/arm_current_status")

    # 根據 arm_id 選欄位名稱
    if arm_id == 1:
        height_field = "height1"
        length_field = "length1"
        claw_field = "claw1"
    else:
        height_field = "height2"
        length_field = "length2"
        claw_field = "claw2"

    pub = rospy.Publisher(cmd_topic, CmdCutPliers, queue_size=10)

    ready_event = threading.Event()
    ready_event.set()

    root = tk.Tk()
    ui = CutPliersUI(
        root,
        pub,
        ready_event,
        arm_id=arm_id,
        status_topic=status_topic,
        height_field=height_field,
        length_field=length_field,
        claw_field=claw_field,
    )

    t = threading.Thread(target=ros_sub_thread, args=(ui, status_topic), daemon=True)
    t.start()

    root.mainloop()
