"""
 * @file mujoco_simulation.py
 * @brief simulation in mujoco
 * @author mayuxuan
 * @version 1.0
 * @date 2025-05-08
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
"""

import os
import time
import socket
import struct
import threading
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer



MODEL_NAME = "lite3"                      
XML_PATH   = "../../../Lite3_description/lite3_mjcf/mjcf/Lite3.xml"           
LOCAL_PORT = 20001                         
CTRL_IP    = "127.0.0.1"                   
CTRL_PORT  = 30010
USE_VIEWER = True                         
DT         = 0.001                         


URDF_INIT = {
    "lite3": np.array([0, -1.35453, 2.54948] * 4, dtype=np.float32)
}

class MuJoCoSimulation:
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_relpath: str = XML_PATH,
                 local_port: int = LOCAL_PORT,
                 ctrl_ip: str = CTRL_IP,
                 ctrl_port: int = CTRL_PORT):

        # UDP 通信
        self.local_port = local_port
        self.ctrl_addr  = (ctrl_ip, ctrl_port)
        self.recv_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("0.0.0.0", local_port))
        self.send_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 加载 MJCF 
        xml_full = str(Path(__file__).resolve().parent / xml_relpath)
        if not os.path.isfile(xml_full):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_full}")

        self.model = mujoco.MjModel.from_xml_path(xml_full)
        self.data  = mujoco.MjData(self.model)

        # 机器人自由度列表
        self.actuator_ids = [a for a in range(self.model.nu)]           # 0..11
        self.dof          = len(self.actuator_ids)

        # 初始化站立姿态
        self._set_initial_pose(model_key)

        # 缓存
        self.kp_cmd   = np.zeros((self.dof, 1), np.float32)
        self.kd_cmd   = np.zeros_like(self.kp_cmd)
        self.pos_cmd  = np.zeros_like(self.kp_cmd)
        self.vel_cmd  = np.zeros_like(self.kp_cmd)
        self.tau_ff   = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0

        print(f"[INFO] MuJoCo model loaded, dof = {self.dof}")

        # 可视化
        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            # self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

    # --------------------------------------------------------

    def _set_initial_pose(self, key: str):
        """关节位置设置为与 PyBullet 脚本一致的初始角度"""
        qpos0 = self.data.qpos.copy()
        qpos0[7:7+self.dof] = URDF_INIT[key]    # ,3-6 basequat，0-2 basepos
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)  

    # ---------------- 主循环 -----------------
    def start(self):
        # 接收线程
        threading.Thread(target=self._udp_receiver, daemon=True).start()
        print(f"[INFO] UDP receiver on 0.0.0.0:{self.local_port}")

        # 主模拟循环
        step = 0
        while True:
            t0 = time.perf_counter()

            # 控制律
            self._apply_joint_torque()

            # 模拟一步
            mujoco.mj_step(self.model, self.data)

            # 采样 & 发送观测
            self._send_robot_state(step)

            # 可视化
            if self.viewer:
                self.viewer.sync()

            # 以 1 kHz 尝试节流
            step += 1
            self.timestamp = step * DT
            dt = time.perf_counter() - t0
            if dt < DT:
                time.sleep(DT - dt)

    # --------------------------------------------------------

    def _udp_receiver(self):
        """
        12f kp | 12f pos | 12f kd | 12f vel | 12f tau  = 240 bytes
        """
        fmt = "12f"*5
        expected = struct.calcsize(fmt)   # 240
        while True:
            data, addr = self.recv_sock.recvfrom(expected)
            if len(data) < expected:
                print(f"[WARN] UDP packet size {len(data)} != {expected}")
                continue
            unpacked = struct.unpack(fmt, data)
            self.kp_cmd   = np.asarray(unpacked[0:12],  dtype=np.float32).reshape(-1,1)
            self.pos_cmd  = np.asarray(unpacked[12:24], dtype=np.float32).reshape(-1,1)
            self.kd_cmd   = np.asarray(unpacked[24:36], dtype=np.float32).reshape(-1,1)
            self.vel_cmd  = np.asarray(unpacked[36:48], dtype=np.float32).reshape(-1,1)
            self.tau_ff   = np.asarray(unpacked[48:60], dtype=np.float32).reshape(-1,1)
            # print(f"[UDP] cmd from {addr}")

    # --------------------------------------------------------

    def _apply_joint_torque(self):
        # 当前关节状态
        q   = self.data.qpos[7:7+self.dof].reshape(-1,1)
        dq  = self.data.qvel[6:6+self.dof].reshape(-1,1)

        # τ = kp*(q_d - q) + kd*(dq_d - dq) + τ_ff
        self.input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            self.kd_cmd * (self.vel_cmd - dq) +
            self.tau_ff
        )
        # 调试
        print("=== [Joint Command Debug] ===")
        print(f"[Target Pos]:\n{self.pos_cmd.T}")
        print(f"[Actual Pos]:\n{q.T}")
        print(f"[Target Vel]:\n{self.vel_cmd.T}")
        print(f"[Actual Vel]:\n{dq.T}")
        print(f"[Kp Term]:\n{self.kp_cmd.T}")
        print(f"[Kd Term]:\n{self.kd_cmd.T}")
        print(f"[Feedforward Tau]:\n{self.tau_ff.T}")
        print(f"[Final Torque Output]:\n{self.input_tq.T}")
        
        # 写入 control 缓冲区
        self.data.ctrl[:] = self.input_tq.flatten()

    # --------------------------------------------------------
    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q

        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)      # 防止数值漂移导致 |t2|>1
        pitch = np.arcsin(t2)

        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)
    
    
    
    # --------------------------------------------------------

    def _send_robot_state(self, step:int):
        # ----- IMU -----
        q_world   = self.data.qpos[3:7]             # quaternion
        # rpy       = mujoco.mju_mat2Euler(mujoco.mju_quat2Mat(q_world))
        rpy       = self.quaternion_to_euler(q_world)
  

        linvel    = self.data.qvel[3:6]             # world frame
        # angvel    = self.data.qvel[0:3]           # world frame
        angvel_b  = self.data.qvel[0:3]             # body frame
        
        # 简单差分加速度
        # print(f"[IMU] vel: {linvel.flatten()}")
        # print(f"[IMU] last vel: {self.last_base_linvel.flatten()}")
        
        # # acc_world = (linvel - self.last_base_linvel.flatten()) / DT
        # acc_world = (linvel - self.last_base_linvel.flatten()) / DT
        # self.last_base_linvel[:] = linvel.reshape(3,1)
        
        # # z 加速度处理
        # acc_world[2] += 9.81
        # print(f"[IMU] Acc_world: {acc_world.flatten()}")
        
        
        
        # 转到 body frame
        mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(mat, q_world.astype(np.float64))
        R = mat.reshape(3, 3)
        # body_acc  = R.T @ acc_world
        body_acc = self.data.sensordata[0:3]         # body frame
        body_acc[2] += 9.81                          # z 加速度处理
        # body_omega= R.T @ angvel
        angvel = R @ angvel_b                        # world frame

        # ----- 关节 -----
        q = self.data.qpos[7:7+self.dof]
        dq= self.data.qvel[6:6+self.dof]
        tau=self.input_tq.flatten()
        print(f"[IMU] tau: {tau}")
        
        # --- 调试打印 ---

        print(f"[IMU] RPY: {rpy.flatten()}")
        print(f"[IMU] Omega: {angvel_b.flatten()}")
        print(f"[IMU] Acc_body: {body_acc.flatten()}")

        # 打包并发送
        payload = np.concatenate((
            np.array([self.timestamp], dtype=np.float64),
            np.asarray(rpy,   dtype=np.float32),
            np.asarray(body_acc,  dtype=np.float32),
            np.asarray(angvel_b,dtype=np.float32),
            q.astype(np.float32),
            dq.astype(np.float32),
            tau.astype(np.float32)
        ))
        fmt = "1d" + f"{len(payload)-1}f"
        try:
            self.send_sock.sendto(struct.pack(fmt, *payload), self.ctrl_addr)
        except socket.error as ex:
            print(f"[UDP send] {ex}")

# ------------------ main ------------------
if __name__ == "__main__":
    sim = MuJoCoSimulation()
    sim.start()
