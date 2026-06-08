#!/usr/bin/env python3
"""
Minimal UDP test client for the MujocoSystemInterface UDP bridge.

Plays the role of the "real controller":
  - Receives RobotStatePacket (joint states + IMU) from the simulation.
  - Sends JointCommandPacket (PD + feedforward) back to the simulation.

Usage:
  python3 test_udp_client.py [--sim-ip 127.0.0.1] [--state-port 9870] [--cmd-port 9871]
"""

import argparse
import socket
import struct
import time

# ── Packet layout (must match udp_bridge.hpp) ─────────────────────────────────
MAX_JOINTS = 32

# RobotStatePacket
# uint64 timestamp_ns, uint32 num_joints, uint32 _pad,
# float[32] joint_pos, float[32] joint_vel, float[32] joint_torque,
# float[4] imu_quat, float[3] imu_gyro, float[3] imu_accel
STATE_FMT  = f"=QII {MAX_JOINTS}f {MAX_JOINTS}f {MAX_JOINTS}f 4f 3f 3f"
STATE_SIZE = struct.calcsize(STATE_FMT)

# JointCommandPacket
# uint64 timestamp_ns, uint32 num_joints, uint32 _pad,
# float[32] pos_des, float[32] vel_des, float[32] torque_ff, float[32] kp, float[32] kd
CMD_FMT  = f"=QII {MAX_JOINTS}f {MAX_JOINTS}f {MAX_JOINTS}f {MAX_JOINTS}f {MAX_JOINTS}f"
CMD_SIZE = struct.calcsize(CMD_FMT)

JOINT_NAMES = [
    "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
    "left_knee_pitch_joint", "left_ankle_roll_joint","left_ankle_pitch_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_pitch_joint","right_ankle_roll_joint","right_ankle_pitch_joint",
]
NUM_JOINTS = len(JOINT_NAMES)

def parse_state(data: bytes) -> dict:
    fields = struct.unpack(STATE_FMT, data)
    idx = 0
    ts         = fields[idx]; idx += 1
    n          = fields[idx]; idx += 1
    _pad       = fields[idx]; idx += 1
    pos        = fields[idx:idx+MAX_JOINTS]; idx += MAX_JOINTS
    vel        = fields[idx:idx+MAX_JOINTS]; idx += MAX_JOINTS
    torque     = fields[idx:idx+MAX_JOINTS]; idx += MAX_JOINTS
    imu_quat   = fields[idx:idx+4];          idx += 4
    imu_gyro   = fields[idx:idx+3];          idx += 3
    imu_accel  = fields[idx:idx+3]
    return {
        "timestamp_ns": ts,
        "num_joints": n,
        "pos":    list(pos[:n]),
        "vel":    list(vel[:n]),
        "torque": list(torque[:n]),
        "imu_quat":  list(imu_quat),
        "imu_gyro":  list(imu_gyro),
        "imu_accel": list(imu_accel),
    }

def build_command(num_joints: int,
                  pos_des:   list,
                  vel_des:   list,
                  torque_ff: list,
                  kp: list,
                  kd: list) -> bytes:
    def pad(lst): return lst + [0.0] * (MAX_JOINTS - len(lst))
    ts = int(time.time_ns())
    return struct.pack(
        CMD_FMT,
        ts, num_joints, 0,
        *pad(pos_des), *pad(vel_des), *pad(torque_ff), *pad(kp), *pad(kd),
    )

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-ip",     default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=9870)
    parser.add_argument("--cmd-port",   type=int, default=9871)
    args = parser.parse_args()

    # Recv socket — receives state packets FROM the simulation
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("0.0.0.0", args.state_port))
    recv_sock.settimeout(2.0)
    print(f"Listening for state packets on :{args.state_port}")

    # Send socket — sends command packets TO the simulation
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_addr = (args.sim_ip, args.cmd_port)
    print(f"Sending commands to {args.sim_ip}:{args.cmd_port}")
    print(f"Expected state packet size: {STATE_SIZE} bytes")
    print(f"Expected cmd packet size:   {CMD_SIZE} bytes\n")

    # Default safe command: hold current position with moderate gains, zero torque
    # These will be overwritten each cycle with the received current position
    pos_des   = [0.0] * NUM_JOINTS
    vel_des   = [0.0] * NUM_JOINTS
    torque_ff = [0.0] * NUM_JOINTS
    kp        = [50.0] * NUM_JOINTS   # Nm/rad
    kd        = [2.0]  * NUM_JOINTS   # Nm·s/rad

    recv_count = 0
    try:
        while True:
            # ── Receive state ──────────────────────────────────────────────
            try:
                data, addr = recv_sock.recvfrom(4096)
            except socket.timeout:
                print("[WARN] No state packet received — is the simulation running with use_udp:=true?")
                continue

            if len(data) != STATE_SIZE:
                print(f"[WARN] Unexpected packet size {len(data)}, expected {STATE_SIZE}")
                continue

            state = parse_state(data)
            recv_count += 1

            # ── Print every 100th packet ───────────────────────────────────
            if recv_count % 100 == 0:
                n = state["num_joints"]
                print(f"[{recv_count}] num_joints={n}")
                for i in range(min(n, NUM_JOINTS)):
                    print(f"  {JOINT_NAMES[i]:30s}  "
                          f"pos={state['pos'][i]:+.4f}  "
                          f"vel={state['vel'][i]:+.4f}  "
                          f"torque={state['torque'][i]:+.4f}")
                q = state["imu_quat"]
                g = state["imu_gyro"]
                a = state["imu_accel"]
                print(f"  IMU quat  w={q[0]:.3f} x={q[1]:.3f} y={q[2]:.3f} z={q[3]:.3f}")
                print(f"  IMU gyro  x={g[0]:.3f} y={g[1]:.3f} z={g[2]:.3f}")
                print(f"  IMU accel x={a[0]:.3f} y={a[1]:.3f} z={a[2]:.3f}\n")

            # ── Hold-position command: set pos_des = current pos ───────────
            pos_des = list(state["pos"][:NUM_JOINTS])

            cmd = build_command(NUM_JOINTS, pos_des, vel_des, torque_ff, kp, kd)
            send_sock.sendto(cmd, send_addr)

    except KeyboardInterrupt:
        print(f"\nStopped after {recv_count} packets.")
    finally:
        recv_sock.close()
        send_sock.close()

if __name__ == "__main__":
    main()
