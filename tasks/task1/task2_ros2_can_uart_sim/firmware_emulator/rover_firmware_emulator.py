#!/usr/bin/env python3
import socket
import struct
import time
import math
import os
import sys
import pty
import select
import termios
import tty

# ==============================================================================
# 4WD ROVER MICROCONTROLLER FIRMWARE EMULATOR
# ==============================================================================

def calculate_xor_checksum(data_bytes: bytes) -> int:
    chk = 0
    for b in data_bytes[:7]:
        chk ^= b
    return chk

def calculate_nmea_crc(sentence: str) -> str:
    crc = 0
    for char in sentence:
        crc ^= ord(char)
    return f"{crc:02X}"

class RoverFirmwareEmulator:
    def __init__(self, serial_port="/tmp/ttyV1", can_interface="vcan0"):
        self.serial_port_path = serial_port
        self.can_interface = can_interface
        
        # 4WD Motor States
        self.target_vel = [0.0, 0.0, 0.0, 0.0]
        self.measured_pos = [0.0, 0.0, 0.0, 0.0]
        self.measured_vel = [0.0, 0.0, 0.0, 0.0]

        # IMU Physics State
        self.accel_x = 0.01
        self.accel_y = -0.02
        self.yaw = 0.0
        
        self.can_sock = None
        self.is_udp_can = False
        self.serial_fd = None
        self.master_pty = None

    def setup_can(self):
        try:
            self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_sock.bind((self.can_interface,))
            self.can_sock.setblocking(False)
            self.is_udp_can = False
            print(f"[Firmware Emulator] Bound to native CAN interface '{self.can_interface}'.")
            return
        except Exception as e:
            print(f"[Firmware Emulator] Native CAN interface '{self.can_interface}' not available ({e}).")

        # Fallback: Virtual UDP CAN Loopback socket (Port 9091)
        try:
            self.can_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.can_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.can_sock.bind(("127.0.0.1", 9091))
            self.can_sock.setblocking(False)
            self.is_udp_can = True
            print("[Firmware Emulator] Bound to Virtual UDP CAN fallback (127.0.0.1:9091).")
        except Exception as e:
            print(f"[Firmware Emulator] Error opening UDP CAN fallback: {e}")
            self.can_sock = None

    def setup_serial(self):
        if os.path.exists(self.serial_port_path):
            try:
                self.serial_fd = os.open(self.serial_port_path, os.O_RDWR | os.O_NONBLOCK)
                print(f"[Firmware Emulator] Opened serial port '{self.serial_port_path}'.")
                return
            except Exception as e:
                print(f"[Firmware Emulator] Error opening '{self.serial_port_path}': {e}")

        # Fallback PTY pair creation if /tmp/ttyV1 is missing
        print("[Firmware Emulator] Creating fallback internal PTY pair...")
        master, slave = pty.openpty()
        slave_name = os.ttyname(slave)
        self.master_pty = master
        self.serial_fd = master
        os.system(f"ln -sf {slave_name} /tmp/ttyV0 2>/dev/null")
        os.system(f"ln -sf {slave_name} /tmp/ttyV1 2>/dev/null")
        print(f"[Firmware Emulator] Linked fallback PTY to /tmp/ttyV0 and /tmp/ttyV1 ({slave_name}).")

    def process_can_rx(self):
        if not self.can_sock:
            return

        try:
            while True:
                data, _ = self.can_sock.recvfrom(16)
                if len(data) < 16:
                    break

                can_id, can_dlc, data_bytes = struct.unpack("=IB3x8s", data)
                
                # Check motor command IDs 0x101 - 0x104
                if 0x101 <= can_id <= 0x104:
                    motor_id = data_bytes[0]
                    if 1 <= motor_id <= 4:
                        expected_chk = calculate_xor_checksum(data_bytes)
                        actual_chk = data_bytes[7]
                        
                        if actual_chk == expected_chk:
                            target_v = struct.unpack("<f", data_bytes[1:5])[0]
                            self.target_vel[motor_id - 1] = target_v
        except BlockingIOError:
            pass
        except Exception:
            pass

    def send_can_encoder_tx(self):
        if not self.can_sock:
            return

        for m_idx in range(4):
            motor_id = m_idx + 1
            can_id = 0x200 + motor_id
            
            payload = bytearray(8)
            payload[0] = motor_id
            struct.pack_into("<f", payload, 1, self.measured_pos[m_idx])
            payload[5] = 0x00
            payload[6] = 0x00
            payload[7] = calculate_xor_checksum(payload)

            frame = struct.pack("=IB3x8s", can_id, 8, bytes(payload))
            try:
                if self.is_udp_can:
                    self.can_sock.sendto(frame, ("127.0.0.1", 9090))
                else:
                    self.can_sock.send(frame)
            except Exception:
                pass

    def send_uart_imu_tx(self):
        if self.serial_fd is None:
            return

        content = f"ROVER,IMU,{self.accel_x:.3f},{self.accel_y:.3f},{self.yaw:.4f}"
        crc = calculate_nmea_crc(content)
        sentence = f"${content}*{crc}\n"

        try:
            os.write(self.serial_fd, sentence.encode('ascii'))
        except Exception:
            pass

    def update_physics(self, dt):
        for i in range(4):
            alpha = 0.2
            self.measured_vel[i] += alpha * (self.target_vel[i] - self.measured_vel[i])
            self.measured_pos[i] += self.measured_vel[i] * dt

        self.accel_x = 0.05 * math.sin(time.time() * 2.0) + (self.target_vel[0] - self.measured_vel[0])
        self.accel_y = 0.02 * math.cos(time.time() * 1.5)
        self.yaw += (self.measured_vel[0] - self.measured_vel[1]) * 0.1 * dt
        self.yaw = math.fmod(self.yaw, 2.0 * math.pi)

    def run(self):
        print("==========================================================")
        print("   Rover Microcontroller Firmware Emulator (Task 2)       ")
        print("==========================================================")
        self.setup_can()
        self.setup_serial()
        print("[Firmware Emulator] Running 100 Hz telemetry & control loop...")

        dt = 0.01
        step = 0
        while True:
            t0 = time.time()
            
            self.process_can_rx()
            self.update_physics(dt)
            
            if step % 2 == 0:
                self.send_can_encoder_tx()
            
            self.send_uart_imu_tx()

            step += 1
            elapsed = time.time() - t0
            sleep_time = max(0.0, dt - elapsed)
            time.sleep(sleep_time)

if __name__ == "__main__":
    emulator = RoverFirmwareEmulator()
    try:
        emulator.run()
    except KeyboardInterrupt:
        print("\n[Firmware Emulator] Exiting.")
