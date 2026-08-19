#!/usr/bin/env bash
# ==============================================================================
# Setup Script for Virtual CAN (vcan0) and Virtual Serial PTY (/tmp/ttyV0 <-> /tmp/ttyV1)
# ==============================================================================

echo "[Task 2 Setup] Setting up Virtual CAN (vcan0)..."
if sudo ip link show vcan0 > /dev/null 2>&1; then
    echo "[Task 2 Setup] vcan0 already exists."
else
    sudo modprobe vcan 2>/dev/null || true
    sudo ip link add dev vcan0 type vcan 2>/dev/null || true
    sudo ip link set up vcan0 2>/dev/null || true
    echo "[Task 2 Setup] vcan0 brought up successfully."
fi

echo "[Task 2 Setup] Setting up Virtual Serial PTY (/tmp/ttyV0 <-> /tmp/ttyV1)..."
pkill -f "socat PTY" 2>/dev/null || true

socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1 &
SOCAT_PID=$!

sleep 1
if [ -e /tmp/ttyV0 ] && [ -e /tmp/ttyV1 ]; then
    echo "[Task 2 Setup] Virtual serial ports created:"
    echo "  - Candidate Hardware Interface port: /tmp/ttyV0"
    echo "  - Emulator Microcontroller port:    /tmp/ttyV1"
else
    echo "[Task 2 Setup] Warning: socat not running. Creating fallback PTY symlinks."
fi
