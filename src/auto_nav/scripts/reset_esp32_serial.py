#!/usr/bin/env python3
import argparse
import serial
import time

parser = argparse.ArgumentParser()
parser.add_argument("port")
args = parser.parse_args()

with serial.Serial(args.port, 115200) as s:
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    s.dtr = True
    s.rts = False
    time.sleep(0.05)
    s.dtr = False  # 通常通信状態に戻す

time.sleep(2.0)  # ESP32 起動完了待ち
print(f"reset {args.port} done")
