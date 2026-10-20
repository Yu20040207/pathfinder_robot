#!/usr/bin/env python3
"""Bounded, exclusive serial bench runner for BASE_CONTROL_V1/V2 firmware."""
import argparse
import csv
import fcntl
import math
import os
import select
import termios
import time

DEVICE = '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:70:69:F7:F0:B0-if00'
FIELDS = ['time_us', 'left_count', 'right_count', 'left_mm_s', 'right_mm_s',
          'left_setpoint', 'right_setpoint', 'left_pwm', 'right_pwm', 'active', 'fault']

class Serial:
    def __init__(self, device):
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.buffer = bytearray()
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            a = termios.tcgetattr(self.fd)
            a[0] = a[1] = a[3] = 0
            a[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            a[4] = a[5] = termios.B115200
            a[6][termios.VMIN] = a[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
        except BaseException:
            os.close(self.fd)
            raise
    def write(self, value):
        data = value.encode('ascii')
        if os.write(self.fd, data) != len(data):
            raise OSError('partial serial write')
    def lines(self, wait=.02):
        lines = []
        if select.select([self.fd], [], [], wait)[0]:
            data = os.read(self.fd, 4096)
            self.buffer.extend(data)
            if len(self.buffer)>16384:
                raise OSError('oversized serial buffer')
            while b'\n' in self.buffer:
                line, _, rest = self.buffer.partition(b'\n')
                self.buffer = bytearray(rest)
                lines.append(line.decode('ascii', errors='replace').strip())
        return lines
    def close(self):
        try:
            self.write('x\n')
            termios.tcdrain(self.fd)
        finally:
            fcntl.ioctl(self.fd, termios.TIOCNXCL)
            os.close(self.fd)

def state(line):
    tokens = line.split()
    if len(tokens)!=12 or tokens[0]!='S':
        return None
    try:
        row = dict(zip(FIELDS, map(float, tokens[1:])))
        return row if all(math.isfinite(v) for v in row.values()) else None
    except ValueError:
        return None

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', default=DEVICE)
    p.add_argument('--action', choices=['inspect', 'run', 'timeout', 'duty', 'loaded', 'arc', 'sync', 'turn'], default='inspect')
    p.add_argument('--left', type=float, default=0, help='left wheel mm/s')
    p.add_argument('--right', type=float, default=0, help='right wheel mm/s')
    p.add_argument('--seconds', type=float, default=2)
    p.add_argument('--duty', type=float, default=0, help='fixed forward duty for both wheels, 0 to 0.65')
    p.add_argument('--left-duty', type=float, help='override left fixed forward duty')
    p.add_argument('--right-duty', type=float, help='override right fixed forward duty')
    p.add_argument('--delta', type=float, default=0, help='arc right-minus-left wheel speed in mm/s')
    p.add_argument('--direction', type=int, default=-1, help='sync: -1 reverse, +1 forward; turn: +1 left, -1 right')
    p.add_argument('--csv')
    args = p.parse_args()
    left_duty = args.duty if args.left_duty is None else args.left_duty
    right_duty = args.duty if args.right_duty is None else args.right_duty
    if not all(math.isfinite(v) for v in (args.left, args.right, args.seconds, args.duty, left_duty, right_duty, args.delta)) or not .1<=args.seconds<=10:
        p.error('finite parameters and duration 0.1 to 10 seconds required')
    if args.action in ('run', 'timeout') and max(abs(args.left), abs(args.right))>100:
        p.error('normal wheel speeds must be within +/-100 mm/s')
    if args.action=='duty' and (not 0<=left_duty<=.65 or not 0<=right_duty<=.65 or
                                max(left_duty,right_duty)<.3 or args.seconds>2):
        p.error('duty test requires both duties within 0 to 0.65, one >=0.30, and at most 2 seconds')
    if args.action=='loaded' and (not 150<=args.left<=200 or not 150<=args.right<=200 or args.seconds>2):
        p.error('loaded test requires both wheel speeds 150 to 200 mm/s and at most 2 seconds')
    if args.action=='arc' and (abs(args.delta)>50 or args.seconds>2):
        p.error('arc test requires wheel-speed difference within +/-50 mm/s and at most 2 seconds')
    if args.action=='sync' and (args.direction not in (-1, 1) or args.seconds>1):
        p.error('sync test requires direction +/-1 and at most 1 second')
    if args.action=='turn' and (args.direction not in (-1, 1) or args.seconds>.5):
        p.error('turn test requires direction +/-1 and at most 0.5 seconds')
    port = Serial(args.device)
    records=[]
    try:
        # x is also safe on the old diagnostic firmware. Never send velocity
        # digits to that firmware: it interprets digits as raw PWM commands.
        port.write('x')
        deadline=time.monotonic()+2
        identified=False
        initial=None
        next_handshake=0
        while time.monotonic()<deadline:
            if time.monotonic()>=next_handshake:
                port.write('?')
                next_handshake=time.monotonic()+.3
            for line in port.lines():
                if line in ('BASE_CONTROL_V1', 'BASE_CONTROL_V2') or line.startswith('BASE_CONTROL_V1:'):
                    identified=True
                row=state(line)
                if row is not None:
                    initial=row
            if identified and initial is not None:
                break
        if not identified or initial is None:
            raise RuntimeError('BASE_CONTROL_V1/V2 handshake missing; no motion command sent')
        if initial['active'] or initial['left_pwm'] or initial['right_pwm']:
            raise RuntimeError('controller did not confirm stopped state')
        if args.action!='inspect' and initial['fault']:
            raise RuntimeError(f"controller fault {int(initial['fault'])}; diagnose before clearing")
        start=time.monotonic()
        next_command=start
        stop_sent=False
        last_sample=start
        while time.monotonic()-start<args.seconds+.7:
            now=time.monotonic()
            elapsed=now-start
            if args.action!='inspect' and elapsed<args.seconds and now>=next_command:
                if args.action=='duty':
                    port.write(f'd {left_duty:g} {right_duty:g}\n')
                elif args.action=='loaded':
                    port.write(f'b {args.left:g} {args.right:g}\n')
                elif args.action=='arc':
                    port.write(f'a {args.delta:g}\n')
                elif args.action=='sync':
                    port.write(f'y {args.direction}\n')
                elif args.action=='turn':
                    port.write(f't {args.direction}\n')
                else:
                    port.write(f'v {args.left:g} {args.right:g}\n')
                next_command=now+.1
            elif args.action in ('run', 'duty', 'loaded', 'arc', 'sync', 'turn') and elapsed>=args.seconds and not stop_sent:
                port.write('x\n'); stop_sent=True
            for line in port.lines():
                row=state(line)
                if row is None:
                    continue
                last_sample=time.monotonic()
                records.append(row)
                if row['fault'] and args.action!='inspect':
                    raise RuntimeError(f"controller fault {int(row['fault'])}; test stopped")
            if time.monotonic()-last_sample>.5:
                raise RuntimeError('telemetry timeout; test stopped')
        if not records:
            raise RuntimeError('no telemetry')
        end=records[-1]
        if end['active'] or abs(end['left_pwm'])>.0001 or abs(end['right_pwm'])>.0001:
            raise RuntimeError('stop was not confirmed')
        moving=[r for r in records if r['active']]
        print(f"action={args.action} samples={len(records)} active_samples={len(moving)} stop_confirmed=True fault={int(end['fault'])}")
        if moving:
            recent=moving[-min(10,len(moving)):]
            print('last_active_mean_mm_s left=%.2f right=%.2f' % (
                sum(r['left_mm_s'] for r in recent)/len(recent),
                sum(r['right_mm_s'] for r in recent)/len(recent)))
        print('final_counts left=%d right=%d' % (end['left_count'],end['right_count']))
    finally:
        port.close()
        if args.csv and records:
            with open(args.csv, 'w', newline='') as f:
                writer=csv.DictWriter(f,fieldnames=FIELDS)
                writer.writeheader();writer.writerows(records)

if __name__=='__main__':
    main()
