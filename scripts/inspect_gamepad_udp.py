#!/usr/bin/env python3
"""Inspect the exact wbc_ballet 61-float gamepad UDP stream without touching motors."""
import argparse, socket, struct, time

P = argparse.ArgumentParser()
P.add_argument('--host', default='0.0.0.0')
P.add_argument('--port', type=int, default=55001)
args = P.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((args.host, args.port))
sock.settimeout(1.0)
print(f'Listening on UDP {args.host}:{args.port}; expected 244-byte packets (61 x <f4). Ctrl-C to stop.')
count=0; t0=time.monotonic(); latest=None
try:
    while True:
        try:
            data, peer = sock.recvfrom(65535)
        except socket.timeout:
            print('NO PACKETS in last 1 s')
            continue
        if len(data) != 244:
            print(f'IGNORED {len(data)} bytes from {peer}; expected 244')
            continue
        vals=struct.unpack('<61f', data)
        if not all(x == x and abs(x) != float('inf') for x in vals):
            print('IGNORED non-finite packet'); continue
        targets=vals[:29]; mask=tuple(1 if x >= .5 else 0 for x in vals[29:58]); vel=vals[58:61]
        count += 1; now=time.monotonic()
        if now-t0 >= 1.0:
            active=[i for i,m in enumerate(mask) if m]
            active_targets=[round(targets[i],3) for i in active]
            hz=count/(now-t0)
            print(f'{hz:5.1f} Hz from {peer[0]}  velocity={[round(x,3) for x in vel]}  active_axes={active}  targets(active)={active_targets}')
            count=0; t0=now
except KeyboardInterrupt:
    pass
finally:
    sock.close()
