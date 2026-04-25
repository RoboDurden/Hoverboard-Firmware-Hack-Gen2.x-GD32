#!/usr/bin/env python3
"""Emit a 360-entry Q15 sine table as a C literal.

Run once; paste the output into Src/bldcFOC.c as
`static const int16_t sine_q15[360] = { ... };`.

Table format: sine_q15[i] = round(sin(i * pi / 180) * 32767), 10 per line.
Range: -32767..+32767, so signed int16_t multiplies can't overflow int32.
"""
import math

N = 360
VALUES = [round(math.sin(i * math.pi / 180) * 32767) for i in range(N)]

print("static const int16_t sine_q15[360] = {")
for i in range(0, N, 10):
    row = ", ".join(f"{v:6d}" for v in VALUES[i:i + 10])
    print(f"\t{row},")
print("};")
