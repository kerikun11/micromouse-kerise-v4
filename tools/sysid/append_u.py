import sys
import os

if len(sys.argv) < 3:
    print('file tra rot')
    exit(1)

filename = sys.argv[1]
tra = sys.argv[2]
rot = sys.argv[3]

with open(filename, 'r') as f:
    base, ext = os.path.splitext(filename)
    filename_out = base + '_.csv'
    with open(filename_out, 'w') as of:
        for l in f.readlines():
            l = l.strip()
            of.write(f'{l}\t{tra}\t{rot}\n')
