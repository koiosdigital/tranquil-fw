#!/usr/bin/env python3
"""Convert a binary file to a C array for embedding."""

import sys

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input.bin> <output.c>", file=sys.stderr)
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    with open(input_file, 'rb') as f:
        data = f.read()

    with open(output_file, 'w') as f:
        f.write('/* Auto-generated - TEE handlers binary linked at 0x600FE300 */\n')
        f.write('#include <stdint.h>\n')
        f.write('#include <stddef.h>\n\n')
        f.write('const uint8_t _tee_handlers_bin[] = {\n    ')
        for i, b in enumerate(data):
            f.write(f'0x{b:02x},')
            if (i + 1) % 16 == 0:
                f.write('\n    ')
            else:
                f.write(' ')
        f.write('\n};\n\n')
        f.write(f'const size_t _tee_handlers_bin_len = {len(data)};\n')

    print(f"Generated {output_file} ({len(data)} bytes)")

if __name__ == '__main__':
    main()
