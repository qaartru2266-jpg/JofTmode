import sys, pathlib
src, dst = sys.argv[1], sys.argv[2]
data = pathlib.Path(src).read_bytes()
with open(dst, "w", newline="\n") as f:
    f.write('#include <cstdint>\n')
    f.write('alignas(16) const unsigned char g_model[] = {\n')
    for i, b in enumerate(data):
        f.write(f'0x{b:02x},')
        if i % 12 == 11:
            f.write('\n')
    f.write('\n};\n')
    f.write(f'const unsigned int g_model_len = {len(data)};\n')
