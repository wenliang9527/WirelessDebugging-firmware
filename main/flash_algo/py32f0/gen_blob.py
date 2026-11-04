#!/usr/bin/env python3
"""gen_blob.py — 从 py32f0_loader.bin 生成 py32f0_blob.c / py32f0_addrs.h

用法(在 py32f0/ 目录下):
    arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Os -ffreestanding \\
        -fno-builtin -nostdlib -Werror -Wall \\
        -Wl,-T,py32f0.ld -Wl,--build-id=none -o py32f0_loader.elf py32f0_loader.c
    arm-none-eabi-objcopy -O binary py32f0_loader.elf py32f0_loader.bin
    python gen_blob.py

地址槽位是 py32f0.ld 里的常量,这里只做一致性断言:反汇编符号必须
落在预期槽位上,槽位被塞爆时链接器先报 overlap,这里兜底再验一次。
"""
import subprocess
import sys

SLOTS = {  # 符号 -> (地址, 槽大小)
    "py32_init":         (0x20000000, 0x40),
    "py32_uninit":       (0x20000040, 0x40),
    "py32_erase_sector": (0x20000080, 0x40),
    "py32_program_page": (0x200000C0, 0x80),
    "py32_bkpt_trap":    (0x20000140, 0x10),
}
BLOB_END = 0x200001E0          # 覆盖 helpers 槽(0x150 + 0x90)
OBJDUMP = None                 # 由 --objdump 指定,默认 PATH 里找


def find_tool(name):
    import shutil
    p = shutil.which(name)
    if not p:
        sys.exit(f"找不到 {name},用 --objdump 指定绝对路径")
    return p


def main():
    global OBJDUMP
    if "--objdump" in sys.argv:
        OBJDUMP = sys.argv[sys.argv.index("--objdump") + 1]
    else:
        OBJDUMP = find_tool("arm-none-eabi-objdump")

    blob = open("py32f0_loader.bin", "rb").read()
    if len(blob) > BLOB_END - 0x20000000:
        sys.exit(f"blob {len(blob):#x}B 超出声明范围 {BLOB_END - 0x20000000:#x}B")

    # 反汇编校验符号落位
    out = subprocess.run([OBJDUMP, "-d", "py32f0_loader.elf"],
                         capture_output=True, text=True, check=True).stdout
    for sym, (addr, slot) in SLOTS.items():
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 2 and parts[1].rstrip(":") == f"<{sym}>":
                got = int(parts[0].rstrip(":"), 16)
                if got != addr:
                    sys.exit(f"{sym} 落在 {got:#x},预期 {addr:#x}(改 ld 或改本表)")
                break
        else:
            sys.exit(f"符号 {sym} 不在反汇编输出里")

    words = []
    padded = blob + b"\x00" * ((-len(blob)) % 4)
    for i in range(0, len(padded), 4):
        words.append(int.from_bytes(padded[i:i + 4], "little"))

    with open("py32f0_blob.c", "w", encoding="utf-8", newline="\n") as f:
        f.write("/* 由 gen_blob.py 从 py32f0_loader.bin 生成,勿手改。\n"
                " * loader 源码/链接脚本/再生成步骤见本目录。 */\n"
                "#include <stdint.h>\n\n"
                f"const uint32_t py32f0_blob[{len(words)}] = {{\n")
        for i in range(0, len(words), 6):
            f.write("    " + ", ".join(f"0x{w:08x}" for w in words[i:i + 6]) + ",\n")
        f.write("};\n")

    with open("py32f0_addrs.h", "w", encoding="utf-8", newline="\n") as f:
        f.write("/* 由 gen_blob.py 生成,勿手改:PY32F0 loader 的槽位地址与 RAM 布局 */\n"
                "#ifndef __PY32F0_ADDRS_H__\n#define __PY32F0_ADDRS_H__\n\n"
                f"#define PY32F0_BLOB_WORDS {len(words)}\n\n"
                "/* 槽位(py32f0.ld 定址,gen_blob.py 断言过落位) */\n"
                "#define PY32F0_ADDR_INIT         0x20000000u\n"
                "#define PY32F0_ADDR_UNINIT       0x20000040u\n"
                "#define PY32F0_ADDR_ERASE_SECTOR 0x20000080u\n"
                "#define PY32F0_ADDR_PROGRAM_PAGE 0x200000C0u\n"
                "#define PY32F0_ADDR_BKPT         0x20000140u\n\n"
                "/* blob 之外的 RAM 安排(见 py32f0.ld 头注释) */\n"
                "#define PY32F0_STACK_TOP         0x20000360u\n"
                "#define PY32F0_BUFFER            0x20000400u\n"
                "#define PY32F0_BUFFER_SIZE       1024u\n"
                "#define PY32F0_PAGE_SIZE         1024u\n"
                "#define PY32F0_FLASH_BASE        0x08000000u\n\n"
                "#endif\n")

    print(f"py32f0_blob.c: {len(words)} words ({len(blob)} bytes loader)")


if __name__ == "__main__":
    main()
