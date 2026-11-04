/* py32f0.ld — PY32F0 定闪存算法的固定地址布局
 *
 * 目标:PY32F0 SRAM 基址 0x20000000(全系 ≥2KB 可用,本布局占用 <0x160)。
 * 每个入口函数一个 section、一个定址槽位;槽位大小在链接时强制
 * (塞不下会 overlap 报错,不会静默串位)。
 *
 * 槽位表(地址 = 常量,烧录引擎侧直接引用,见 py32f0_addrs.h):
 *   0x20000000 init         槽 0x40
 *   0x20000040 uninit       槽 0x40
 *   0x20000080 erase_sector 槽 0x40
 *   0x200000C0 program_page 槽 0x80
 *   0x20000140 bkpt 陷阱    槽 0x10
 *   0x20000150 内部辅助函数 槽 0x90(wait_bsy/clear_flags/wait_done)
 * blob 总长 0x1E0(取整到字)。
 *
 * 栈与数据缓冲不在 loader 侧,由引擎安排(bkpt 之上):
 *   SP 顶   = 0x20000360(往下长,0x20000160..0x20000360 共 512B)
 *   缓冲区  = 0x20000400 起 1KB(整页编程)
 */
ENTRY(py32_init)

MEMORY
{
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 2K
}

SECTIONS
{
    .text.init         0x20000000 : { *(.text.init) }
    .text.uninit       0x20000040 : { *(.text.uninit) }
    .text.erase_sector 0x20000080 : { *(.text.erase_sector) }
    .text.program_page 0x200000C0 : { *(.text.program_page) }
    .text.bkpt         0x20000140 : { *(.text.bkpt) }
    .text.helpers      0x20000150 : { *(.text) *(.text.*) }

    /DISCARD/ : { *(.comment) *(.ARM.attributes) }
}
