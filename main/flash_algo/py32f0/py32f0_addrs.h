/* 由 gen_blob.py 生成,勿手改:PY32F0 loader 的槽位地址与 RAM 布局 */
#ifndef __PY32F0_ADDRS_H__
#define __PY32F0_ADDRS_H__

#define PY32F0_BLOB_WORDS 101

/* 槽位(py32f0.ld 定址,gen_blob.py 断言过落位) */
#define PY32F0_ADDR_INIT         0x20000000u
#define PY32F0_ADDR_UNINIT       0x20000040u
#define PY32F0_ADDR_ERASE_SECTOR 0x20000080u
#define PY32F0_ADDR_PROGRAM_PAGE 0x200000C0u
#define PY32F0_ADDR_BKPT         0x20000140u

/* blob 之外的 RAM 安排(见 py32f0.ld 头注释) */
#define PY32F0_STACK_TOP         0x20000360u
#define PY32F0_BUFFER            0x20000400u
#define PY32F0_BUFFER_SIZE       1024u
#define PY32F0_PAGE_SIZE         1024u
#define PY32F0_FLASH_BASE        0x08000000u

#endif
