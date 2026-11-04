/*
 * py32f0_loader.c — PY32F0 系列的定闪存算法(RAM loader)
 *
 * 跑在目标 MCU 的 SRAM 里,由调试器经 swd_flash_syscall_exec 调起:
 *   r0..r3 = 参数,r9 = static base,sp = 栈顶,lr = bkpt 陷阱地址,
 *   返回 0 = 成功,非 0 = 失败(约定见 components/DAP/include/flash_blob.h)。
 *
 * PY32F0 的闪存控制器与 STM32F1 同构(寄存器基址 0x40022000:
 * KEYR +0x04 / SR +0x0C / CR +0x10 / AR +0x14),页擦除走 PER+AR+STRT,
 * 编程走 PG + 16 位半字写。PY32F003 页大小 1KB(128B 的小容量变体
 * 由家族表另立条目,不在这份 loader 里猜)。
 *
 * 无 libc、无静态数据,四个入口各自固定地址(见 py32f0.ld),
 * 生成物 py32f0_blob.c 由 gen_blob.py 从 .bin 重新生成。
 */
#include <stdint.h>

#define FLASH_BASE 0x40022000u
#define FLASH_KEYR (*(volatile uint32_t *)(FLASH_BASE + 0x04))
#define FLASH_SR   (*(volatile uint32_t *)(FLASH_BASE + 0x0C))
#define FLASH_CR   (*(volatile uint32_t *)(FLASH_BASE + 0x10))
#define FLASH_AR   (*(volatile uint32_t *)(FLASH_BASE + 0x14))

#define SR_BSY      (1u << 0)
#define SR_PGERR    (1u << 2)
#define SR_WRPRTERR (1u << 4)
#define SR_EOP      (1u << 5)
#define CR_PG       (1u << 0)
#define CR_PER      (1u << 1)
#define CR_STRT     (1u << 6)
#define CR_LOCK     (1u << 7)

static void wait_bsy(void) {
    while (FLASH_SR & SR_BSY) {
    }
}

static void clear_flags(void) {
    /* F1 型 SR 标志写 1 清零 */
    FLASH_SR = SR_EOP | SR_PGERR | SR_WRPRTERR;
}

/* 等一次操作完成(BSY 落下),返回 0=成功 1=闪存报错 */
static uint32_t wait_done(void) {
    wait_bsy();
    if (FLASH_SR & (SR_PGERR | SR_WRPRTERR)) {
        clear_flags();
        return 1;
    }
    if (FLASH_SR & SR_EOP) {
        FLASH_SR = SR_EOP;
    }
    return 0;
}

__attribute__((section(".text.init"))) uint32_t py32_init(uint32_t adr,
                                                          uint32_t clk,
                                                          uint32_t fnc) {
    (void)adr;
    (void)clk;
    (void)fnc;
    FLASH_KEYR = 0x45670123u;
    FLASH_KEYR = 0xCDEF89ABu;
    if (FLASH_CR & CR_LOCK) {
        return 1; /* 两次 KEYR 后仍锁着 = 已处于解锁态或硬件异常,按失败报 */
    }
    clear_flags();
    return 0;
}

__attribute__((section(".text.uninit"))) uint32_t py32_uninit(uint32_t fnc) {
    (void)fnc;
    FLASH_CR |= CR_LOCK;
    return 0;
}

__attribute__((section(".text.erase_sector"))) uint32_t py32_erase_sector(
    uint32_t adr) {
    wait_bsy();
    FLASH_CR |= CR_PER;
    FLASH_AR = adr;
    FLASH_CR |= CR_STRT;
    if (wait_done() != 0) {
        FLASH_CR &= ~CR_PER;
        return 1;
    }
    FLASH_CR &= ~CR_PER;
    return 0;
}

__attribute__((section(".text.program_page"))) uint32_t py32_program_page(
    uint32_t dst, uint32_t size, uint32_t buf) {
    wait_bsy();
    FLASH_CR |= CR_PG;
    const volatile uint16_t *src = (const volatile uint16_t *)buf;
    volatile uint16_t *d = (volatile uint16_t *)dst;
    for (uint32_t i = 0; i < (size >> 1); i++) {
        d[i] = src[i];
        wait_bsy(); /* 半字写完等 BSY,防流水打满 */
    }
    FLASH_CR &= ~CR_PG;
    return wait_done();
}

/* syscall 的返回陷阱:lr 指到这里,函数返回时 BKPT 把核心拍停,
 * swd_wait_until_halted() 由此感知算法执行完毕。 */
__attribute__((section(".text.bkpt"), naked)) void py32_bkpt_trap(void) {
    __asm volatile("bkpt 0\n bx lr\n");
}
