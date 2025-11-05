/* app_example/startup_min.S — 适用于 CH32V203C8T6 的最小启动 */
    .section .init
    .globl _start
_start:
    /* 设置栈顶（由链接脚本提供 _estack） */
    la   sp, _estack

    /* 拷贝 .data: from _sidata -> [_sdata, _edata) */
    la   a0, _sidata
    la   a1, _sdata
    la   a2, _edata
1:
    beq  a1, a2, 2f
    lw   t0, 0(a0)
    sw   t0, 0(a1)
    addi a0, a0, 4
    addi a1, a1, 4
    blt  a1, a2, 1b

2:  /* 清 .bss: [_sbss, _ebss) -> 0 */
    la   a1, _sbss
    la   a2, _ebss
3:
    beq  a1, a2, 4f
    sw   x0, 0(a1)
    addi a1, a1, 4
    blt  a1, a2, 3b

4:  /* 可选：调用 SystemInit（若你用到芯片库的时钟初始化） */
    .weak SystemInit
    la   t0, SystemInit
    beqz t0, 5f
    jalr ra, t0

5:  /* 跳 main */
    .globl main
    la   t0, main
    jalr ra, t0

/* main 返回则死循环 */
hang:
    j hang
