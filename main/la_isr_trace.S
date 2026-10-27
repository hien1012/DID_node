/*
 * Debug：LA 量測腳的 ISR 部分 (APP_LA_PROBE_ENABLE；task 切換部分見 la_task_trace.h)。
 *
 * IDF 5.1.2 Xtensa 的 level 1..3 中斷 (xtensa_vectors.S) 都走
 *   call0 _frxt_int_enter → dispatch_c_isr (呼叫各 ISR) → call0 _frxt_int_exit
 * main/CMakeLists.txt 以 -Wl,--wrap 把這兩個呼叫導到這裡 (不改 IDF)：
 *  - 進入最外層中斷 (port_interruptNesting 0→1 之後)：本核心的 idle 腳與 task 腳拉低
 *  - 離開最外層中斷 (1→0 之前)：依 la_probe_core_idle / la_probe_core_pin 拉回
 * 只有 traceISR_ENTER/EXIT 不夠：IDF 只在 shared 中斷與 systick 呼叫它們，
 * 一般 (non-shared) 中斷要開 SystemView 才會呼叫。
 *
 * 中斷結束要切換 task 時，這裡先恢復被中斷的 task，接著 vPortYieldFromInt →
 * vTaskSwitchContext 的 traceTASK_SWITCHED_IN 再換腳 (切換本身的時間算舊 task，
 * 與 task 主動 yield 一致)。不能看 port_switch_flag 省掉恢復：該核心排程器暫停時
 * vTaskSwitchContext 不切換、也不呼叫 hook，腳會一直停在低。
 *
 * 不涵蓋：level 4/5 高優先權中斷與 NMI (不經 _frxt_int_enter)，以及向量本身在
 * _frxt_int_enter 之前 / _xt_context_restore 的暫存器存取 (數十個指令)。
 * 會用到 SAR：兩個呼叫點的 SAR 都已存進中斷 frame，離開中斷時還原。
 */
#include "sdkconfig.h"
#include "app_config.h"
#include "board_pins.h"

/* LA_ISR_WRAP 由 main/CMakeLists.txt 在加上 --wrap 時一起定義 */
#ifdef LA_ISR_WRAP

#include "xtensa/xtensa_context.h"
#include "soc/gpio_reg.h"

    .extern port_interruptNesting
    .extern la_probe_core_pin
    .extern la_probe_core_idle

/* 被包住的 _frxt_int_enter 回向量的位址，每核心一格。只在中斷關閉 (EXCM) 期間
 * 使用，同一核心不會重入。放 DRAM：cache 關閉時也會進中斷。 */
    .section .bss.la_isr_ret, "aw", @nobits
    .align  4
    .type   la_isr_ret, @object
la_isr_ret:
    .space  8
    .size   la_isr_ret, 8

/* 把 1 << (\pin & 31) 寫進 \lo (pin < 32) 或 \hi。\pin 是放 GPIO 編號的暫存器
 * (不變)，\t1 / \t2 會被覆蓋。 */
    .macro  la_pin_write pin, t1, t2, lo, hi
    movi    \t1, 1
    ssl     \pin
    sll     \t1, \t1
    movi    \t2, \lo
    bltui   \pin, 32, .Lla_pin_lo\@
    movi    \t2, \hi
.Lla_pin_lo\@:
    s32i    \t1, \t2, 0
    .endm

    .section .iram1, "ax"

/*
 * call0 進來，此時只有 a0/a1 存過 (a1 指向向量剛配置的中斷 frame)，a2..a15
 * 還是被中斷者的值。先借 a2/a3 (暫存到 frame 中 _xt_context_save 稍後也會寫的同一格)
 * 把 a0 記到 la_isr_ret，再呼叫原函式；它回來時整個 context 已存好、a1 可能已換到
 * ISR stack，a0/a2..a4 可自由使用 (原函式本身也用 a2..a4)，中斷仍關閉。
 */
    .global __wrap__frxt_int_enter
    .type   __wrap__frxt_int_enter, @function
    .align  4
__wrap__frxt_int_enter:
    s32i    a2, a1, XT_STK_A2
    s32i    a3, a1, XT_STK_A3
    getcoreid a2
    movi    a3, la_isr_ret
    addx4   a3, a2, a3
    s32i    a0, a3, 0
    l32i    a2, a1, XT_STK_A2
    l32i    a3, a1, XT_STK_A3
    movi    a0, __real__frxt_int_enter
    callx0  a0

    getcoreid a4
    movi    a2, port_interruptNesting
    addx4   a2, a4, a2
    l32i    a2, a2, 0
    bnei    a2, 1, 1f                   /* 巢狀 (>1) 或排程器未啟動 (0)：不動 */

    movi    a2, LA_PIN_IDLE_CPU0
    beqz    a4, 2f
    movi    a2, LA_PIN_IDLE_CPU1
2:
    la_pin_write a2, a0, a3, GPIO_OUT_W1TC_REG, GPIO_OUT1_W1TC_REG
    movi    a2, la_probe_core_pin
    add     a2, a2, a4
    l8ui    a2, a2, 0
    beqz    a2, 1f
    la_pin_write a2, a0, a3, GPIO_OUT_W1TC_REG, GPIO_OUT1_W1TC_REG
1:
    movi    a3, la_isr_ret
    addx4   a3, a4, a3
    l32i    a0, a3, 0
    ret
    .size   __wrap__frxt_int_enter, . - __wrap__frxt_int_enter

/*
 * ISR 都跑完了。原函式一開始就覆蓋 a0/a2..a4 且不回到呼叫者，所以這裡可直接用，
 * 最後跳過去。先關中斷 (原函式隨後也會做)：PS.INTLEVEL 此時還是本中斷的 level，
 * 更高 level 的中斷可能在恢復途中插進來。
 */
    .global __wrap__frxt_int_exit
    .type   __wrap__frxt_int_exit, @function
    .align  4
__wrap__frxt_int_exit:
    rsil    a0, XCHAL_EXCM_LEVEL
    getcoreid a4
    movi    a2, port_interruptNesting
    addx4   a2, a4, a2
    l32i    a2, a2, 0
    bnei    a2, 1, 1f                   /* 仍在巢狀中，或排程器未啟動 */

    movi    a2, la_probe_core_idle
    add     a2, a2, a4
    l8ui    a2, a2, 0
    beqz    a2, 2f
    movi    a2, LA_PIN_IDLE_CPU0
    beqz    a4, 3f
    movi    a2, LA_PIN_IDLE_CPU1
3:
    la_pin_write a2, a0, a3, GPIO_OUT_W1TS_REG, GPIO_OUT1_W1TS_REG
2:
    movi    a2, la_probe_core_pin
    add     a2, a2, a4
    l8ui    a2, a2, 0
    beqz    a2, 1f
    la_pin_write a2, a0, a3, GPIO_OUT_W1TS_REG, GPIO_OUT1_W1TS_REG
1:
    movi    a0, __real__frxt_int_exit
    jx      a0
    .size   __wrap__frxt_int_exit, . - __wrap__frxt_int_exit

#endif /* LA_ISR_WRAP */
