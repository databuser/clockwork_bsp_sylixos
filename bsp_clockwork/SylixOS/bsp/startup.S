;/*********************************************************************************************************
;**
;**                                    中国软件开源组织
;**
;**                                   嵌入式实时操作系统
;**
;**                                SylixOS(TM)  LW : long wing
;**
;**                               Copyright All Rights Reserved
;**
;**--------------文件信息--------------------------------------------------------------------------------
;**
;** 文   件   名: startup.S
;**
;** 创   建   人: Han.Hui (韩辉)
;**
;** 文件创建日期: 2007 年 12 月 09 日
;**
;** 描        述: 处理器系统启动.
;*********************************************************************************************************/
    
#ifndef ASSEMBLY
#define ASSEMBLY 1
#endif

#include "config.h"
#include <arch/assembler.h>

    FILE_BEGIN()

;/*********************************************************************************************************
;  配置各处理器模式堆栈空间的大小
;*********************************************************************************************************/

#define SVC_STACK_SIZE  0x00002000
#define SYS_STACK_SIZE  0x00001000
#define FIQ_STACK_SIZE  0x00001000
#define UND_STACK_SIZE  0x00001000
#define ABT_STACK_SIZE  0x00001000
#define IRQ_STACK_SIZE  0x00001000

#define CPU_STACKS_SIZE     (SVC_STACK_SIZE + \
                             SYS_STACK_SIZE + \
                             FIQ_STACK_SIZE + \
                             IRQ_STACK_SIZE + \
                             UND_STACK_SIZE + \
                             ABT_STACK_SIZE)

;/*********************************************************************************************************
;  声明外部函数和导出全局函数
;*********************************************************************************************************/

    IMPORT_LABEL(archIntEntry)
    IMPORT_LABEL(archAbtEntry)
    IMPORT_LABEL(archPreEntry)
    IMPORT_LABEL(archUndEntry)
    IMPORT_LABEL(archSwiEntry)

    IMPORT_LABEL(targetInit)
    IMPORT_LABEL(halPrimaryCpuMain)
    IMPORT_LABEL(halSecondaryCpuMain)
    IMPORT_LABEL(armBranchPredictorInvalidate)
    IMPORT_LABEL(armBranchPredictionEnable)
    IMPORT_LABEL(armBranchPredictionDisable)
    IMPORT_LABEL(armDCacheV7ClearAll)
    IMPORT_LABEL(armDCacheV7Disable)
    IMPORT_LABEL(armDCacheV7InvalidateAll)
    IMPORT_LABEL(armICacheInvalidateAll)
    IMPORT_LABEL(armICacheDisable)
    IMPORT_LABEL(armAuxControlFeatureDisable)
    IMPORT_LABEL(armMmuDisable)

;/*********************************************************************************************************
;  异常向量表
;*********************************************************************************************************/
    .code 32
    .section .vector, "ax"
    .balign 4

FUNC_DEF(vector)
    B       reset
    LDR     PC, undefineEntry
    LDR     PC, swiEntry
    LDR     PC, prefetchEntry
    LDR     PC, abortEntry
    LDR     PC, reserveEntry
    LDR     PC, irqEntry
    LDR     PC, fiqEntry
    FUNC_END()

FUNC_LABEL(undefineEntry)
    .word   archUndEntry

FUNC_LABEL(swiEntry)
    .word   archSwiEntry

FUNC_LABEL(prefetchEntry)
    .word   archPreEntry

FUNC_LABEL(abortEntry)
    .word   archAbtEntry

FUNC_LABEL(reserveEntry)
    .word   0

FUNC_LABEL(irqEntry)
    .word   archIntEntry

FUNC_LABEL(fiqEntry)
    .word   0

;/********************************************************************************************************
;  复位入口
;********************************************************************************************************/

    FILE_BEGIN()

FUNC_DEF(reset)

;/********************************************************************************************************
;  进入 SVC32 模式(关中断)
;********************************************************************************************************/

    MSR     CPSR_c, #(SVC32_MODE | DIS_INT)

;/********************************************************************************************************
;  初始化堆栈
;********************************************************************************************************/

    BL      archMpCur

    LDR     R1 , =__stack_end                                          ;/*  栈区顶端地址                */
LINE_LABEL(0)
    CMP     R0, #0
    SUBNE   R1, R1, #CPU_STACKS_SIZE
    SUBNE   R0, R0, #1
    BNE     0b

    MOV     R0, R1
    BL      armInitSP

;/********************************************************************************************************
;  如果不是 Primary CPU, 则跳转到 secondaryCpuResetEntry
;********************************************************************************************************/

    BL      archMpCur
    CMP     R0, #0
    BLNE    secondaryCpuResetEntry

;/********************************************************************************************************
;  关闭 D-Cache(回写并无效)
;  关闭 I-Cache(无效)
;  无效并关闭分支预测
;  关闭 MMU(无效 TLB)
;********************************************************************************************************/

    BL      armDCacheV7ClearAll
    BL      armDCacheV7Disable
    BL      armICacheInvalidateAll
    BL      armICacheDisable

    BL      armBranchPredictorInvalidate
    BL      armBranchPredictionDisable

    BL      armMmuInvalidateTLB
    BL      armMmuDisable

;/********************************************************************************************************
;  初始化 DATA 段
;********************************************************************************************************/

    LDR     R1, =_etext                         ;/*  -> ROM data end                                    */
    LDR     R2, =_data                          ;/*  -> data start                                      */
    LDR     R3, =_edata                         ;/*  -> end of data                                     */
LINE_LABEL(1)
    CMP     R2, R3                              ;/*  check if data to move                              */
    LDRLO   R0, [R1], #4                        ;/*  copy it                                            */
    STRLO   R0, [R2], #4
    BLO     1b                                  ;/*  loop until done                                    */

;/********************************************************************************************************
;  清 0 BSS 段
;********************************************************************************************************/

    MOV     R0, #0                              ;/*  get a zero                                         */
    LDR     R1, =__bss_start                    ;/*  -> bss start                                       */
    LDR     R2, =__bss_end                      ;/*  -> bss end                                         */
LINE_LABEL(2)
    CMP     R1, R2                              ;/*  check if data to clear                             */
    STRLO   R0, [R1], #4                        ;/*  clear 4 bytes                                      */
    BLO     2b                                  ;/*  loop until done                                    */

    DSB

;/********************************************************************************************************
;  调用 targetInit 函数
;********************************************************************************************************/

    LDR     R10, =targetInit
    MOV     LR, PC
    BX      R10

;/********************************************************************************************************
;  进入 halPrimaryCpuMain 函数 (argc = 0, argv = NULL, frame pointer = NULL)
;********************************************************************************************************/

    MOV     R0, #0
    MOV     R1, #0
    MOV     R2, #0
    MOV     FP, #0

    LDR     R10, =halPrimaryCpuMain
    MOV     LR, PC
    BX      R10

    B       .

    FUNC_END()

;/********************************************************************************************************
;  Secondary CPU 复位入口
;********************************************************************************************************/

FUNC_DEF(secondaryCpuResetEntry)

;/********************************************************************************************************
;  关闭 D-Cache(无效)
;  关闭 I-Cache(无效)
;  无效并关闭分支预测
;  关闭 MMU(无效 TLB)
;********************************************************************************************************/

    BL      armDCacheV7InvalidateAll
    BL      armDCacheV7Disable
    BL      armICacheInvalidateAll
    BL      armICacheDisable

    BL      armBranchPredictorInvalidate
    BL      armBranchPredictionDisable

    BL      armMmuInvalidateTLB
    BL      armMmuDisable

;/********************************************************************************************************
;  进入 halSecondaryCpuMain 函数 (argc = 0, argv = NULL, frame pointer = NULL)
;********************************************************************************************************/

    MOV     R0, SP
    MOV     R1, #0
    MOV     R2, #0
    MOV     FP, #0

    LDR     R10, =halSecondaryCpuMain
    MOV     LR, PC
    BX      R10

    B       .

    FUNC_END()

;/********************************************************************************************************
;  初始化各处理器模式的堆栈指针
;********************************************************************************************************/

FUNC_DEF(armInitSP)

    MSR     CPSR_c, #(SVC32_MODE | DIS_INT)
    MOV     SP, R0
    SUB     R0, R0, #SVC_STACK_SIZE

    MSR     CPSR_c, #(SYS32_MODE | DIS_INT)
    MOV     SP, R0
    SUB     R0, R0, #SYS_STACK_SIZE

    MSR     CPSR_c, #(FIQ32_MODE | DIS_INT)
    MOV     SP, R0
    SUB     R0, R0, #FIQ_STACK_SIZE

    MSR     CPSR_c, #(IRQ32_MODE | DIS_INT)
    MOV     SP, R0
    SUB     R0, R0, #IRQ_STACK_SIZE

    MSR     CPSR_c, #(UND32_MODE | DIS_INT)
    MOV     SP, R0
    SUB     R0, R0, #UND_STACK_SIZE

    MSR     CPSR_c, #(ABT32_MODE | DIS_INT)
    MOV     SP, R0
    SUB     R0, R0, #ABT_STACK_SIZE

    MSR     CPSR_c, #(SVC32_MODE | DIS_INT)

    BX      LR

    FUNC_END()

    FILE_END()

;/********************************************************************************************************
;  END
;********************************************************************************************************/
