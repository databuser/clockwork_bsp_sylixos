/*
 * armGic.c
 *
 *  Created on: Jun 6, 2019
 *      Author: databus
 */
#define  __SYLIXOS_KERNEL
#include "SylixOS.h"
#include "driver/gic/armGic.h"
#include "arch/arm/common/cp15/armCp15.h"
/*********************************************************************************************************
  GIC 寄存器偏移
*********************************************************************************************************/
#define GIC_DISTRIBUTOR_REGS_BASE_OFFSET        (0x1000)/*  GIC 中断发布器寄存器偏移                    */
#define GIC_CPU_INTERFACE_REGS_BASE_OFFSET      (0x2000)/*  GIC CPU 接口寄存器偏移                      */
/*********************************************************************************************************
  GIC 中断发布器寄存器
  使用 GICv2 寄存器名字, 但不包括 GICv2 寄存器
*********************************************************************************************************/
typedef struct {
    volatile UINT32     uiCTLR;                         /*  Distributor Control Register.               */
    volatile UINT32     uiTYPER;                        /*  Interrupt Controller Type Register.         */
    volatile UINT32     uiIIDR;                         /*  Distributor Implementer Identification Reg. */
    volatile UINT32     uiReserved0[29];
    volatile UINT32     uiIGROUPRn[8];                  /*  Interrupt Group Registers.                  */
    volatile UINT32     uiReserved1[24];
    volatile UINT32     uiISENABLERn[32];               /*  Interrupt Set-Enable Registers.             */
    volatile UINT32     uiICENABLERn[32];               /*  Interrupt Clear-Enable Registers.           */
    volatile UINT32     uiISPENDRn[32];                 /*  Interrupt Set-Pending Registers.            */
    volatile UINT32     uiICPENDRn[32];                 /*  Interrupt Clear-Pending Registers.          */
    volatile UINT32     uiICDABRn[32];                  /*  Active Bit Registers.                       */
    volatile UINT32     uiReserved2[32];
    volatile UINT8      uiIPRIORITYRn[255 * sizeof(UINT32)];/*  Interrupt Priority Registers.           */
    volatile UINT32     uiReserved3;
    volatile UINT8      uiITARGETSRn[255 * sizeof(UINT32)]; /*  Interrupt Processor Targets Registers.  */
    volatile UINT32     uiReserved4;
    volatile UINT32     uiICFGRn[64];                   /*  Interrupt Configuration Registers.          */
    volatile UINT32     uiReserved5[128];
    volatile UINT32     uiSGIR;                         /*  Software Generated Interrupt Register.      */
} GIC_DISTRIBUTOR_REG;
/*********************************************************************************************************
  GIC_DISTRIBUTOR_REG.uiCTLR 寄存器位域常量
*********************************************************************************************************/
#define BTI_MASK_CTLR_ENABLEGRP1                (0x1ul << 1)
#define BTI_MASK_CTLR_EABBLEGRP0                (0x1ul << 0)
/*********************************************************************************************************
  GIC_DISTRIBUTOR_REG.uiSGIR 寄存器位域常量
*********************************************************************************************************/
#define BIT_OFFS_SGIR_TARGETLISTFILTER          (24)
#define BTI_MASK_SGIR_TARGETLISTFILTER          (0x3ul << BIT_OFFS_SGIR_TARGETLISTFILTER)

#define BIT_OFFS_SGIR_CPUTARGETLIST             (16)
#define BTI_MASK_SGIR_CPUTARGETLIST             (0xFFul << BIT_OFFS_SGIR_CPUTARGETLIST)

#define BIT_OFFS_SGIR_NSATT                     (15)
#define BTI_MASK_SGIR_NSATT                     (0x1ul << BIT_OFFS_SGIR_NSATT),

#define BIT_OFFS_SGIR_SGIINTID                  (0)
#define BTI_MASK_SGIR_SGIINTID                  (0xFul << BIT_OFFS_SGIR_SGIINTID)
/*********************************************************************************************************
  GIC CPU 接口寄存器
  使用 GICv2 寄存器名字, 但不包括 GICv2 寄存器
*********************************************************************************************************/
typedef struct {
    volatile UINT32     uiCTLR;                         /*  CPU Interface Control Register.             */
    volatile UINT32     uiPMR;                          /*  Interrupt Priority Mask Register.           */
    volatile UINT32     uiBPR;                          /*  Binary Point Register.                      */
    volatile UINT32     uiIAR;                          /*  Interrupt Acknowledge Register.             */
    volatile UINT32     uiEOIR;                         /*  End of Interrupt Register.                  */
    volatile UINT32     uiRPR;                          /*  Running Priority Register.                  */
    volatile UINT32     uiHPPIR;                        /*  Highest Priority Pending Interrupt Register.*/
    volatile UINT32     uiABPR;                         /*  Aliased Binary Point Register.              */
                                                        /*  (only visible with a secure access)         */
    volatile UINT32     uiReserved[56];
    volatile UINT32     uiIIDR;                         /*  CPU Interface Identification Register.      */
} GIC_CPU_INTERFACE_REGS;
/*********************************************************************************************************
  GIC_CPU_INTERFACE_REGS.uiCTLR 寄存器位域常量
*********************************************************************************************************/
#define BIT_OFFS_CTLR_ENABLES                   (0)
#define BTI_MASK_CTLR_ENABLES                   (0x1ul << BIT_OFFS_CTLR_ENABLES)

#define BIT_OFFS_CTLR_ENABLENS                  (1)
#define BTI_MASK_CTLR_ENABLENS                  (0x1ul << BIT_OFFS_CTLR_ENABLENS)

#define BIT_OFFS_CTLR_ACKCTL                    (2)
#define BTI_MASK_CTLR_ACKCTL                    (0x1ul << BIT_OFFS_CTLR_ACKCTL)

#define BIT_OFFS_CTLR_FIQENABLE                 (3)
#define BTI_MASK_CTLR_FIQENABLE                 (0x1ul << BIT_OFFS_CTLR_FIQENABLE)

#define BIT_OFFS_CTLR_SBPR                      (4)
#define BTI_MASK_CTLR_SBPR                      (0x1ul << BIT_OFFS_CTLR_SBPR)
/*********************************************************************************************************
** 函数名称: armGicIntDistributorGet
** 功能描述: 获得中断发布器
** 输　入  : NONE
** 输　出  : 中断发布器寄存器基址
*********************************************************************************************************/
static LW_INLINE GIC_DISTRIBUTOR_REG  *armGicIntDistributorGet (VOID)
{
    REGISTER addr_t  ulBase = armPrivatePeriphBaseGet() + GIC_DISTRIBUTOR_REGS_BASE_OFFSET;

    return  ((GIC_DISTRIBUTOR_REG *)ulBase);
}
/*********************************************************************************************************
** 函数名称: armGicCpuInterfaceGet
** 功能描述: 获得 CPU 接口
** 输　入  : NONE
** 输　出  : CPU 接口寄存器基址
*********************************************************************************************************/
static LW_INLINE GIC_CPU_INTERFACE_REGS  *armGicCpuInterfaceGet (VOID)
{
    REGISTER addr_t  ulBase = armPrivatePeriphBaseGet() + GIC_CPU_INTERFACE_REGS_BASE_OFFSET;

    return  ((GIC_CPU_INTERFACE_REGS *)ulBase);
}
/*********************************************************************************************************
** 函数名称: armGicIntRegOffsGet
** 功能描述: 获得中断寄存器偏移
** 输　入  : uiIrqID           中断号
** 输　出  : 中断寄存器偏移
*********************************************************************************************************/
static LW_INLINE UINT32  armGicIntRegOffsGet (UINT32  uiIrqID)
{
    return  (uiIrqID / 32);
}
/*********************************************************************************************************
** 函数名称: armGicIntBitOffsGet
** 功能描述: 获得中断位偏移
** 输　入  : uiIrqID           中断号
** 输　出  : 中断位偏移
*********************************************************************************************************/
static LW_INLINE UINT32  armGicIntBitOffsGet (UINT32  uiIrqID)
{
    return  (uiIrqID & 0x1F);
}
/*********************************************************************************************************
** 函数名称: armGicIntBitMaskGet
** 功能描述: 获得中断位掩码
** 输　入  : uiIrqID           中断号
** 输　出  : 中断位掩码
*********************************************************************************************************/
static LW_INLINE UINT32  armGicIntBitMaskGet (UINT32  uiIrqID)
{
    return  (1 << armGicIntBitOffsGet(uiIrqID));
}
/*********************************************************************************************************
** 函数名称: armGicEnable
** 功能描述: 使能 GIC 发布器(使能后安全和非安全中断会传递给 CPU 接口)
** 输　入  : bEnable           是否使能
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicEnable (BOOL  bEnable)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER UINT32  uiValue = read32((addr_t)&pDistributor->uiCTLR);

    if (bEnable) {
        /*
         * Enable both secure and non-secure.
         */
        uiValue |=   BTI_MASK_CTLR_EABBLEGRP0 | BTI_MASK_CTLR_ENABLEGRP1;

    } else {
        /*
         * Clear the enable bits.
         */
        uiValue &= ~(BTI_MASK_CTLR_EABBLEGRP0 | BTI_MASK_CTLR_ENABLEGRP1);
    }
    write32(uiValue, (addr_t)&pDistributor->uiCTLR);
}
/*********************************************************************************************************
** 函数名称: armGicIrqSecuritySet
** 功能描述: 中断安全模式设置
** 输　入  : uiIrqID           中断号
**           bSecurity         安全模式
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicIrqSecuritySet (UINT32  uiIrqID, BOOL  bSecurity)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER UINT32  uiReg   = armGicIntRegOffsGet(uiIrqID);
    REGISTER UINT32  uiMask  = armGicIntBitMaskGet(uiIrqID);
    REGISTER UINT32  uiValue = read32((addr_t)&pDistributor->uiIGROUPRn[uiReg]);

    if (bSecurity) {
        uiValue |=  uiMask;
    } else {
        uiValue &= ~uiMask;
    }
    write32(uiValue, (addr_t)&pDistributor->uiIGROUPRn[uiReg]);
}
/*********************************************************************************************************
** 函数名称: armGicIrqEnable
** 功能描述: 中断使能
** 输　入  : uiIrqID           中断号
**           bEnable           是否使能
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicIrqEnable (UINT32  uiIrqID, BOOL  bEnable)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER UINT32  uiReg   = armGicIntRegOffsGet(uiIrqID);
    REGISTER UINT32  uiMask  = armGicIntBitMaskGet(uiIrqID);
    REGISTER UINT32  uiValue = 0;

    /*
     * Select set-enable or clear-enable register based on enable flag.
     */
    if (bEnable) {
        uiValue |= uiMask;
        write32(uiValue, (addr_t)&pDistributor->uiISENABLERn[uiReg]);
    } else {
        uiValue |= uiMask;
        write32(uiValue, (addr_t)&pDistributor->uiICENABLERn[uiReg]);
    }
}
/*********************************************************************************************************
** 函数名称: armGicIrqIsEnable
** 功能描述: 判断中断是否使能
** 输　入  : uiIrqID           中断号
** 输　出  : 中断是否使能
*********************************************************************************************************/
BOOL  armGicIrqIsEnable (UINT32  uiIrqID)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER UINT32  uiReg  = armGicIntRegOffsGet(uiIrqID);
    REGISTER UINT32  uiMask = armGicIntBitMaskGet(uiIrqID);

    return  ((read32((addr_t)&pDistributor->uiICENABLERn[uiReg]) & uiMask) ? LW_TRUE : LW_FALSE);
}
/*********************************************************************************************************
** 函数名称: armGicIrqPrioritySet
** 功能描述: 设置中断优先级
** 输　入  : uiIrqID           中断号
**           uiPriority        优先级(0 - 255, 0 为最高优先级)
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicIrqPrioritySet (UINT32  uiIrqID, UINT32  uiPriority)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();

    /*
     * Update the priority register. The priority registers are byte accessible, and the register
     * struct has the priority registers as a byte array, so we can just index directly by the
     * interrupt ID.
     */
    write8((uiPriority & 0xFF), (addr_t)&pDistributor->uiIPRIORITYRn[uiIrqID]);
}
/*********************************************************************************************************
** 函数名称: armGicIrqTargetSet
** 功能描述: 中断目标设置
** 输　入  : uiIrqID           中断号
**           uiCpuMask         目标处理器位组(位 0 代表 CPU 0, 依此类推）
**           bEnable           是否使能
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicIrqTargetSet (UINT32  uiIrqID, UINT32  uiCpuMask, BOOL  bEnable)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER UINT8  ucValue = read8((addr_t)&pDistributor->uiITARGETSRn[uiIrqID]);

    /*
     * Like the priority registers, the target registers are byte accessible, and the register
     * struct has the them as a byte array, so we can just index directly by the
     * interrupt ID.
     */
    if (bEnable) {
        ucValue |=  ((UINT8)uiCpuMask & 0xFF);
    } else {
        ucValue &= ~((UINT8)uiCpuMask & 0xFF);
    }

    write8(ucValue, (addr_t)&pDistributor->uiITARGETSRn[uiIrqID]);
}
/*********************************************************************************************************
** 函数名称: armGicIrqTargetGet
** 功能描述: 获得中断目标
** 输　入  : uiIrqID           中断号
**           puiCpuId          目标处理器ID(从0开始)
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicIrqTargetGet (UINT32  uiIrqID, UINT32  *puiCpuId)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER UINT8  ucValue = read8((addr_t)&pDistributor->uiITARGETSRn[uiIrqID]);

    UINT i;

    *puiCpuId = 0;

    for (i = 0; i < 8; i++) {
        if (ucValue & (1 << i)) {
            *puiCpuId = i;
            break;
        }
    }
}
/*********************************************************************************************************
** 函数名称: armGicSoftwareIntSend
** 功能描述: 发送软件中断
** 输　入  : uiIrqID           中断号
**           uiOption          选项
**           uiTargetList      目标处理器位组(位 0 代表 CPU 0, 依此类推,
**                             当 uiOption 为 GIC_SW_INT_OPTION_USE_TARGET_LIST 时有效)
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicSoftwareIntSend (UINT32  uiIrqID, UINT32  uiOption, UINT32  uiTargetList)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER UINT32  uiValue;

    uiValue =  (((UINT32)uiOption) << BIT_OFFS_SGIR_TARGETLISTFILTER)
             | (uiTargetList << BIT_OFFS_SGIR_CPUTARGETLIST)
             | (uiIrqID & 0xF);

    write32(uiValue, (addr_t)&pDistributor->uiSGIR);
}
/*********************************************************************************************************
** 函数名称: armGicCpuEnable
** 功能描述: 使能当前 CPU 到 GIC 的接口
** 输　入  : bEnable           是否使能
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicCpuEnable (BOOL  bEnable)
{
    REGISTER GIC_CPU_INTERFACE_REGS  *pInterface = armGicCpuInterfaceGet();
    REGISTER UINT32  uiValue = read32((addr_t)&pInterface->uiCTLR);

    if (bEnable) {
        uiValue |=   BTI_MASK_CTLR_ENABLES | BTI_MASK_CTLR_ENABLENS;
    } else {
        uiValue &= ~(BTI_MASK_CTLR_ENABLES | BTI_MASK_CTLR_ENABLENS);
    }

    write32(uiValue, (addr_t)&pInterface->uiCTLR);
}
/*********************************************************************************************************
** 函数名称: armGicCpuPriorityMaskSet
** 功能描述: 设置当前 CPU 的优先级掩码
** 输　入  : uiPriority        能够传递到当前 CPU 的中断的最低优先级(255 表示所有中断)
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicCpuPriorityMaskSet (UINT32  uiPriority)
{
    REGISTER GIC_CPU_INTERFACE_REGS  *pInterface = armGicCpuInterfaceGet();

    write32(uiPriority & 0xFF, (addr_t)&pInterface->uiPMR);
}
/*********************************************************************************************************
** 函数名称: armGicIrqReadAck
** 功能描述: 通知 GIC 开始处理中断
** 输　入  : NONE
** 输　出  : 当前 CPU 可以处理的最高优先级中断的中断号(1022 或 1023 时, 表示发生了一个假的中断)
**
** Normally, this function is called at the beginning of the IRQ handler. It tells the GIC
** that you are starting to handle an interupt, and returns the number of the interrupt you
** need to handle. After the interrupt is handled, you should call armGicIrqWriteDone()
** to signal that the interrupt is completely handled.
**
** In some cases, a spurious interrupt might happen. One possibility is if another CPU handles
** the interrupt. When a spurious interrupt occurs, the end of the interrupt should be indicated
** but nothing else.
**
** @return The number for the highest priority interrupt available for the calling CPU. If
**     the return value is 1022 or 1023, a spurious interrupt has occurred.
**
*********************************************************************************************************/
UINT32  armGicIrqReadAck (VOID)
{
    REGISTER GIC_CPU_INTERFACE_REGS  *pInterface = armGicCpuInterfaceGet();

    return  (read32((addr_t)&pInterface->uiIAR));
}
/*********************************************************************************************************
** 函数名称: armGicIrqWriteDone
** 功能描述: 通知 GIC 完成处理中断
** 输　入  : uiIrqID           中断号
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicIrqWriteDone (UINT32  uiIrqID)
{
    REGISTER GIC_CPU_INTERFACE_REGS  *pInterface = armGicCpuInterfaceGet();

    write32(uiIrqID, (addr_t)&pInterface->uiEOIR);
}
/*********************************************************************************************************
** 函数名称: armGicInit
** 功能描述: 初始化 GIC
** 输　入  : NONE
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicInit (VOID)
{
    REGISTER GIC_DISTRIBUTOR_REG  *pDistributor = armGicIntDistributorGet();
    REGISTER INT  i;

    armGicEnable(LW_FALSE);                             /*  First disable the distributor.              */

    for (i = 0; i < 32; i++) {
                                                        /*  Clear all pending interrupts.               */
        write32(~0, (addr_t)&pDistributor->uiICPENDRn[i]);
    }

    for (i = 0; i < 32; i++) {
                                                        /*  Disable all interrupts.                     */
        write32(~0, (addr_t)&pDistributor->uiICENABLERn[i]);
    }

    for (i = 0; i < 8; i++) {
                                                        /*  Set all interrupts to secure.               */
        write32(0, (addr_t)&pDistributor->uiIGROUPRn[i]);
    }

    armGicEnable(LW_TRUE);                              /*  Now enable the distributor.                 */
}
/*********************************************************************************************************
** 函数名称: armGicCpuInit
** 功能描述: 初始化当前 CPU 使用 GIC 接口
** 输　入  : bPreemption            是否使用中断抢占
**           uiPriority             能够传递到当前 CPU 的中断的最低优先级(255 表示所有中断)
** 输　出  : NONE
*********************************************************************************************************/
VOID  armGicCpuInit (BOOL  bPreemption,  UINT32  uiPriority)
{
    REGISTER GIC_CPU_INTERFACE_REGS  *pInterface = armGicCpuInterfaceGet();

    armGicCpuPriorityMaskSet(uiPriority);               /*  Init the GIC CPU interface.                 */

    if (bPreemption) {
        write32(0x0, (addr_t)&pInterface->uiBPR);       /*  Enable preemption.                          */
    } else {
        write32(0x7, (addr_t)&pInterface->uiBPR);       /*  Disable preemption.                         */
    }

    armGicCpuEnable(LW_TRUE);                           /*  Enable signaling the CPU.                   */
}
/*********************************************************************************************************
** 函数名称: armGicIntVecterEnable
** 功能描述: 使能指定的中断向量
** 输  入  : ulVector         向量
**           bSecurity        安全模式
**           ulPriority       优先级
**           uiCpuMask        目标处理器位组(位 0 代表 CPU 0, 依此类推）
** 输  出  : NONE
*********************************************************************************************************/
VOID  armGicIntVecterEnable (ULONG  ulVector,  BOOL  bSecurity, ULONG  ulPriority, ULONG  uiCpuMask)
{
    armGicIrqEnable(ulVector, LW_FALSE);

    armGicIrqSecuritySet(ulVector, bSecurity);

    armGicIrqPrioritySet(ulVector, ulPriority);

    armGicIrqTargetSet(ulVector, 0xFF, LW_FALSE);

    armGicIrqTargetSet(ulVector, uiCpuMask, LW_TRUE);

    armGicIrqEnable(ulVector, LW_TRUE);
}
/*********************************************************************************************************
** 函数名称: armGicIntVecterDisable
** 功能描述: 禁能指定的中断向量
** 输  入  : ulVector         向量
** 输  出  : NONE
*********************************************************************************************************/
VOID  armGicIntVecterDisable (ULONG  ulVector)
{
    armGicIrqEnable(ulVector, LW_FALSE);

    armGicIrqTargetSet(ulVector, 0xFF, LW_FALSE);
}
/*********************************************************************************************************
  END
*********************************************************************************************************/

