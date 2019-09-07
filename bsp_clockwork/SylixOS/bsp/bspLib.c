/*********************************************************************************************************
**
**                                    中国软件开源组织
**
**                                   嵌入式实时操作系统
**
**                                SylixOS(TM)  LW : long wing
**
**                               Copyright All Rights Reserved
**
**--------------文件信息--------------------------------------------------------------------------------
**
** 文   件   名: bspLib.c
**
** 创   建   人: Han.Hui (韩辉)
**
** 文件创建日期: 2007 年 12 月 09 日
**
** 描        述: 处理器需要为 SylixOS 提供的功能支持.
*********************************************************************************************************/
#define  __SYLIXOS_KERNEL
#include "config.h"
#include "SylixOS.h"
#include "driver/include/irq_numbers.h"
#include "arch/arm/asm/hwcap.h"
#include "bspBoard.h"
#include "driver/gic/armGic.h"
#include "driver/cpu/cpu.h"
#include "arch/arm/common/cp15/armCp15.h"
#include "driver/timer/timer.h"
/*********************************************************************************************************
  BSP 信息
*********************************************************************************************************/
static const CHAR   _G_pcCpuInfo[]     = "Allwinner R16, Quad-core Cortex-A7 Up to 1.2GHz";
static const CHAR   _G_pcCacheInfo[]   = "128KBytes(D-32K/I-32K) L1-Cache per core,512KBytes L2-Cache";
static const CHAR   _G_pcPacketInfo[]  = BSP_CFG_PLATFORM_NAME;
static const CHAR   _G_pcVersionInfo[] = "BSP version 1.0.0 for "__SYLIXOS_RELSTR;
/*********************************************************************************************************
  中断相关
*********************************************************************************************************/
/*********************************************************************************************************
  精确时间换算参数
*********************************************************************************************************/
static UINT32   _G_uiFullCnt;
static UINT64   _G_ui64NSecPerCnt7;
static UINT64   _G_ui64ComparatorCur;
/*********************************************************************************************************
** 函数名称: bspIntInit
** 功能描述: 中断系统初始化
** 输  入  : NONE
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspIntInit (VOID)
{
    armGicInit();
    armHighVectorDisable();
    armGicCpuInit(LW_FALSE, 255);
    armVectorBaseAddrSet(BSP_CFG_RAM_BASE);
//    API_InterVectorSetFlag(SPI_PIO, LW_IRQ_FLAG_QUEUE);
}
/*********************************************************************************************************
** 函数名称: bspIntHandle
** 功能描述: 中断入口
** 输  入  : NONE
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspIntHandle (VOID)
{
    REGISTER UINT32  uiAck       = armGicIrqReadAck();
    REGISTER UINT32  uiSourceCpu = (uiAck >> 10) & 0x7;
    REGISTER UINT32  uiVector    = uiAck & 0x1FF;

    (VOID)uiSourceCpu;

    archIntHandle((ULONG)uiVector, LW_FALSE);
    armGicIrqWriteDone(uiAck);
}
/*********************************************************************************************************
** 函数名称: bspIntVectorEnable
** 功能描述: 使能指定的中断向量
** 输  入  : ulVector     中断向量
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspIntVectorEnable (ULONG  ulVector)
{
    armGicIntVecterEnable(ulVector, LW_FALSE, ARM_DEFAULT_INT_PRIORITY, 1 << 0);
}
/*********************************************************************************************************
** 函数名称: bspIntVectorDisable
** 功能描述: 禁能指定的中断向量
** 输  入  : ulVector     中断向量
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspIntVectorDisable (ULONG  ulVector)
{
    armGicIntVecterDisable(ulVector);
}
/*********************************************************************************************************
** 函数名称: bspIntVectorIsEnable
** 功能描述: 检查指定的中断向量是否使能
** 输  入  : ulVector     中断向量
** 输  出  : LW_FALSE 或 LW_TRUE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
BOOL  bspIntVectorIsEnable (ULONG  ulVector)
{
    return  (armGicIrqIsEnable(ulVector) ? LW_TRUE : LW_FALSE);
}
/*********************************************************************************************************
** 函数名称: bspIntVectorSetPriority
** 功能描述: 设置指定的中断向量的优先级
** 输  入  : ulVector     中断向量号
**           uiPrio       优先级
** 输　出  : ERROR CODE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
#if LW_CFG_INTER_PRIO > 0

ULONG   bspIntVectorSetPriority (ULONG  ulVector, UINT  uiPrio)
{
    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: bspIntVectorGetPriority
** 功能描述: 获取指定的中断向量的优先级
** 输  入  : ulVector     中断向量号
**           puiPrio      优先级
** 输　出  : ERROR CODE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
ULONG   bspIntVectorGetPriority (ULONG  ulVector, UINT  *puiPrio)
{
    *puiPrio = 0;
    return  (ERROR_NONE);
}

#endif                                                                  /*  LW_CFG_INTER_PRIO > 0       */
/*********************************************************************************************************
** 函数名称: bspIntVectorSetTarget
** 功能描述: 设置指定的中断向量的目标 CPU
** 输　入  : ulVector      中断向量号
**           stSize        CPU 掩码集内存大小
**           pcpuset       CPU 掩码
** 输　出  : ERROR CODE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
#if LW_CFG_INTER_TARGET > 0

ULONG   bspIntVectorSetTarget (ULONG  ulVector, size_t  stSize, const PLW_CLASS_CPUSET  pcpuset)
{
    UINT    i;

    stSize *= 8;
    stSize  = min(stSize, sizeof(UINT32) * 8);

    for (i = 0; i < stSize; i++) {
        if (LW_CPU_ISSET(i, pcpuset)) {
            armGicIntVecterEnable(ulVector, LW_FALSE, ARM_DEFAULT_INT_PRIORITY, 1UL << i);
            break;
        }
    }

    return  (ERROR_NONE);
}
/*********************************************************************************************************
** 函数名称: bspIntVectorGetTarget
** 功能描述: 获取指定的中断向量的目标 CPU
** 输　入  : ulVector      中断向量号
**           stSize        CPU 掩码集内存大小
**           pcpuset       CPU 掩码
** 输　出  : ERROR CODE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
ULONG   bspIntVectorGetTarget (ULONG  ulVector, size_t  stSize, PLW_CLASS_CPUSET  pcpuset)
{
    UINT  uiCpuId;

    armGicIrqTargetGet(ulVector, &uiCpuId);

    if (uiCpuId < (stSize * 8)) {
        uiCpuId = 0;
    }

    LW_CPU_SET(uiCpuId, pcpuset);

    return  (ERROR_NONE);
}

#endif                                                                  /*  LW_CFG_INTER_TARGET > 0     */
/*********************************************************************************************************
  BSP 信息
*********************************************************************************************************/
/*********************************************************************************************************
** 函数名称: bspInfoCpu
** 功能描述: BSP CPU 信息
** 输　入  : NONE
** 输　出  : CPU 信息
** 全局变量:
** 调用模块:
*********************************************************************************************************/
CPCHAR  bspInfoCpu (VOID)
{
    return  (_G_pcCpuInfo);
}
/*********************************************************************************************************
** 函数名称: bspInfoCache
** 功能描述: BSP CACHE 信息
** 输　入  : NONE
** 输　出  : CACHE 信息
** 全局变量:
** 调用模块:
*********************************************************************************************************/
CPCHAR  bspInfoCache (VOID)
{
    return  (_G_pcCacheInfo);
}
/*********************************************************************************************************
** 函数名称: bspInfoPacket
** 功能描述: BSP PACKET 信息
** 输　入  : NONE
** 输　出  : PACKET 信息
** 全局变量:
** 调用模块:
*********************************************************************************************************/
CPCHAR  bspInfoPacket (VOID)
{
    return  (_G_pcPacketInfo);
}
/*********************************************************************************************************
** 函数名称: bspInfoVersion
** 功能描述: BSP VERSION 信息
** 输　入  : NONE
** 输　出  : BSP VERSION 信息
** 全局变量:
** 调用模块:
*********************************************************************************************************/
CPCHAR  bspInfoVersion (VOID)
{
    return  (_G_pcVersionInfo);
}
/*********************************************************************************************************
** 函数名称: bspInfoHwcap
** 功能描述: BSP 硬件特性
** 输　入  : NONE
** 输　出  : 硬件特性 (如果支持硬浮点, 可以加入 HWCAP_VFP , HWCAP_VFPv3 , HWCAP_VFPv3D16 , HWCAP_NEON)
** 全局变量:
** 调用模块:
*********************************************************************************************************/
ULONG  bspInfoHwcap (VOID)
{
    return  (HWCAP_VFP | HWCAP_VFPv4 | HWCAP_NEON | HWCAP_LPAE);
}
/*********************************************************************************************************
** 函数名称: bspInfoRomBase
** 功能描述: BSP ROM 基地址
** 输　入  : NONE
** 输　出  : ROM 基地址
** 全局变量:
** 调用模块:
*********************************************************************************************************/
addr_t bspInfoRomBase (VOID)
{
    return  (BSP_CFG_ROM_BASE);
}
/*********************************************************************************************************
** 函数名称: bspInfoRomSize
** 功能描述: BSP ROM 大小
** 输　入  : NONE
** 输　出  : ROM 大小
** 全局变量:
** 调用模块:
*********************************************************************************************************/
size_t bspInfoRomSize (VOID)
{
    return  (BSP_CFG_ROM_SIZE);
}
/*********************************************************************************************************
** 函数名称: bspInfoRamBase
** 功能描述: BSP RAM 基地址
** 输　入  : NONE
** 输　出  : RAM 基地址
** 全局变量:
** 调用模块:
*********************************************************************************************************/
addr_t bspInfoRamBase (VOID)
{
    return  (BSP_CFG_RAM_BASE);
}
/*********************************************************************************************************
** 函数名称: bspInfoRamSize
** 功能描述: BSP RAM 大小
** 输　入  : NONE
** 输　出  : RAM 大小
** 全局变量:
** 调用模块:
*********************************************************************************************************/
size_t bspInfoRamSize (VOID)
{
    return  (BSP_CFG_RAM_SIZE);
}
/*********************************************************************************************************
  BSP HOOK
*********************************************************************************************************/
/*********************************************************************************************************
** 函数名称: bspTaskCreateHook
** 功能描述: 任务创建 hook
** 输  入  : ulId     任务 ID
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTaskCreateHook (LW_OBJECT_ID  ulId)
{
    (VOID)ulId;

    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspTaskDeleteHook
** 功能描述: 任务删除 hook
** 输  入  : ulId         任务 ID
**           pvReturnVal  返回值
**           ptcb         任务控制块
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTaskDeleteHook (LW_OBJECT_ID  ulId, PVOID  pvReturnVal, PLW_CLASS_TCB  ptcb)
{
    (VOID)ulId;
    (VOID)pvReturnVal;
    (VOID)ptcb;

    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspTaskSwapHook
** 功能描述: 任务切换 hook
** 输  入  : hOldThread       被换出的任务
**           hNewThread       要运行的任务
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTaskSwapHook (LW_OBJECT_HANDLE   hOldThread, LW_OBJECT_HANDLE   hNewThread)
{
    (VOID)hOldThread;
    (VOID)hNewThread;

    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspTaskIdleHook
** 功能描述: idle 任务调用此函数
** 输  入  : NONE
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTaskIdleHook (VOID)
{
    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspTaskIdleHook
** 功能描述: 每个操作系统时钟节拍，系统将调用这个函数
** 输  入  : i64Tick      系统当前时钟
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTickHook (INT64   i64Tick)
{
    (VOID)i64Tick;

    /*
     * 将清中断操作放在bspTIckHook()中，修复高精度定时器出现负值BUG
     */
    timerClearirq(0);
}
/*********************************************************************************************************
** 函数名称: bspWdTimerHook
** 功能描述: 看门狗定时器到时间时都会调用这个函数
** 输  入  : ulId     任务 ID
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspWdTimerHook (LW_OBJECT_ID  ulId)
{
    (VOID)ulId;

    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspTCBInitHook
** 功能描述: 初始化 TCB 时会调用这个函数
** 输  入  : ulId     任务 ID
**           ptcb     任务控制块
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTCBInitHook (LW_OBJECT_ID  ulId, PLW_CLASS_TCB   ptcb)
{
    (VOID)ulId;
    (VOID)ptcb;

    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspKernelInitHook
** 功能描述: 系统初始化完成时会调用这个函数
** 输  入  : NONE
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspKernelInitHook (VOID)
{
    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspReboot
** 功能描述: 系统重新启动
** 输　入  : iRebootType       重启类型
**           ulStartAddress    启动开始地址
** 输　出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspReboot (INT  iRebootType, addr_t  ulStartAddress)
{
    (VOID)ulStartAddress;

    /*
     * TODO: 加入你的处理代码, 建议使用硬件看门狗或硬件复位电路来复位, 如果没有, 保留下面的代码
     */

#ifdef __BOOT_INRAM
    ((VOID (*)(INT))bspInfoRamBase())(iRebootType);
#else
    ((VOID (*)(INT))bspInfoRomBase())(iRebootType);
#endif                                                                  /*  __BOOT_INRAM                */
}
/*********************************************************************************************************
** 函数名称: bspDebugMsg
** 功能描述: 打印系统调试信息
** 输　入  : pcMsg     信息
** 输　出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspDebugMsg (CPCHAR  pcMsg)
{
    /*
     * 通过 UART 打印系统调试信息
     */
    bspBoardDebugMsg(pcMsg);
}
/*********************************************************************************************************
  CACHE 相关接口
*********************************************************************************************************/
/*********************************************************************************************************
** 函数名称: bspL2CBase
** 功能描述: 获得 L2 控制器基地址
** 输  入  : pulBase      返回的基地址
** 输  出  : -1 表示不启用 L2
** 全局变量:
** 调用模块:
*********************************************************************************************************/
INT  bspL2CBase (addr_t  *pulBase)
{
    /*
     * TODO: 通过查阅处理器手册, 将 L2 控制器基地址赋给 *pulBase 变量,
     * 如果不启用 L2, 返回 PX_ERROR 即可, 如果启用, 返回 ERROR_NONE
     */
    return  (PX_ERROR);
}
/*********************************************************************************************************
** 函数名称: bspL2CAux
** 功能描述: 获得 L2 控制器 Aux 控制掩码
** 输  入  : puiAuxVal      Aux 寄存器值
**           puiAuxMask     Aux 控制掩码
** 输  出  : -1 表示不启用 L2
** 全局变量:
** 调用模块:
*********************************************************************************************************/
INT  bspL2CAux (UINT32  *puiAuxVal, UINT32  *puiAuxMask)
{
    /*
     * TODO: 通过读取硬件 Aux 寄存器, 并赋给 *puiAuxVal 变量, 并将 Aux 控制掩码赋给 *puiAuxMask 变量,
     * 如果不启用 L2, 返回 PX_ERROR 即可, 如果启用, 返回 ERROR_NONE
     */
    *puiAuxVal  =  0u;
    *puiAuxMask = ~0u;
    
    return  (ERROR_NONE);
}
/*********************************************************************************************************
  MMU 相关接口
*********************************************************************************************************/
/*********************************************************************************************************
** 函数名称: bspMmuPgdMaxNum
** 功能描述: 获得 PGD 池的数量
** 输  入  : NONE
** 输  出  : PGD 池的数量 (1 个池可映射 4GB 空间, 推荐返回 1)
** 全局变量:
** 调用模块:
*********************************************************************************************************/
ULONG  bspMmuPgdMaxNum (VOID)
{
    /*
     * TODO: 返回 PGD 池的数量, 推荐返回 1
     */
    return  (1);
}
/*********************************************************************************************************
** 函数名称: bspMmuPgdMaxNum
** 功能描述: 获得 PTE 池的数量
** 输  入  : NONE
** 输  出  : PTE 池的数量 (映射 4GB 空间, 需要 4096 个 PTE 池)
** 全局变量:
** 调用模块:
*********************************************************************************************************/
ULONG  bspMmuPteMaxNum (VOID)
{
    /*
     * TODO: 返回 PTE 池的数量 (映射 4GB 空间, 需要 4096 个 PTE 池)
     */
    return  (4096);
}

/*********************************************************************************************************
  操作系统多核接口
*********************************************************************************************************/
#define BSP_HOLDING_PEN_START    0xdeadbeef                             /*  启动从核的魔数              */
static volatile ULONG            _G_ulHoldingPen[LW_CFG_MAX_PROCESSORS] = {0};

/*
 *  标记本核启动完成
 */
VOID  bspCpuUpDone (VOID)
{
    ULONG  ulCPUId = archMpCur();

    KN_SMP_MB();
    _G_ulHoldingPen[ulCPUId] = BSP_HOLDING_PEN_START;
    KN_SMP_MB();
}

/*
 *  向指定核发送一个核间中断
 */
VOID   bspMpInt (ULONG  ulCPUId)
{
    armGicSoftwareIntSend(ARM_SW_INT_VECTOR(ulCPUId),
                          GIC_SW_INT_OPTION_USE_TARGET_LIST,
                          1 << ulCPUId);
}

/*
 *  SMP启动同步点1
 */
static INT  bspCpuUpSyncPoint1 (PVOID  pvArg)
{
    API_CacheClear(DATA_CACHE, 0, (size_t)~0);

    return  (ERROR_NONE);
}

/*
 *  SMP启动同步点2
 */
static VOID  bspCpuUpSyncPoint2 (PVOID  pvArg)
{
    ULONG  ulCPUId = (ULONG)pvArg;

    KN_SMP_MB();
    while (_G_ulHoldingPen[ulCPUId] != BSP_HOLDING_PEN_START);
}

/*
 *  SMP CACHE同步点
 */
static VOID  bspCpuCacheSync (VOID)
{
    API_CacheClear(DATA_CACHE, LW_NULL, (size_t)~0);
}

/*
 *  SMP 启动同步点
 */
VOID  bspCpuUpSync (VOID)
{
    INT              i;
    LW_CLASS_CPUSET  cpuset;

    LW_CPU_ZERO(&cpuset);

    LW_CPU_FOREACH (i) {
        LW_CPU_SET(i, &cpuset);
    }

    API_CacheBarrier(bspCpuCacheSync, LW_NULL, sizeof(cpuset), &cpuset);
}

/*
 *  启动指定cpu核
 */
VOID   bspCpuUp (ULONG  ulCPUId)
{
    INTREG  iregInterLevel;

    iregInterLevel = KN_INT_DISABLE();

    _G_ulHoldingPen[ulCPUId] = 0;
    KN_SMP_MB();

    API_KernelSmpCallAllOther(bspCpuUpSyncPoint1, NULL,
                              bspCpuUpSyncPoint2, (PVOID)ulCPUId, IPIM_OPT_NOKERN);

    bspCpuUpSyncPoint1((PVOID)ulCPUId);
    cpuStart2nd(ulCPUId);
    bspCpuUpSyncPoint2((PVOID)ulCPUId);
    KN_INT_ENABLE(iregInterLevel);
}

/*
 *  停止一个 CPU
 */
VOID   bspCpuDown (ULONG  ulCPUId)
{
    cpuDisable(ulCPUId);
}
/*********************************************************************************************************
  操作系统 CPU 速度控制接口
*********************************************************************************************************/
/*********************************************************************************************************
** 函数名称: bspSuspend
** 功能描述: 系统进入休眠状态
** 输  入  : NONE
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID    bspSuspend (VOID)
{
    /*
     * TODO: 加入你的处理代码, 如果没有, 请保留下面的调试信息
     */
    bspDebugMsg("bspSuspend() error: this BSP CAN NOT support this operate!\r\n");
}
/*********************************************************************************************************
** 函数名称: bspCpuPowerSet
** 功能描述: CPU 设置运行速度
** 输  入  : uiPowerLevel     运行速度级别
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID    bspCpuPowerSet (UINT  uiPowerLevel)
{
    /*
     * TODO: 加入你的处理代码
     */
}
/*********************************************************************************************************
** 函数名称: bspCpuPowerGet
** 功能描述: CPU 获取运行速度
** 输  入  : puiPowerLevel    运行速度级别
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID    bspCpuPowerGet (UINT  *puiPowerLevel)
{
    /*
     * TODO: 加入你的处理代码, 如果没有, 请保留下面的代码
     */
    if (puiPowerLevel) {
        *puiPowerLevel = LW_CPU_POWERLEVEL_TOP;
    }
}
/*********************************************************************************************************
  操作系统时间相关函数
*********************************************************************************************************/
/*********************************************************************************************************
  TICK 服务相关配置
*********************************************************************************************************/
#define TICK_IN_THREAD  0
#if TICK_IN_THREAD > 0
static LW_HANDLE    htKernelTicks;                                      /*  操作系统时钟服务线程句柄    */
#endif                                                                  /*  TICK_IN_THREAD > 0          */
/*********************************************************************************************************
** 函数名称: __tickThread
** 功能描述: 初始化 tick 服务线程
** 输  入  : NONE
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
#if TICK_IN_THREAD > 0

static VOID  __tickThread (VOID)
{
    for (;;) {
        API_ThreadSuspend(htKernelTicks);
        API_KernelTicks();                                              /*  内核 TICKS 通知             */
        API_TimerHTicks();                                              /*  高速 TIMER TICKS 通知       */
    }
}

#endif                                                                  /*  TICK_IN_THREAD > 0          */
/*********************************************************************************************************
** 函数名称: __tickTimerIsr
** 功能描述: tick 定时器中断服务程序
** 输  入  : NONE
** 输  出  : 中断服务返回
** 全局变量:
** 调用模块:
*********************************************************************************************************/
static irqreturn_t  __tickTimerIsr (VOID)
{
    /*
     * TODO: 通过设置硬件寄存器, 清除 tick 定时器中断
     */

    API_KernelTicksContext();                                           /*  保存被时钟中断的线程控制块  */

#if TICK_IN_THREAD > 0
    API_ThreadResume(htKernelTicks);
#else
    API_KernelTicks();                                                  /*  内核 TICKS 通知             */
    API_TimerHTicks();                                                  /*  高速 TIMER TICKS 通知       */
#endif                                                                  /*  TICK_IN_THREAD > 0          */

    return  (LW_IRQ_HANDLED);
}
/*********************************************************************************************************
** 函数名称: bspTickInit
** 功能描述: 初始化 tick 时钟
** 输  入  : NONE
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTickInit (VOID)
{
#if TICK_IN_THREAD > 0
    LW_CLASS_THREADATTR  threadattr;
#endif                                                                  /*  TICK_IN_THREAD > 0          */

#if TICK_IN_THREAD > 0
    API_ThreadAttrBuild(&threadattr, (8 * LW_CFG_KB_SIZE),
                        LW_PRIO_T_TICK,
                        LW_OPTION_THREAD_STK_CHK |
                        LW_OPTION_THREAD_UNSELECT |
                        LW_OPTION_OBJECT_GLOBAL |
                        LW_OPTION_THREAD_SAFE, LW_NULL);

    htKernelTicks = API_ThreadCreate("t_tick",
                                     (PTHREAD_START_ROUTINE)__tickThread,
                                     &threadattr,
                                     NULL);
#endif                                                                  /*  TICK_IN_THREAD > 0          */

    UINT32   uiIncrementValue;
    uiIncrementValue     = timerInputFreqGet(0) / LW_TICK_HZ;
    _G_uiFullCnt         = uiIncrementValue;
    _G_ui64NSecPerCnt7   = ((1000 * 1000 * 1000 / LW_TICK_HZ) << 7) / _G_uiFullCnt;
    _G_ui64ComparatorCur = 0;

    API_InterVectorConnect(ARM_TICK_INT_VECTOR,
                           (PINT_SVR_ROUTINE)__tickTimerIsr,
                           LW_NULL,
                           "tick_timer");

    API_InterVectorEnable(ARM_TICK_INT_VECTOR);

    timerStart(0, LW_TICK_HZ);
}
/*********************************************************************************************************
** 函数名称: bspTickHighResolution
** 功能描述: 修正从最近一次 tick 到当前的精确时间.
** 输　入  : ptv       需要修正的时间
** 输　出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspTickHighResolution (struct timespec *ptv)
{
    REGISTER  UINT32  uiCntCur;

    uiCntCur = _G_uiFullCnt - timerCurGet(0);

    /*
     *  检查是否有 TICK 中断请求
     */
    if (timerIsIrqPending(0)) {

        /*
         *  这里由于 TICK 没有及时更新, 所以需要重新获取并且加上一个 TICK 的时间
         */
        uiCntCur = _G_uiFullCnt - timerCurGet(0);

        if (uiCntCur != _G_uiFullCnt) {
            uiCntCur += _G_uiFullCnt;
        }
    }

    ptv->tv_nsec += (LONG)((_G_ui64NSecPerCnt7 * uiCntCur) >> 7);
    if (ptv->tv_nsec >= 1000000000) {
        ptv->tv_nsec -= 1000000000;
        ptv->tv_sec++;
    }
}
/*********************************************************************************************************
** 函数名称: bspDelayUs
** 功能描述: 延迟微秒
** 输  入  : ulUs     微秒数
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID bspDelayUs (ULONG ulUs)
{
    /*
     * TODO: 根据你的处理器性能, 修改为你的处理代码
     */
    volatile UINT  i;

    while (ulUs) {
        ulUs--;
        for (i = 0; i < 80; i++) {
        }
    }
}
/*********************************************************************************************************
** 函数名称: bspDelayNs
** 功能描述: 延迟纳秒
** 输  入  : ulNs     纳秒数
** 输  出  : NONE
** 全局变量:
** 调用模块:
*********************************************************************************************************/
VOID  bspDelayNs (ULONG ulNs)
{
    /*
     * TODO: 根据你的处理器性能, 修改为你的处理代码
     */
    volatile UINT  i;

    while (ulNs) {
        ulNs = (ulNs < 100) ? (0) : (ulNs - 100);
        for (i = 0; i < 40; i++) {
        }
    }
}
/*********************************************************************************************************
  semihosting
*********************************************************************************************************/
#define import(__use_no_semihosting_swi)

void  _ttywrch (int  ch)
{
    bspDebugMsg("__use_no_semihosting_swi _ttywrch()!\n");
    while (1);
}

void  _sys_exit (int  return_code)
{
    bspDebugMsg("__use_no_semihosting_swi _sys_exit()!\n");
    while (1);
}
/*********************************************************************************************************
  GCC 需求
*********************************************************************************************************/
#ifdef __GNUC__
int  __aeabi_read_tp (void)
{
    return  (0);
}
#endif                                                                  /*  __GNUC__                    */
/*********************************************************************************************************
  END
*********************************************************************************************************/
