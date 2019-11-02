/*
 * pinmux.h
 *
 *  Created on: Jun 9, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_DRIVER_PINMUX_PINMUX_H_
#define SYLIXOS_DRIVER_PINMUX_PINMUX_H_


/*********************************************************************************************************
  »ùµØÖ·¶¨Òå
*********************************************************************************************************/
#define GPIO_BASE    (0x01c20800)                                       /*  GPIO¶Ë¿ÚµÄ»ùÖ·              */
#define R_GPIO_BASE  (0x01f02c00)                                       /*  À©Õ¹GPIO¶Ë¿ÚµÄ»ùÖ·          */
/*********************************************************************************************************
  GPIO¶Ë¿Ú±àºÅºê (R16Ð¾Æ¬Ö»ÓÐ7×éGPIO)
*********************************************************************************************************/
#define GPIO_PORT_B  (1)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚB               */
#define GPIO_PORT_C  (2)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚC               */
#define GPIO_PORT_D  (3)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚD               */
#define GPIO_PORT_E  (4)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚE               */
#define GPIO_PORT_F  (5)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚF               */
#define GPIO_PORT_G  (6)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚG               */
#define GPIO_PORT_H  (7)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚH               */
#define GPIO_PORT_L  (8)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚL               */
/*********************************************************************************************************
  GPIO¹Ü½Å±àºÅºê
*********************************************************************************************************/
#define GPIO_PIN_00  (0)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ00        */
#define GPIO_PIN_01  (1)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ01        */
#define GPIO_PIN_02  (2)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ02        */
#define GPIO_PIN_03  (3)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ03        */
#define GPIO_PIN_04  (4)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ04        */
#define GPIO_PIN_05  (5)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ05        */
#define GPIO_PIN_06  (6)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ06        */
#define GPIO_PIN_07  (7)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ07        */
#define GPIO_PIN_08  (8)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ08        */
#define GPIO_PIN_09  (9)                                                /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ09        */
#define GPIO_PIN_10  (10)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ10        */
#define GPIO_PIN_11  (11)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ11        */
#define GPIO_PIN_12  (12)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ12        */
#define GPIO_PIN_13  (13)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ13        */
#define GPIO_PIN_14  (14)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ14        */
#define GPIO_PIN_15  (15)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ15        */
#define GPIO_PIN_16  (16)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ16        */
#define GPIO_PIN_17  (17)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ17        */
#define GPIO_PIN_18  (18)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ18        */
#define GPIO_PIN_19  (19)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ19        */
#define GPIO_PIN_20  (20)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ20        */
#define GPIO_PIN_21  (21)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ21        */
#define GPIO_PIN_22  (22)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ22        */
#define GPIO_PIN_23  (23)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ23        */
#define GPIO_PIN_24  (24)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ24        */
#define GPIO_PIN_25  (25)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ25        */
#define GPIO_PIN_26  (26)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ26        */
#define GPIO_PIN_27  (27)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ27        */
#define GPIO_PIN_28  (28)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ28        */
#define GPIO_PIN_29  (29)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ29        */
#define GPIO_PIN_30  (30)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ30        */
#define GPIO_PIN_31  (31)                                               /*  Ð¾Æ¬GPIO¶Ë¿ÚÒý½ÅºÅ31        */

#define GPIO_NUM(port, pin)      ((32 * (port)) + ((pin) & 0x1f))       /*  GPIO±àºÅºÏ³Éºê              */
/*********************************************************************************************************
  ¹Ü½Å¹¦ÄÜÑ¡Ôñºê
*********************************************************************************************************/
#define ALT0  0x0
#define ALT1  0x1
#define ALT2  0x2
#define ALT3  0x3
#define ALT4  0x4
#define ALT5  0x5
#define ALT6  0x6
#define ALT7  0x7

/*********************************************************************************************************
  SylixOSÖÐR16 GPIO±àºÅ
*********************************************************************************************************/
enum  sunxiR16Gpios {
    GPIO_B_00 = 0,
    GPIO_B_01,
    GPIO_B_02,
    GPIO_B_03,
    GPIO_B_04,
    GPIO_B_05,
    GPIO_B_06,
    GPIO_B_07,
    GPIO_C_00,
    GPIO_C_01,
    GPIO_C_02,
    GPIO_C_03,
    GPIO_C_04,
    GPIO_C_05,
    GPIO_C_06,
    GPIO_C_07,
    GPIO_C_08,
    GPIO_C_09,
    GPIO_C_10,
    GPIO_C_11,
    GPIO_C_12,
    GPIO_C_13,
    GPIO_C_14,
    GPIO_C_15,
    GPIO_C_16,
    GPIO_D_00,
    GPIO_D_01,
    GPIO_D_02,
    GPIO_D_03,
    GPIO_D_04,
    GPIO_D_05,
    GPIO_D_06,
    GPIO_D_07,
    GPIO_D_08,
    GPIO_D_09,
    GPIO_D_10,
    GPIO_D_11,
    GPIO_D_12,
    GPIO_D_13,
    GPIO_D_14,
    GPIO_D_15,
    GPIO_D_16,
    GPIO_D_17,
    GPIO_D_18,
    GPIO_D_19,
    GPIO_D_20,
    GPIO_D_21,
    GPIO_E_00,
    GPIO_E_01,
    GPIO_E_02,
    GPIO_E_03,
    GPIO_E_04,
    GPIO_E_05,
    GPIO_E_06,
    GPIO_E_07,
    GPIO_E_08,
    GPIO_E_09,
    GPIO_E_10,
    GPIO_E_11,
    GPIO_E_12,
    GPIO_E_13,
    GPIO_E_14,
    GPIO_E_15,
    GPIO_E_16,
    GPIO_E_17,
    GPIO_F_00,
    GPIO_F_01,
    GPIO_F_02,
    GPIO_F_03,
    GPIO_F_04,
    GPIO_F_05,
    GPIO_G_00,
    GPIO_G_01,
    GPIO_G_02,
    GPIO_G_03,
    GPIO_G_04,
    GPIO_G_05,
    GPIO_G_06,
    GPIO_G_07,
    GPIO_G_08,
    GPIO_G_09,
    GPIO_G_10,
    GPIO_G_11,
    GPIO_G_12,
    GPIO_G_13,
    GPIO_H_00,
    GPIO_H_01,
    GPIO_H_02,
    GPIO_H_03,
    GPIO_H_04,
    GPIO_H_05,
    GPIO_H_06,
    GPIO_H_07,
    GPIO_H_08,
    GPIO_H_09,
    GPIO_L_00,
    GPIO_L_01,
    GPIO_L_02,
    GPIO_L_03,
    GPIO_L_04,
    GPIO_L_05,
    GPIO_L_06,
    GPIO_L_07,
    GPIO_L_08,
    GPIO_L_09,
    GPIO_L_10,
    GPIO_L_11,
};

#define  GPIO_AMOUNT    (GPIO_L_11 + 1)
#ifndef  GPIO_NONE
#define  GPIO_NONE       LW_CFG_MAX_GPIOS
#endif

#define GET_SOC_PORT_NUM(soc)  ((soc) >> 5)
#define GET_SOC_PIN_NUM(soc)   ((soc) & 0x1f)
/*********************************************************************************************************
  º¯ÊýÉùÃ÷
*********************************************************************************************************/
INT32  gpioPinmuxSet(UINT32  uiNum, UINT32 uiCfg);
INT32  gpioPinmuxGet(UINT32  uiNum);
INT32  gpioSysToSoc(UINT32  uiNum);

#endif /* SYLIXOS_DRIVER_PINMUX_PINMUX_H_ */
