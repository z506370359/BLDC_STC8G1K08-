/*******************************************************************************
 * stc8g.h - STC8G1K08 特殊功能寄存器定义（BLDC项目精简版）
 * 
 * 目标芯片:  STC8G1K08-38I-TSSOP20
 * 
 * 说明: 仅包含本项目用到的寄存器。完整定义请使用 STC-ISP 工具自带的头文件
 ******************************************************************************/
#ifndef __STC8G_H__
#define __STC8G_H__

#include <reg51.h>
#include <intrins.h>

/*--- 端口寄存器 ---*/
sfr P1      = 0x90;
sfr P1M0    = 0x92;
sfr P1M1    = 0x91;
sfr P3      = 0xB0;
sfr P3M0    = 0xB2;
sfr P3M1    = 0xB1;
sfr P5      = 0xC8;
sfr P5M0    = 0xCA;
sfr P5M1    = 0xC9;

/*--- 端口位定义 ---*/
sbit P10    = P1^0;
sbit P11    = P1^1;
sbit P12    = P1^2;
sbit P13    = P1^3;
sbit P14    = P1^4;
sbit P15    = P1^5;
sbit P16    = P1^6;
sbit P17    = P1^7;

sbit P30    = P3^0;
sbit P31    = P3^1;
sbit P32    = P3^2;
sbit P33    = P3^3;
sbit P34    = P3^4;
sbit P35    = P3^5;
sbit P36    = P3^6;
sbit P37    = P3^7;

sbit P54    = P5^4;
sbit P55    = P5^5;

/*--- 定时器辅助寄存器 ---*/
sfr AUXR    = 0x8E;
/* AUXR 各位含义:
 *   bit7: T0x12  (1=定时器0用1T模式, 0=12T模式)
 *   bit6: T1x12  (1=定时器1用1T模式, 0=12T模式)
 *   bit5: UART_M0x6
 *   bit4: T2R    (定时器2运行控制)
 *   bit3: T2_C/T (定时器2时钟源)
 *   bit2: T2x12  (1=定时器2用1T模式)
 *   bit1: EXTRAM (0=允许访问xdata)
 *   bit0: S1ST2  (1=串口1用定时器2做波特率发生器)
 */

sfr INTCLKO = 0x8F;
sfr T2H     = 0xD6;
sfr T2L     = 0xD7;

/*--- 中断使能扩展寄存器 ---*/
sfr IE2     = 0xAF;
/* IE2 各位含义:
 *   bit2: ET2   (定时器2中断使能)
 *   bit1: ESPI  (SPI中断使能)
 *   bit0: ES2   (串口2中断使能)
 */

/*--- PCA/CCP/PWM 模块寄存器 ---*/
sfr CCON    = 0xD8;
sbit CCF0   = CCON^0;   /* CCP0 中断标志 */
sbit CCF1   = CCON^1;   /* CCP1 中断标志 */
sbit CCF2   = CCON^2;   /* CCP2 中断标志 */
sbit CR     = CCON^6;   /* PCA 计数器运行控制位 */
sbit CF     = CCON^7;   /* PCA 计数器溢出标志 */

sfr CMOD    = 0xD9;
/* CMOD 各位含义:
 *   bit7: CIDL         (空闲模式下停止PCA)
 *   bit[4:1]: CPS[2:0] PCA时钟源选择
 *     000=Fosc/12, 001=Fosc/2, 010=定时器0溢出,
 *     011=ECI外部脉冲, 100=Fosc, 101=Fosc/4, 110=Fosc/6, 111=Fosc/8
 *   bit0: ECF          (使能溢出中断)
 */

sfr CL      = 0xE9;     /* PCA 计数器低8位 */
sfr CH      = 0xF9;     /* PCA 计数器高8位 */

sfr CCAPM0  = 0xDA;     /* CCP0 工作模式寄存器 */
sfr CCAPM1  = 0xDB;     /* CCP1 工作模式寄存器 */
sfr CCAPM2  = 0xDC;     /* CCP2 工作模式寄存器 */
/* CCAPMn 各位含义:
 *   bit6: ECOM  (使能比较功能)
 *   bit5: CAPP  (上升沿捕获)
 *   bit4: CAPN  (下降沿捕获)
 *   bit3: MAT   (匹配)
 *   bit2: TOG   (翻转输出)
 *   bit1: PWM   (PWM模式使能)
 *   bit0: ECCF  (使能CCFn中断)
 */

sfr CCAP0L  = 0xEA;     /* CCP0 捕获/比较 低字节 */
sfr CCAP0H  = 0xFA;     /* CCP0 捕获/比较 高字节(PWM占空比) */
sfr CCAP1L  = 0xEB;     /* CCP1 捕获/比较 低字节 */
sfr CCAP1H  = 0xFB;     /* CCP1 捕获/比较 高字节(PWM占空比) */
sfr CCAP2L  = 0xEC;     /* CCP2 捕获/比较 低字节 */
sfr CCAP2H  = 0xFC;     /* CCP2 捕获/比较 高字节 */

sfr PCA_PWM0 = 0xF2;    /* CCP0 PWM 扩展位 */
sfr PCA_PWM1 = 0xF3;    /* CCP1 PWM 扩展位 */
sfr PCA_PWM2 = 0xF4;    /* CCP2 PWM 扩展位 */

/*--- ADC 模数转换寄存器 ---*/
sfr ADC_CONTR = 0xBC;
/* ADC_CONTR 各位含义:
 *   bit7: ADC_POWER  (ADC电源开关)
 *   bit6: ADC_START  (启动一次转换)
 *   bit5: ADC_FLAG   (转换完成标志, 软件清零)
 *   bit[3:0]: ADC_CHS (通道选择 0~14)
 */
sfr ADC_RES   = 0xBD;   /* 转换结果 高8位 */
sfr ADC_RESL  = 0xBE;   /* 转换结果 低2位 */
sfr ADCCFG    = 0xDE;
/* ADCCFG 各位含义:
 *   bit[5:4]: SPEED[1:0] (转换速度, 00最慢 11最快)
 *   bit0: RESFMT (0=左对齐, 1=右对齐)
 */

/*--- 比较器寄存器 ---*/
sfr CMPCR1  = 0xE6;
/* CMPCR1 各位含义:
 *   bit7: CMPEN   (比较器使能)
 *   bit6: CMPIF   (中断标志, 写0清除)
 *   bit5: PIE     (结果由0变1时产生中断)
 *   bit4: NIE     (结果由1变0时产生中断)
 *   bit3: PIS     (正端输入: 0=P3.7/CMP+, 1=ADC通道)
 *   bit2: NIS     (负端输入: 0=内部1.19V, 1=P3.6/CMP-)
 *   bit1: CMPOE   (结果输出到P3.4/CMPO引脚)
 *   bit0: CMPRES  (当前比较结果, 只读)
 */
sfr CMPCR2  = 0xE7;
/* CMPCR2 各位含义:
 *   bit7: INVCMPO  (反转输出极性)
 *   bit6: DISFLT   (禁用数字滤波)
 *   bit[5:0]: LCDTY (滤波时钟周期数)
 */

/*--- 外设引脚切换寄存器 ---*/
sfr P_SW2   = 0xBA;
/* P_SW2 bit7: EAXFR (使能扩展SFR区域访问) */

/*--- 中断优先级寄存器 ---*/
sfr IP      = 0xB8;
sfr IPH     = 0xB7;
sfr IP2     = 0xB5;
sfr IP2H    = 0xB6;

/*--- 看门狗控制寄存器 ---*/
sfr WDT_CONTR = 0xC1;

/*--- Keil C51 中断向量编号 ---*/
#define INT0_VECTOR      0   /* 外部中断0 */
#define TIMER0_VECTOR    1   /* 定时器0 */
#define INT1_VECTOR      2   /* 外部中断1 */
#define TIMER1_VECTOR    3   /* 定时器1 */
#define UART1_VECTOR     4   /* 串口1 */
#define ADC_VECTOR       5   /* ADC转换完成 */
#define LVD_VECTOR       6   /* 低电压检测 */
#define PCA_VECTOR       7   /* PCA模块 */
#define UART2_VECTOR     8   /* 串口2 */
#define SPI_VECTOR       9   /* SPI */
#define INT2_VECTOR     10   /* 外部中断2 */
#define INT3_VECTOR     11   /* 外部中断3 */
#define TIMER2_VECTOR   12   /* 定时器2 */
#define CMP_VECTOR      21   /* 比较器 */

/*--- 常用数据类型简写 ---*/
typedef unsigned char  u8;
typedef unsigned int   u16;
typedef unsigned long  u32;
typedef signed char    s8;
typedef signed int     s16;
typedef signed long    s32;

#endif /* __STC8G_H__ */
