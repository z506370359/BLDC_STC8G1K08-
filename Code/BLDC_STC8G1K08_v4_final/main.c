/*******************************************************************************
 * main.c - 无刷直流风扇电机控制器 (STC8G1K08移植版)
 *
 * 目标芯片:  STC8G1K08-38I-TSSOP20, 24MHz内部IRC, Keil C51编译器
 * 电机型号:  D8025 12V 5000RPM, 单霍尔传感器, 两步换相
 * 驱动拓扑:  6个MOS管全桥, 单霍尔A
 *
 * 移植自: PIC16F1503 项目 (本能电机 / Mr Zhong / Merlin Zeng)
 *
 * 整体架构:
 *   - PCA 8位PWM (~23.4kHz): CCP0(P1.1)/CCP1(P1.0) 驱动下桥臂MOS
 *   - INT0 (P3.2): 霍尔A边沿检测 (双沿触发, IT0=0)
 *   - 定时器0: 1ms节拍 (状态机定时、ADC采样)
 *   - 定时器1: 自由运行12T模式 (0.5μs/计数, 测量换相周期)
 *   - PCA溢出中断: 上升沿延迟/软切换定时 (23.4kHz轮询)
 *   - 比较器: 过流保护 (CMP+ P3.7, CMP- P3.6)
 *   - PI闭环转速控制
 ******************************************************************************/
#include "bldc_config.h"

/*=============================================================================
 * 全局变量
 *===========================================================================*/

/* 电机状态机 */
typedef enum {
    ST_POWER_OFF = 0,       /* 上电初始状态 */
    ST_SOFT_START,          /* 软启动 */
    ST_MOTOR_RUN,           /* 正常运行 (闭环或开环) */
    ST_MOTOR_RESTART,       /* 堵转后重启等待 */
    ST_MOTOR_OFF            /* 电机关闭, 等待输入 */
} MotorState;

MotorState motorState, lastMotorState;

/* 定时计数 */
volatile u16 fanBaseMsClk;             /* 毫秒计数器, 定时器0中断中递增 */
volatile u8  pwmInCacheClk;            /* 输入滤波定时计数 */

/* 原子操作: 清零16位定时变量时关中断, 防止写入一半被Timer0打断 */
#define ResetMsClk() do { EA=0; fanBaseMsClk=0; EA=1; } while(0)

/* 转速输入 */
u8  pwmInDuty;                         /* ADC采样得到的输入占空比 (0-100%) */
u8  rawPwmInDuty;                      /* 未经滤波的原始输入占空比 */
u8  pwmInPercentCache;                 /* 经过平滑处理的输入占空比 */

/* 转速测量 (INT0中断写入, 主循环读取, 必须volatile) */
volatile u16 phasePeriod[4];                    /* 最近4次半周期时间 (定时器1计数值) */
volatile u32 electricalCyclePeriod;             /* 4个半周期之和 (用于计算转速) */
u8  phaseChangeCnt;                    /* 换相次数计数 */

/* PWM输出 */
u8  pwmOutDuty;                        /* 当前输出占空比 (0-255) */
u8  pwmOutPercent;                     /* 当前输出百分比 (0-100) */

/* 换相状态标志 */
volatile bit bHallState;               /* 当前霍尔A电平 */
volatile bit bHallStartDelayEn;        /* 上升沿延迟激活标志 */
volatile bit bSoftSwitchDelayEn;       /* 软切换延迟激活标志 */
volatile bit bSoftSwitchDelayOn;       /* 软切换功能已启用 (经过足够换相次数后) */
volatile bit bFanLockIF;               /* 风扇堵转标志 */
volatile bit bPcaDutyWriteEn;          /* 允许PCA占空比更新标志 */
volatile u16 softSwitchTarget;                  /* 软切换触发的定时器1目标值 */
volatile u16 electricalPhasePulse;              /* 四分之一电气周期 */

/* PCA占空比缓存 (主循环写, PCA中断读, 必须volatile) */
volatile u8  pcaDutyCache;                      /* 待写入PCA的占空比值 */

/* ADC相关 */
u16 adcAccVal;                         /* ADC采样累加值 */
u8  adcCnt;                            /* ADC采样计数 */
bit bAdcFirstRun;                      /* 首次运行标志 (跳过第一次读取) */
#define ADC_SAMPLE_CNT  25             /* 每次平均的采样次数 */

/* PI控制变量 */
u8  speedPercent;                      /* 当前转速百分比 */
u8  targetSpeedPercent;                /* 目标转速百分比 */
u8  pwmInDPercent;                     /* PI用的平滑目标值 */
u16 pwm_i, pwm_p, pwm_pid;            /* 积分项、比例项、输出 */
u8  pwmIClk, pwmDClk;                 /* PI控制定时计数 */

#ifdef FAN_BACK_LD
u16 ldClk;                             /* LD反馈定时计数 */
#endif

/*=============================================================================
 * 转速查找表 (闭环模式)
 * 输入: PWM占空比百分比(0-100), 输出: 目标转速占最大转速的百分比
 *===========================================================================*/
#ifdef CLOSE_LOOP
code u8 targetSpeedList[101] = {
    /* 目前全部填100, 表示不管输入多少都全速运行
     * 如需变速控制, 修改此表, 例如: [0]=0, [10]=10, ..., [100]=100 */
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,
    100
};
#else
/* 开环模式查找表: 输入百分比 → 输出千分比 (0-1000) */
code u16 openLoopList[101] = {
      0,   0,   0,   0,   0,
     85,  85,  85,  85,  85,
     85,  93, 101, 109, 117,
    125, 132, 139, 146, 153,
    160, 168, 176, 184, 192,
    200, 208, 216, 224, 232,
    240, 248, 256, 264, 272,
    280, 288, 296, 304, 312,
    320, 328, 336, 344, 352,
    360, 370, 380, 390, 400,
    410, 420, 430, 440, 450,
    460, 468, 476, 484, 492,
    500, 510, 520, 530, 540,
    550, 562, 574, 586, 598,
    610, 620, 630, 640, 650,
    660, 671, 682, 693, 704,
    715, 728, 741, 754, 767,
    780, 794, 808, 822, 836,
    850, 866, 882, 898, 914,
    930, 944, 958, 972, 986,
    1000
};
#endif

/*=============================================================================
 * 函数前向声明
 *===========================================================================*/
void InitSystem(void);
void InitGPIO(void);
void InitPCA(void);
void InitTimer0(void);
void InitTimer1(void);
void InitADC(void);
void InitComparator(void);
void InitInterrupts(void);

void MotorOff(void);
void MotorOn(void);
void ClearFanLockFlag(void);
void WritePwmDuty(u8 duty);
void ExecuteCommutation(void);

u16  GetSpeed(void);
u8   GetSpeedPercent(void);
u8   GetPwmInDuty(void);
u8   GetRawPwmInDuty(void);
u8   GetTargetSpeedPercent(void);
u8   IsFanLock(void);

void PidControlInit(void);
void PidControl(void);

/*-----------------------------------------------------------------------------
 * 安全读取16位定时器1 - 双读校验法
 *
 * 8051读16位寄存器需要两条指令(先读高字节再读低字节), 如果读取中途
 * 定时器恰好从0x0FFF进位到0x1000, 就会读出0x0F00这样的垃圾值。
 * 解决办法: 读两次高字节, 如果前后不一致说明发生了进位, 重新读。
 *---------------------------------------------------------------------------*/
u16 ReadTimer1(void)
{
    u8 th, tl;
    do {
        th = TH1;
        tl = TL1;
    } while (th != TH1);  /* 高字节变了说明读取期间发生进位, 重读 */
    return ((u16)th << 8) | tl;
}

/*=============================================================================
 * 初始化函数
 *===========================================================================*/

/*-----------------------------------------------------------------------------
 * GPIO初始化
 * STC8G端口模式由 PnM1:PnM0 两个寄存器组合设定:
 *   00 = 准双向口 (弱上拉)
 *   01 = 推挽输出 (强驱动)
 *   10 = 高阻输入
 *   11 = 开漏输出
 *---------------------------------------------------------------------------*/
void InitGPIO(void)
{
    /* 使能扩展SFR访问 */
    P_SW2 |= 0x80;

    /* P1口配置:
     * P1.0 (CCP1/LV下管):  推挽  01
     * P1.1 (CCP0/LU下管):  推挽  01
     * P1.2 (HU上管):       推挽  01
     * P1.3 (HV上管):       推挽  01
     * P1.4 (ADC输入):      高阻  10
     * P1.5 (FG/RD反馈):    推挽  01
     * P1.6 (预留):         推挽  01
     * P1.7 (预留):         推挽  01
     */
    P1M0 = 0xEF;   /* 1110 1111 */
    P1M1 = 0x10;   /* 0001 0000 */
    P1   = 0x00;   /* 全部输出低电平 */

    /* P3口配置:
     * P3.2 (INT0/霍尔):    准双向 00 (内部弱上拉, 兼容开漏霍尔IC)
     *                       注意: 如果外部已有上拉电阻, 也可改为高阻(10)
     * P3.3:                推挽  01
     * P3.4 (CMPO):         推挽  01
     * P3.6 (CMP-基准):     高阻  10
     * P3.7 (CMP+检测):     高阻  10
     * 其余:                准双向 00
     */
    P3M0 = 0x18;   /* 0001 1000: P3.3和P3.4推挽 */
    P3M1 = 0xC0;   /* 1100 0000: P3.6和P3.7高阻, P3.2恢复为准双向(00) */
    P3   = 0x04;   /* P3.2预置高电平, 配合内部弱上拉 */

    /* P5口: P5.4和P5.5设为高阻 */
    P5M0 = 0x00;
    P5M1 = 0x30;
}

/*-----------------------------------------------------------------------------
 * PCA初始化 - 产生23.4kHz PWM驱动下桥臂MOS
 *---------------------------------------------------------------------------*/
void InitPCA(void)
{
    CR = 0;                     /* 停止PCA计数器 */
    CL = 0;
    CH = 0;

    /* CMOD: 空闲不停止, 时钟源=FOSC/4(6MHz), 使能溢出中断 */
    CMOD = 0x0B;                /* 0000 1011 */

    /* CCP0 (P1.1 = LU下管): 初始关闭PWM, 占空比=0% */
    CCAPM0 = PCA_PWM_OFF;
    CCAP0H = 0xFF;             /* 硬件反转: 0xFF = 0%占空比 */
    CCAP0L = 0xFF;
    PCA_PWM0 = 0x00;

    /* CCP1 (P1.0 = LV下管): 初始关闭PWM, 占空比=0% */
    CCAPM1 = PCA_PWM_OFF;
    CCAP1H = 0xFF;             /* 硬件反转: 0xFF = 0%占空比 */
    CCAP1L = 0xFF;
    PCA_PWM1 = 0x00;

    /* CCP2: 本项目不使用 */
    CCAPM2 = 0x00;

    pcaDutyCache = 0xFF;            /* 硬件反转: 0xFF = 0%占空比 */

    CR = 1;                     /* 启动PCA计数器 */
}

/*-----------------------------------------------------------------------------
 * 定时器0初始化 - 1ms节拍中断
 * 模式0: 16位自动重装
 * 1T模式: 计数频率 = FOSC = 24MHz
 * 重装值 = 65536 - 24000 = 41536 = 0xA240
 *---------------------------------------------------------------------------*/
void InitTimer0(void)
{
    AUXR |= 0x80;              /* T0x12 = 1, 使用1T模式 */
    TMOD &= 0xF0;              /* 定时器0模式0 (16位自动重装) */
    TH0 = 0xA2;                /* 重装值高字节 */
    TL0 = 0x40;                /* 重装值低字节 */
    TF0 = 0;                   /* 清除溢出标志 */
    TR0 = 1;                   /* 启动定时器0 */
    ET0 = 1;                   /* 使能定时器0中断 */
}

/*-----------------------------------------------------------------------------
 * 定时器1初始化 - 换相周期测量
 * 模式0: 16位自动重装 (用作自由运行计数器)
 * 12T模式: 计数频率 = FOSC/12 = 2MHz (每个计数0.5μs)
 * 重装值 = 0 (自由运行, 溢出周期 = 65536 × 0.5μs = 32.768ms)
 *---------------------------------------------------------------------------*/
void InitTimer1(void)
{
    AUXR &= ~0x40;             /* T1x12 = 0, 使用12T模式 */
    TMOD &= 0x0F;              /* 定时器1模式0 */
    TH1 = 0;                   /* 重装值 = 0 */
    TL1 = 0;
    TF1 = 0;                   /* 清除溢出标志 */
    TR1 = 1;                   /* 启动定时器1 */
    ET1 = 0;                   /* 不使能定时器1中断 (软件查询TF1) */
}

/*-----------------------------------------------------------------------------
 * ADC初始化 - 读取调速指令电压
 * 通道4 (P1.4), 右对齐, 在定时器0中断中手动触发转换
 *---------------------------------------------------------------------------*/
void InitADC(void)
{
    ADCCFG = 0x21;             /* 转换速度=01(适中), 右对齐 */
    ADC_CONTR = 0x80 | ADC_SPEED_CH;   /* 打开ADC电源, 选择通道 */
    adcAccVal = 0;
    adcCnt = 0;
    bAdcFirstRun = 1;          /* 标记首次运行 */
    ADC_CONTR |= 0x40;         /* 启动第一次转换, 结果在下次Timer0中断时读取 */
}

/*-----------------------------------------------------------------------------
 * 比较器初始化 - 过流保护
 * CMP+ = P3.7 (采样电阻上的电流信号电压)
 * CMP- = P3.6 (外部分压器提供的基准电压)
 * 当电流超过阈值(CMP+ > CMP-), 产生正向边沿中断
 *---------------------------------------------------------------------------*/
void InitComparator(void)
{
    CMPCR1 = 0x00;             /* 先关闭比较器 */
    CMPCR2 = 0x10;             /* 16个时钟周期数字滤波 */

    /* 使能比较器, 使能正向边沿中断, 负端选P3.6 */
    CMPCR1 = 0xA4;             /* 1010 0100: CMPEN=1, PIE=1, NIS=1 */
}

/*-----------------------------------------------------------------------------
 * 中断系统初始化
 *---------------------------------------------------------------------------*/
void InitInterrupts(void)
{
    /* 初始化换相周期历史记录 */
    phasePeriod[0] = 0xFFFF;
    phasePeriod[1] = 0xFFFF;
    phasePeriod[2] = 0xFFFF;
    phasePeriod[3] = 0xFFFF;
    electricalCyclePeriod = 0xFFFF;
    phaseChangeCnt = 0;

    /* INT0: 双沿触发 (STC8G中IT0=0表示双沿), 初始禁用 */
    IT0 = 0;
    EX0 = 0;                   /* MotorOn()时再使能 */

    /* 中断优先级: INT0(霍尔) = 最高 */
    PX0 = 1;
    IP |= 0x01;

    EA = 1;                    /* 开总中断 */
}

/*-----------------------------------------------------------------------------
 * 系统总初始化
 *---------------------------------------------------------------------------*/
void InitSystem(void)
{
    WDT_CONTR = 0x00;          /* 初始化期间关闭看门狗 */

    EA = 0;                    /* 初始化期间关闭总中断 */

    InitGPIO();                /* GPIO端口模式 */
    AllMosOff();               /* 确保所有MOS关断 */
    InitPCA();                 /* PCA PWM模块 */
    InitTimer0();              /* 1ms节拍定时器 */
    InitTimer1();              /* 换相计时器 */
    InitADC();                 /* 模数转换 */
    InitComparator();          /* 过流保护比较器 */
    InitInterrupts();          /* 中断配置 */
}

/*=============================================================================
 * 电机控制函数
 *===========================================================================*/

/*--- 关闭电机: 禁用霍尔中断, 关闭全部MOS ---*/
void MotorOff(void)
{
    EX0 = 0;                   /* 禁用霍尔中断 */
    CF = 0;                    /* 清PCA溢出标志 */
    AllMosOff();               /* 关闭全部MOS管 */
}

/*--- 启动电机: 使能霍尔中断, 等待第一次换相 ---*/
void MotorOn(void)
{
    IE0 = 0;                   /* 清除待处理的INT0标志 */
    EX0 = 1;                   /* 使能霍尔中断 */
    bSoftSwitchDelayOn = 0;    /* 重置软切换标志 */
}

/*--- 清除堵转标志 ---*/
void ClearFanLockFlag(void)
{
    bFanLockIF = 0;
}

/*--- 判断是否堵转 ---*/
u8 IsFanLock(void)
{
    return (bFanLockIF == 1);
}

/*--- 写入PWM占空比到缓存 ---*/
/*
 * 重要: STC8G PCA 8位PWM极性是反的!
 *   CL < CCAPnL → 输出低, CL >= CCAPnL → 输出高
 *   实际占空比 = (256 - CCAPnH) / 256
 *   即: CCAPnH=0 → 100%满占空比, CCAPnH=255 → 约0%
 *
 * 因此 pcaDutyCache 存储硬件反转值 (255 - 逻辑值),
 * pwmOutDuty 保留逻辑值(0=关, 255=满)供PI计算使用
 */
void WritePwmDuty(u8 duty_val)
{
    if (duty_val > PWM_DUTY_MAX)
        duty_val = PWM_DUTY_MAX;
    pwmOutDuty = duty_val;             /* 逻辑值: 给PI控制器用 */
    pcaDutyCache = 255 - duty_val;     /* 硬件值: 反转后写入CCAPnH */
}

/*-----------------------------------------------------------------------------
 * 执行换相 - 根据霍尔状态切换MOS导通组合
 *
 * 霍尔A高电平: HU导通 + LV输出PWM (电流从U相流向V相)
 * 霍尔A低电平: HV导通 + LU输出PWM (电流从V相流向U相)
 *
 * 原理: 先关断全部MOS, 写入占空比, 再开启对应的MOS组合
 *       上桥臂用GPIO直接控制, 下桥臂用PCA PWM控制
 *       CCAPM寄存器清零时PCA引脚恢复为GPIO, 端口锁存器预设为0, MOS自动关断
 *---------------------------------------------------------------------------*/
void ExecuteCommutation(void)
{
    AllMosOff();                /* 先关闭全部MOS, 防止直通 */

    /* 将缓存的占空比写入两个PCA通道 */
    CCAP0H = pcaDutyCache;
    CCAP1H = pcaDutyCache;

    if (bHallState) {
        /* 霍尔高: U相上管导通, V相下管输出PWM */
        PIN_HU = 1;
        CCAPM1 = PCA_PWM_ON;   /* 使能CCP1 (P1.0 = LV) */
    } else {
        /* 霍尔低: V相上管导通, U相下管输出PWM */
        PIN_HV = 1;
        CCAPM0 = PCA_PWM_ON;   /* 使能CCP0 (P1.1 = LU) */
    }
}

/*=============================================================================
 * 转速测量
 *===========================================================================*/

/*--- 获取当前转速 (RPM) ---*/
u16 GetSpeed(void)
{
    u32 tempU32;
    u32 tempPeriod;

    /* 关中断读取32位变量, 防止INT0中断在读取中途改写导致数据错乱 */
    EA = 0;
    tempPeriod = electricalCyclePeriod;
    EA = 1;

    tempU32 = SPEED_CALC_CONST;
    tempU32 /= (tempPeriod + 1);  /* +1防止除零 */
    if (tempU32 > 0xFFFF)
        return 0xFFFF;
    return (u16)tempU32;
}

/*--- 获取当前转速百分比 (相对最大转速) ---*/
u8 GetSpeedPercent(void)
{
    u16 spd = GetSpeed();
    spd /= (MAX_SPEED / 100);
    if (spd > 105)
        return 105;             /* 限制最大105%, 允许少量超调 */
    return (u8)spd;
}

/*--- 获取目标转速百分比 ---*/
u8 GetTargetSpeedPercent(void)
{
    return pwmInPercentCache;
}

/*=============================================================================
 * ADC / 转速输入处理
 *===========================================================================*/

/*--- 获取滤波后的输入占空比 ---*/
u8 GetPwmInDuty(void)
{
    return pwmInDuty;
}

/*--- 获取原始(未滤波)输入占空比 ---*/
u8 GetRawPwmInDuty(void)
{
    return rawPwmInDuty;
}

/*-----------------------------------------------------------------------------
 * ADC采样处理 - 在定时器0中断中每1ms调用一次 (非阻塞)
 *
 * 采用"读上次结果 + 启动下次转换"的异步模式, 消除了while死等:
 *   - 每次中断进来先检查上次转换是否完成
 *   - 读取结果后立即启动下次转换
 *   - 转换在后台进行, 不占用CPU时间
 *
 * 累加25次采样后取平均, 将10位ADC值(0~1023)转换为百分比(0~100)
 *---------------------------------------------------------------------------*/
void AdcProcess(void)
{
    u16 adcVal;

    /* 检查上次转换是否已完成 */
    if (!(ADC_CONTR & 0x20))
        return;                         /* 还没转完, 下次再来 */

    ADC_CONTR &= ~0x20;               /* 清除ADC_FLAG */

    /* 首次运行: 丢弃第一次结果(上电首次转换可能不准), 直接启动下次 */
    if (bAdcFirstRun) {
        bAdcFirstRun = 0;
        ADC_CONTR |= 0x40;             /* 启动下次转换 */
        return;
    }

    /* 读取10位结果 (右对齐: ADC_RES存高2位, ADC_RESL存低8位) */
    adcVal = ((u16)(ADC_RES & 0x03) << 8) | ADC_RESL;

    /* 立即启动下次转换 (转换在后台进行, 不阻塞CPU) */
    ADC_CONTR |= 0x40;

    /* 累加 */
    adcAccVal += adcVal;

    if (++adcCnt >= ADC_SAMPLE_CNT) {
        adcCnt = 0;
        /* 平均后缩放: 0-1023 → 0-100 */
        adcVal = adcAccVal / ADC_SAMPLE_CNT;
        adcAccVal = 0;
        adcVal = adcVal * 100 / 1023;
        if (adcVal > 100)
            adcVal = 100;
        rawPwmInDuty = 100 - (u8)adcVal;    /* 取反 (与PIC原版一致) */

#ifdef CLOSE_LOOP
        pwmInDuty = targetSpeedList[rawPwmInDuty]; /* 查表得到目标转速 */
#else
        pwmInDuty = rawPwmInDuty;           /* 开环直接使用 */
#endif
    }
}

/*=============================================================================
 * PI控制器 (实际为PI, 无微分项)
 *===========================================================================*/

/*--- PI控制器初始化: 积分项从当前占空比开始 ---*/
void PidControlInit(void)
{
    pwm_i = (u16)pwmOutDuty;   /* 积分项初始化为当前占空比 */
    pwmInDPercent = GetSpeedPercent();
    pwmIClk = 0;
    pwmDClk = 0;
}

/*-----------------------------------------------------------------------------
 * PI控制主函数
 *
 * 每PID_CYCLE(50ms)执行一次:
 *   1. 目标转速平滑跟踪 (30ms步进±1%)
 *   2. 读取当前转速百分比
 *   3. 计算误差, 更新积分项和比例项
 *   4. 输出占空比 = P项 + I项, 钳位到0~255
 *
 * 与PIC版的区别:
 *   PIC版所有变量除以4防溢出, 输出时左移2位; 这里直接用u16, 最大255不会溢出
 *---------------------------------------------------------------------------*/
void PidControl(void)
{
    u8 pidTempU8;

    /* 目标值平滑: 每30ms向目标靠近1% */
    if (pwmDClk >= 30) {
        pwmDClk = 0;
        if (pwmInDPercent < GetTargetSpeedPercent())
            pwmInDPercent++;
        else if (pwmInDPercent > GetTargetSpeedPercent())
            pwmInDPercent--;
    }

    /* 每PID_CYCLE毫秒执行一次PI运算 */
    if (pwmIClk >= PID_CYCLE) {
        pwmIClk = 0;
        speedPercent = GetSpeedPercent();
        targetSpeedPercent = pwmInDPercent;

        if (targetSpeedPercent >= speedPercent) {
            /* ---- 转速偏低, 需要加速 ---- */
            pidTempU8 = targetSpeedPercent - speedPercent;

            /* 积分项: 累加误差 × Ki */
            if (pidTempU8 >= 1)
                pwm_i += (u16)pidTempU8 * PWM_I_PARAMETER;
            /* 积分项上限钳位 */
            if (pwm_i > PWM_DUTY_MAX)
                pwm_i = PWM_DUTY_MAX;

            /* 比例项: 误差>3%时才生效, 避免小误差时抖动 */
            pwm_p = 0;
            if (pidTempU8 > 3)
                pwm_p = (u16)pidTempU8 * PWM_P_PARAMETER;

            /* 合成输出 */
            pwm_pid = pwm_p + pwm_i;
            if (pwm_pid > PWM_DUTY_MAX)
                pwm_pid = PWM_DUTY_MAX;

        } else {
            /* ---- 转速偏高, 需要减速 ---- */
            pidTempU8 = speedPercent - targetSpeedPercent;

            /* 减小积分项 */
            pwm_p = 0;
            if (pidTempU8 >= 1)
                pwm_p = (u16)pidTempU8 * PWM_I_PARAMETER;
            if (pwm_i < pwm_p)
                pwm_i = 0;
            else
                pwm_i -= pwm_p;

            /* 比例项 (减速方向) */
            pwm_p = 0;
            if (pidTempU8 > 3)
                pwm_p = (u16)pidTempU8 * PWM_P_PARAMETER;

            /* 合成输出 */
            if (pwm_i < pwm_p)
                pwm_pid = 0;
            else
                pwm_pid = pwm_i - pwm_p;
        }

        /* 输出占空比 */
        WritePwmDuty((u8)pwm_pid);
    }
}

/*=============================================================================
 * 中断服务程序
 *===========================================================================*/

/*-----------------------------------------------------------------------------
 * INT0中断 - 霍尔A边沿触发 (上升沿和下降沿都触发)
 *
 * 这是换相的核心中断。每次霍尔边沿到来时:
 *   1. 立即关闭全部MOS (防止直通短路)
 *   2. 读取定时器1, 记录换相周期
 *   3. 设置上升沿延迟标志 (延迟后再执行换相, 实现超前角)
 *---------------------------------------------------------------------------*/
void Int0_ISR(void) interrupt INT0_VECTOR
{
    u16 tmr1Val;

    /* 1. 第一时间关闭全部MOS, 防止上下桥臂直通 */
    AllMosOff();
    bSoftSwitchDelayEn = 0;
    bPcaDutyWriteEn = 1;

    /* 2. 读取当前霍尔电平 */
    bHallState = PIN_HALL_A;

    /* 3. FG反馈输出跟随霍尔A电平 (仅FG模式) */
#ifdef FAN_BACK_FG
    if (bHallState)
        SetFgRdLdHigh();
    else
        SetFgRdLdLow();
#endif

    /* 4. 测量换相周期 */
    if (TF1) {
        /* 定时器1溢出 = 超过32ms没有换相, 说明转速极低或已停转 */
        TF1 = 0;
        phasePeriod[(++phaseChangeCnt) & 0x03] = 0xFFFF;
    } else {
        /* 正常: 安全读取定时器1当前值作为此次半周期时间 */
        tmr1Val = ReadTimer1();
        phasePeriod[(++phaseChangeCnt) & 0x03] = tmr1Val;
    }
    /* 复位定时器1, 开始计下一个半周期 */
    TH1 = 0;
    TL1 = 0;

    /* 5. 更新电气周期 (4个半周期之和) */
    electricalCyclePeriod = (u32)phasePeriod[0] + phasePeriod[1]
                         + phasePeriod[2] + phasePeriod[3];

    /* 6. 计算四分之一电气周期, 用于软切换定时 */
    electricalPhasePulse = (u16)(electricalCyclePeriod >> 2);

    /* 7. 经过足够多次换相后才启用软切换功能 (避免启动时误触发) */
    if (phaseChangeCnt >= 0x20)
        bSoftSwitchDelayOn = 1;

    /* 8. 设置延迟标志, PCA溢出中断中会在延迟到达后执行实际换相 */
    bHallStartDelayEn = 1;
}

/*-----------------------------------------------------------------------------
 * PCA中断 - PCA计数器溢出 (~23.4kHz, 每~42.7μs触发一次)
 *
 * 负责两件事:
 *   (1) 上升沿延迟定时: 霍尔边沿后延迟RISING_DELAY(200μs)再执行换相
 *       这实现了超前角控制, 在高速时补偿霍尔传感器位置偏差
 *   (2) 软切换定时: 在接近下次换相前SOFT_SWITCH_DELAY(500μs)时
 *       自动减小PWM占空比, 降低换相瞬间的电流, 减少噪音和EMI
 *
 * 注意: PCA溢出频率约23.4kHz, 时间分辨率约42.7μs,
 *       比PIC版的TMR2(每PWM周期=40μs)精度略低, 但实测足够用
 *---------------------------------------------------------------------------*/
void PCA_ISR(void) interrupt PCA_VECTOR
{
    u16 tmr1Now;

    if (CF) {
        CF = 0;                /* 清PCA溢出标志 */

        /* --- 上升沿延迟处理 --- */
        if (bHallStartDelayEn) {
            /* 安全读取定时器1, 判断延迟是否已到 */
            tmr1Now = ReadTimer1();
            if (tmr1Now >= RISING_DELAY) {
                bHallStartDelayEn = 0;
                /* 延迟到达, 执行实际换相 */
                ExecuteCommutation();

                /* 计算软切换触发时刻 = 四分之一周期 - 500μs */
                if (electricalPhasePulse > SOFT_SWITCH_DELAY)
                    softSwitchTarget = electricalPhasePulse - SOFT_SWITCH_DELAY;
                else
                    softSwitchTarget = 0;
                bSoftSwitchDelayEn = 1;
            }
        }

        /* --- 软切换处理: 换相前减半占空比 (仅执行一次) --- */
        /*
         * PCA极性反转下的减半公式:
         *   当前逻辑占空比 = 255 - CCAPnH
         *   减半后逻辑占空比 = (255 - CCAPnH) / 2
         *   新的硬件值 = 255 - (255 - CCAPnH)/2 = 128 + CCAPnH/2
         *
         * 保护条件: 逻辑占空比 >= 0x10 (约6%) 才执行减半
         *   即: (255 - CCAPnH) >= 0x10 → CCAPnH <= 0xEF
         */
        if (bSoftSwitchDelayOn && bSoftSwitchDelayEn) {
            tmr1Now = ReadTimer1();
            if (tmr1Now >= softSwitchTarget) {
                bSoftSwitchDelayEn = 0; /* 立即清除! 保证本次换相周期内只减半一次 */
                bPcaDutyWriteEn = 0;    /* 停止正常占空比更新 */
                /* 将当前占空比减半 (极性反转公式) */
                if (CCAP0H <= 0xEF) CCAP0H = 128 + (CCAP0H >> 1);
                if (CCAP1H <= 0xEF) CCAP1H = 128 + (CCAP1H >> 1);
            }
        }

        /* --- 正常占空比更新 --- */
        if (bPcaDutyWriteEn) {
            CCAP0H = pcaDutyCache;
            CCAP1H = pcaDutyCache;
        }
    }

    /* 清除各通道中断标志 */
    if (CCF0) CCF0 = 0;
    if (CCF1) CCF1 = 0;
    if (CCF2) CCF2 = 0;
}

/*-----------------------------------------------------------------------------
 * 定时器0中断 - 1ms节拍
 *
 * 负责: 递增各种定时计数器、触发ADC采样、检测堵转
 *---------------------------------------------------------------------------*/
void Timer0_ISR(void) interrupt TIMER0_VECTOR
{
    /* 递增各定时计数器 */
    fanBaseMsClk++;
    pwmInCacheClk++;
    pwmIClk++;
    pwmDClk++;
#ifdef FAN_BACK_LD
    ldClk++;
#endif

    /* ADC采样处理 (每1ms采一次) */
    AdcProcess();

    /* 堵转检测: 定时器1溢出 = 超过32.768ms没有换相 */
    if (TF1) {
        bFanLockIF = 1;
    }
}

/*-----------------------------------------------------------------------------
 * 比较器中断 - 过流保护
 *
 * 当电流采样电压(CMP+/P3.7)超过基准电压(CMP-/P3.6)时触发
 * 立即关闭全部MOS, 并设置堵转标志触发MOTOR_RESTART状态
 *
 * 注意: 与PIC版的硬件自动关断(CWG auto-shutdown)不同,
 *       STC版通过软件中断关断, 响应延迟约2~5μs
 *       对于风扇电机这个延迟是可接受的, 如需更快响应可加外部硬件比较器
 *---------------------------------------------------------------------------*/
void Comparator_ISR(void) interrupt CMP_VECTOR
{
    CMPCR1 &= ~0x40;           /* 清除比较器中断标志 (CMPIF) */

    /* 紧急关断: 立即关闭全部MOS */
    AllMosOff();

    /* 设置堵转标志, 主循环会进入MOTOR_RESTART状态 */
    bFanLockIF = 1;

#ifdef FAN_BACK_RD
    SetFgRdLdHigh();            /* RD线输出故障信号 */
#endif
}

/*=============================================================================
 * 主函数 - 电机状态机
 *===========================================================================*/

/*--- 状态机初始化 ---*/
void MotorStateInit(void)
{
    lastMotorState = ST_POWER_OFF;
    motorState = ST_MOTOR_OFF;
}

/*-----------------------------------------------------------------------------
 * 主函数
 *
 * 状态机流程:
 *   MOTOR_OFF → (输入有效持续200ms) → SOFT_START
 *   SOFT_START → (转速达标或超时) → MOTOR_RUN
 *   MOTOR_RUN → (堵转检测) → MOTOR_RESTART
 *   MOTOR_RESTART → (等待5秒) → MOTOR_OFF
 *   任何状态 → (输入归零且转速<50%) → MOTOR_OFF
 *---------------------------------------------------------------------------*/
void main(void)
{
    InitSystem();

#ifdef FAN_BACK_RD
    SetFgRdLdLow();             /* RD线默认正常状态 */
#endif

    /* 等待ADC和系统稳定 (400ms) */
    ResetMsClk();
    while (fanBaseMsClk < 400);

    MotorStateInit();
    pwmInPercentCache = GetPwmInDuty();

    /*--- 主循环: 状态机 + PI控制 ---*/
    while (1) {

        /* 全局停机条件: 输入占空比≤3% 且 转速低于50% */
        if (GetPwmInDuty() <= 3) {
            if (GetSpeedPercent() <= 50)
                motorState = ST_MOTOR_OFF;
        }

        /* 输入平滑处理: 每30ms向实际值靠近±1% (防止突变) */
        if (pwmInCacheClk >= 30) {
            pwmInCacheClk = 0;
            if (pwmInPercentCache < GetPwmInDuty())
                pwmInPercentCache++;
            else if (pwmInPercentCache > GetPwmInDuty())
                pwmInPercentCache--;
        }

        switch (motorState) {

        /*============ 软启动: 从20%线性爬升到30%, 持续1500ms ============*/
        case ST_SOFT_START:
            if (lastMotorState != ST_SOFT_START) {
                /* 首次进入: 初始化 */
                MotorOff();
                WritePwmDuty(0);
                pwmOutPercent = 0;
                MotorOn();
                ResetMsClk();
                lastMotorState = ST_SOFT_START;
            }
            if (pwmOutPercent <= SOFT_START_END_DUTY) {
                /* 按时间线性爬升占空比 */
                pwmOutPercent = SOFT_START_START_DUTY +
                    (u8)(fanBaseMsClk / (SOFT_START_DELAY /
                    (SOFT_START_END_DUTY - SOFT_START_START_DUTY)));
                WritePwmDuty(PercentToDuty(pwmOutPercent));
                ClearFanLockFlag();     /* 软启动期间不判堵转 */
            } else {
#ifdef CLOSE_LOOP
                /* 闭环: 转速达到目标的90%就切换到运行状态 */
                if (GetSpeed() >= (u32)GetTargetSpeedPercent() *
                    (MAX_SPEED * 90UL / 100 / 100)) {
                    motorState = ST_MOTOR_RUN;
                }
#else
                /* 开环: 占空比追上输入值就切换 */
                if (pwmOutPercent + 3 >= pwmInPercentCache) {
                    motorState = ST_MOTOR_RUN;
                }
#endif
                /* 软启动期间也检查堵转 */
                if (IsFanLock())
                    motorState = ST_MOTOR_RESTART;
            }
            /* 超时保护: 软启动+500ms后强制进入运行 */
            if (fanBaseMsClk >= (u16)(SOFT_START_DELAY + 500)) {
                motorState = ST_MOTOR_RUN;
            }
            break;

        /*============ 正常运行: 闭环PI控制 或 开环查表控制 ============*/
        case ST_MOTOR_RUN:
#ifdef CLOSE_LOOP
            if (lastMotorState != ST_MOTOR_RUN) {
                /* 首次进入运行状态 */
                ResetMsClk();
                lastMotorState = ST_MOTOR_RUN;
#ifdef FAN_BACK_RD
                SetFgRdLdLow();         /* 清除故障信号 */
#endif
            }
            /* 进入运行后等200ms让转速稳定, 再启动PI */
            if (fanBaseMsClk < 200) {
                /* 稳定等待期 */
            } else {
                if (pwmIClk == 0 && pwmDClk == 0) {
                    /* 首次进入PI: 初始化积分项 */
                    PidControlInit();
                }
                PidControl();           /* 执行PI运算 */
            }
#else
            /* 开环模式 */
            if (lastMotorState != ST_MOTOR_RUN) {
                ResetMsClk();
                lastMotorState = ST_MOTOR_RUN;
                pwmInPercentCache = pwmOutPercent;
#ifdef FAN_BACK_RD
                SetFgRdLdLow();
#endif
            }
            if (fanBaseMsClk >= OPEN_LOOP_CYCLE) {
                u16 openPermil;
                ResetMsClk();
                openPermil = openLoopList[pwmInPercentCache];
                /* 千分比转8位占空比: permil × 255 / 1000 */
                WritePwmDuty((u8)((u32)openPermil * 255 / 1000));
            }
#endif
            /* 堵转保护: 检测到堵转立即进入重启 */
            if (IsFanLock())
                motorState = ST_MOTOR_RESTART;
            break;

        /*============ 堵转重启: 关机等待5秒后重试 ============*/
        case ST_MOTOR_RESTART:
            if (lastMotorState != ST_MOTOR_RESTART) {
                /* 首次进入: 关闭电机 */
                MotorOff();
                ResetMsClk();
                lastMotorState = ST_MOTOR_RESTART;
            }
#ifdef FAN_BACK_RD
            /* 重启等待期间根据输入状态控制RD线 */
            if (GetRawPwmInDuty() <= FAN_OFF_TO_START_DUTY)
                SetFgRdLdLow();         /* 输入为0, 清除故障 */
            else
                SetFgRdLdHigh();        /* 输入有效, 保持故障指示 */
#endif
            /* 5秒后回到OFF状态, 准备重新启动 */
            if (fanBaseMsClk >= RESTART_DELAY) {
                motorState = ST_MOTOR_OFF;
            }
            break;

        /*============ 电机关闭: 等待有效输入信号 ============*/
        case ST_MOTOR_OFF:
        default:
            if (lastMotorState != ST_MOTOR_OFF) {
                /* 首次进入: 确保电机完全关闭 */
                MotorOff();
                lastMotorState = ST_MOTOR_OFF;
                ResetMsClk();
            }
            /* 输入低于等于启动门限时重置计时 (不启动) */
            if (GetRawPwmInDuty() <= FAN_OFF_TO_START_DUTY) {
                ResetMsClk();
            }
            /* 输入信号持续200ms以上才启动 (防抖) */
            if (fanBaseMsClk >= 200) {
                motorState = ST_SOFT_START;
            }
            break;
        }
    }
}
