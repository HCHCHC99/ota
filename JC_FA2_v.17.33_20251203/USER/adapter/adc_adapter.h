/********************************文件说明*************************************
*文件名: adc_adapter.h

*作者: Yuchen Tan

*版本: V1.0.5

*功能简介:

*备注: 无

*修改履历:
*****************************************************************************/
#ifndef ADC_ADAPTER_H_
#define ADC_ADAPTER_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "system.h"
#include "sys_def.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/

extern SystemContext g_sys_ctx;




#if (MCU_TYPE == MCU_TYPE_STM32)
/*MCU的ADC特性定义*/
#define MCU_AD_CHANNEL_NB       (19)    /*ADC通道总数(16个外部AD通道+3个内部AD通道)*/
#define ADC_FULL_SCALE          (4096)  /*ADC最大量化值(12位=4096)*/
#define ADC_REF_VOLTAGE         (3.3f)  /*ADC参考电压*/
#define ADC_REF_VOLTAGE_MUL10   (33)    /*ADC参考电压的10倍*/
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
/*MCU的ADC特性定义*/
#define MCU_AD_CHANNEL_NB       (24)    /*ADC通道总数(24个外部AD通道)*/
#define ADC_FULL_SCALE          (4096)  /*ADC最大量化值(12位=4096)*/
#define ADC_REF_VOLTAGE         (1.5f)  /*ADC参考电压*/
#define ADC_REF_VOLTAGE_MUL10   (15)    /*ADC参考电压的10倍*/
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
/*MCU的ADC特性定义*/
#define MCU_AD_CHANNEL_NB       (16)    /*ADC通道总数(16个外部AD通道)*/
#define ADC_FULL_SCALE          (4096)  /*ADC最大量化值(12位=4096)*/
#define ADC_REF_VOLTAGE         (3.3f)  /*ADC参考电压*/
#define ADC_REF_VOLTAGE_MUL10   (33)    /*ADC参考电压的10倍*/

#endif

#define SAMPLE_BUFFER_SIZE                      32




/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*与MCU平台相关的抽象类型定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
typedef struct      /*ADC通道类型定义*/
{
    ADC_HandleTypeDef*  hadc;           /*ADC通道对应的外设实例*/
    uint8_t             Channel;        /*ADC通道编号(索引号)(HAL库没有,需用户自定义(eg: 0=通道0))*/
    GPIO_TypeDef*       Port;           /*ADC通道对应的GPIO-Port*/
    uint32_t            Pin;            /*ADC通道对应的GPIO-Pin*/
}ADCH_t;
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
typedef struct      /*ADC通道类型定义*/
{
    en_adc_samp_ch_sel_t    Channel;    /*ADC通道编号*/
    en_gpio_port_t          Port;       /*ADC通道对应的GPIO-Port*/
    en_gpio_pin_t           Pin;        /*ADC通道对应的GPIO-Pin*/
}ADCH_t;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
typedef struct      /*ADC通道类型定义*/
{
    M4_ADC_TypeDef  *ADCx;              /*ADC通道对应的外设实例*/
    uint8_t         Channel;            /*ADC通道编号(索引号)*/
    en_port_t       Port;               /*ADC通道对应的GPIO-Port*/
    en_pin_t        Pin;                /*ADC通道对应的GPIO-Pin*/
}ADCH_t;

#endif

/*ADC通道采样转换状态定义*/
typedef enum
{
    E_CONVERT_IDLE,     //空闲
    E_CONVERT_START,    //启动转换
    E_CONVERTING,       //转换中
    E_CONVERT_CPLT,     //转换完成
}CONVT_STATE_t;

/*ADC单通道转换控制句柄定义*/
typedef struct
{
    /*ADC通道*/
    ADCH_t              ADChannel;          /*ADC通道(与MCU有关的抽象定义)*/
    /*ADC通道采集控制*/
    BOOL                ConvtEn;            /*ADC通道采集使能*/
    CONVT_STATE_t       State;              /*ADC通道采集状态*/
    uint16_t            SampleTime;         /*ADC通道采样累加次数*/
    uint16_t            SampledTime;        /*ADC通道已采样次数*/
    uint16_t            SampleIntervalTimer;/*ADC通道采样间隔控制计数器(1ms自加)*/
    uint16_t            SampleIntervalTime; /*ADC通道单次采样间隔(单位:ms)*/
    /*ADC通道采样结果*/
    uint16_t            ResultOnce;         /*保存最近1次转换的结果*/
    uint32_t            ResultTotal;        /*保存连续SampleTime次转换的结果之和*/
    uint16_t            ResultAverage;      /*保存连续SampleTime次转换的结果之和的平均值*/
    /* 新增：每个通道独立的缓冲区和索引 */
    uint16_t            sample_buffer[SAMPLE_BUFFER_SIZE];  // 通道专属缓冲区
    uint32_t            buffer_index;                        // 通道专属索引

    /* 新增的统计相关变量 */
    uint16_t            sample_max;         // 通道最大采样值
    uint16_t            sample_min;         // 通道最小采样值
    uint32_t            sample_total;       // 通道采样总和
    uint32_t            sample_count;       // 通道采样数量
    bool                first_sample;
}ADC_CH_CTRL_t;

/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*句柄初始化*/
void adc_adapter_hInit(ADC_CH_CTRL_t* pCHCtrl, ADCH_t* ADChannel);
/*模块功能测试*/
void adc_adapter_Test(uint8_t Channel);
/*ADC适配模块-ADC通道单次转换控制*/
void adc_adapter_Calibration(void);
void adc_adapter_Channel_Enable(uint8_t Channel);
void adc_adapter_Channel_Disable(uint8_t Channel);
void adc_adapter_SampleInterval_Timer(void);
void adc_adapter_Set_Channal_SmpTime(uint8_t Channel, uint16_t SampleTime);
void adc_adapter_Set_Channal_SmpIntervalTime(uint8_t Channel, uint16_t SampleIntervalTime);
uint16_t adc_adapter_Get_Channel_Result(uint8_t Channel);
int8_t adc_adapter_SCM_1Ch_Convert(uint8_t Channel);



int8_t hz_adc_adapter_SCM_1Ch_Convert_CUR(uint8_t Channel);
int8_t hz_adc_adapter_SCM_1Ch_Convert_VM(uint8_t Channel);
void adc_adapter_Add_Sample(uint8_t Channel, uint16_t sample_value);
uint16_t adc_adapter_Get_Sampling_Average(uint8_t Channel);
void adc_adapter_Reset_Sampling_Stats(uint8_t Channel);
void adc_adapter_Get_Sample_Buffer(uint8_t Channel, uint16_t* buffer, uint32_t* count, uint32_t max_size);
void adc_adapter_Reset_Sample_Buffer(uint8_t Channel);



// void ADC_CUR(float* CUR_Voltage_ptr, uint16_t SampleIntervalTime, uint16_t SampleTime);




/*****************************变量声明(公开)**********************************
*
*备注: 不建议用extern声明本文件的变量直接给外部使用(解耦).
*公开本文件变量建议方式: 开放返回变量值的接口.
*
*****************************************************************************/

/*****************************变量引用(全局)**********************************
*
*备注: 不建议用extern引用其他文件的变量(解耦).
*引用其他文件变量建议方式: 包含其他文件.h并调用相应接口or传参方式获取其他文件的变量
*
*****************************************************************************/

/*****************************函数引用(全局)**********************************
*
*备注: 不建议用extern引用其他文件的函数(解耦).
*引用其他文件函数的方式: 可包含其他文件.h并调用相应接口
*
*****************************************************************************/


#endif
