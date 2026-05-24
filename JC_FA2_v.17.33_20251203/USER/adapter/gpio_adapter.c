/********************************文件说明*************************************
*文件名: gpio_adapter.c

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介:

*备注: Todo: 补充GPIO引脚功能选择(输入/输出/模拟/外设复用)相关抽象代码

*修改履历:

*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "gpio_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/

/**************************数据类型及结构定义(私有)***************************
*
*备注: 本文件中,不希望被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/

/*****************************函数声明(私有)**********************************
*
*备注: 本文件中,不希望被外部调用的函数统一在这里声明
*
*****************************************************************************/

/********************************变量定义*************************************
*
*备注: 不要在.h中定义变量,防止多个.c包含这个.h,导致编译后报错变量重复定义
*
*****************************************************************************/

/********************************函数定义************************************
*函数名:

*函数功能描述: GPIO适配文件-不同MCU底层驱动适配

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*GPIO引脚模式选择*/
void gpio_adapter_Mode_Sel(GPIO_PORT_t Port, GPIO_PIN_t Pin, uint8_t Mode)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#endif
}
/*GPIO输出引脚输出高电平*/
void gpio_adapter_Set_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    Gpio_SetIO(Port, Pin);
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    PORT_SetBits(Port, Pin);
#endif
}
/*GPIO输出引脚输出低电平*/
void gpio_adapter_Reset_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    Gpio_ClrIO(Port, Pin);
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    PORT_ResetBits(Port, Pin);
#endif
}
/*GPIO输出引脚电平翻转*/
void gpio_adapter_Toggle_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_GPIO_TogglePin(Port, Pin);
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    Gpio_ToggleOutputIO(Port, Pin);
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    PORT_Toggle(Port, Pin);
#endif
}
/*GPIO输入引脚电平读取*/
GPIO_PIN_LEVEL_t gpio_adapter_Read_Pin(GPIO_PORT_t Port, GPIO_PIN_t Pin)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    if(HAL_GPIO_ReadPin(Port, Pin) == GPIO_PIN_RESET)
        return E_GPIO_PIN_RESET;
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Gpio_GetInputIO(Port, Pin) == Reset)
        return E_GPIO_PIN_RESET;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(PORT_GetBit(Port, Pin) == Reset)
        return E_GPIO_PIN_RESET;
#endif
    return E_GPIO_PIN_SET;
}
/*GPIO引脚功能选择*/
void gpio_adapter_Fuc_Sel(GPIO_PORT_t Port, GPIO_PIN_t Pin, GPIO_FUNC_t Func, uint8_t FuncEn)
{
#if (MCU_TYPE == MCU_TYPE_STM32)

#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    Gpio_SetAfMode(Port, Pin, Func);    //设置为GPIO
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
	PORT_SetFunc(Port, Pin, Func, (FuncEn ? Enable : Disable));
#endif
}
/********************************函数定义************************************
*函数名:

*函数功能描述: 串口控制器-测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void gpio_Test(void)
{
    static int8_t s_TestGpio = 0;

    if(s_TestGpio == 1)
    {
        s_TestGpio = 0;
    }
    if(s_TestGpio == 2)
    {
        s_TestGpio = 0;

    }
}
