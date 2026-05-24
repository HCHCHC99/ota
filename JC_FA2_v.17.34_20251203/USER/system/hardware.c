#include "hardware.h"
#include "hc32f46x_gpio.h"
#include "main.h"
#include "adc_adapter.h"
#include "adapter_pwm.h"
#include "hz_nopwm.h"
#include "usart_usb.h"
#include "hall_position.h"
#include "power.h"
#include "motor_adc.h"
#include "dev_pwm.h"


void dev_led_init(void){
    stc_port_init_t     dev_led_init;

    // 初始化结构体清零
    MEM_ZERO_STRUCT(dev_led_init);

    // 配置PB0为输出引脚
    dev_led_init.enPinMode = Pin_Mode_Out;       // 输出模式
    dev_led_init.enPinDrv = Pin_Drv_H;           // 高驱动能力
    dev_led_init.enPullUp = Disable;             // 禁用上拉
    dev_led_init.enExInt = Disable;              // 禁用外部中断
    dev_led_init.enPinOType = Pin_OType_Cmos;    // CMOS输出类型

    // 应用PB0配置
    PORT_Init(GPIO_LED_PORT, GPIO_LED_PIN, &dev_led_init);

    PORT_OE(GPIO_LED_PORT, GPIO_LED_PIN, Enable);

    PORT_SetBits(GPIO_LED_PORT, GPIO_LED_PIN);

    PORT_Init(GPIO_LED_PORT1, GPIO_LED_PIN1, &dev_led_init);

    PORT_OE(GPIO_LED_PORT1, GPIO_LED_PIN1, Enable);

    PORT_SetBits(GPIO_LED_PORT1, GPIO_LED_PIN1);    

}

void Motor_GPIO_Init(void)
{
    stc_port_init_t     stcPortInit;
    
    // 初始化结构体清零
    MEM_ZERO_STRUCT(stcPortInit);
    
    stcPortInit.enPinMode = Pin_Mode_Out;       // 输出模式
    stcPortInit.enPinDrv = Pin_Drv_H;           // 高驱动能力
    stcPortInit.enPullUp = Disable;             // 禁用上拉
    stcPortInit.enExInt = Disable;              // 禁用外部中断
    stcPortInit.enPinOType = Pin_OType_Cmos;    // CMOS输出类型
 
        // 应用PB0配置
    PORT_Init(PortB, Pin08, &stcPortInit);

        // 使能PB0输出
    PORT_OE(PortB, Pin08, Enable);

    // 设置初始电平为低电平
    PORT_ResetBits(PortB, Pin00);    

}

void hz_gpio_PULLUP(void)
{
    stc_port_init_t     stcPortInit;
    
    // 初始化结构体清零
    MEM_ZERO_STRUCT(stcPortInit);
    
    // 配置PB12和PB13为上拉输入引脚
    stcPortInit.enPinMode = Pin_Mode_In;       // 输入模式
    stcPortInit.enPinDrv = Pin_Drv_L;          // 低驱动能力(输入模式下通常使用低驱动)
    stcPortInit.enPullUp = Enable;             // 使能上拉
    stcPortInit.enExInt = Disable;             // 禁用外部中断
    stcPortInit.enPinOType = Pin_OType_Cmos;   // CMOS类型(输入模式下此参数通常不影响)
 
    // 应用PB12配置
    PORT_Init(PortB, Pin12, &stcPortInit);

    PORT_Init(PortA, Pin03, &stcPortInit);

    
    // 应用PB13配置
    PORT_Init(PortB, Pin13, &stcPortInit);

    // 应用PB14配置
    PORT_Init(PortB, Pin14, &stcPortInit);

    // 应用PB5配置
    PORT_Init(PortB, Pin05, &stcPortInit);

    // 应用PB6配置
    PORT_Init(PortB, Pin06, &stcPortInit);

    // 应用PA8配置
    PORT_Init(PortA, Pin08, &stcPortInit);

    // 应用PA9配置
    PORT_Init(PortA, Pin09, &stcPortInit);

    // 应用PC14配置
    PORT_Init(PortC, Pin14, &stcPortInit);


    // 注意：输入模式不需要设置输出使能(OE)和初始电平
}

void Hall_GPIO_Init(void)
{
    stc_port_init_t stcPortInit;  // 定义GPIO端口初始化结构体

    MEM_ZERO_STRUCT(stcPortInit);  // 初始化结构体，清零所有成员

    /* GPIO_M1_LIMIT_BTM 和 GPIO_M1_LIMIT_TOP 初始化（输入模式，不触发外部中断） */
    stcPortInit.enPinMode = Pin_Mode_In;    // 设置为输入模式
    stcPortInit.enPullUp = Disable;         // 禁用上拉电阻
    stcPortInit.enExInt = Disable;          // 禁用外部中断功能

    // 初始化下限位引脚
    PORT_Init(HALL_UPPER_PORT, HALL_UPPER_PIN, &stcPortInit);
    // 初始化上限位引脚
    PORT_Init(HALL_LOWER_PORT, HALL_LOWER_PIN, &stcPortInit);
}

void Power_GPIO_Init(void)
{
    stc_port_init_t stcPortInit;

    // 初始化结构体清零
    MEM_ZERO_STRUCT(stcPortInit);

    // 配置为浮空输入（无上拉/下拉）
    stcPortInit.enPinMode   = Pin_Mode_In;      // 输入模式
    stcPortInit.enPinDrv    = Pin_Drv_L;        // 低驱动（输入模式下不影响）
    stcPortInit.enPullUp    = Disable;          // 禁用上拉（关键！）
    stcPortInit.enExInt     = Disable;          // 禁用外部中断（如果需要中断可修改）
    stcPortInit.enPinOType  = Pin_OType_Cmos;   // CMOS（输入模式下不影响）

    PORT_Init(POWER_POSITIVE_IO_PORT, POWER_POSITIVE_IO_PIN, &stcPortInit);

    PORT_Init(POWER_NEGATIVE_IO_PORT, POWER_NEGATIVE_IO_PIN, &stcPortInit);    
}

// void PushPullOutput_Init(void)
// {
//     stc_port_init_t stcPortInit;
    
//     // 初始化结构体清零
//     MEM_ZERO_STRUCT(stcPortInit);
    
//     stcPortInit.enPinMode = Pin_Mode_Out;       // 输出模式
//     stcPortInit.enPinDrv = Pin_Drv_H;           // 高驱动能力
//     stcPortInit.enPullUp = Disable;             // 禁用上拉
//     stcPortInit.enExInt = Disable;              // 禁用外部中断
//     stcPortInit.enPinOType = Pin_OType_Cmos;    // CMOS输出类型
 
//     // 应用PB8配置
//     PORT_Init(PUSHPULL_PLU_PORT, PUSHPULL_PLU_PIN, &stcPortInit);

//     // 使能PB8输出
//     PORT_OE(PUSHPULL_PLU_PORT, PUSHPULL_PLU_PIN, Enable);

//     // 设置初始电平为低电平
//     PORT_ResetBits(PUSHPULL_PLU_PORT, PUSHPULL_PLU_PIN);     


//     PORT_Init(PUSHPULL_PLV_PORT, PUSHPULL_PLV_PIN, &stcPortInit);


//     PORT_OE(PUSHPULL_PLV_PORT, PUSHPULL_PLV_PIN, Enable);


//     PORT_SetBits(PUSHPULL_PLV_PORT, PUSHPULL_PLV_PIN);     
// }

// void Motor_PHU_IO_Init(void)
// {
//     stc_port_init_t     stcPortInit;
    
//     // 初始化结构体清零
//     MEM_ZERO_STRUCT(stcPortInit);
//     stcPortInit.enPinMode = Pin_Mode_Out;       // 输出模式
//     stcPortInit.enPinDrv = Pin_Drv_H;           // 高驱动能力
//     stcPortInit.enPullUp = Disable;             // 禁用上拉
//     stcPortInit.enExInt = Disable;              // 禁用外部中断
//     stcPortInit.enPinOType = Pin_OType_Cmos;    // CMOS输出类型
 
//     PORT_Init(GPIO_PHU_PORT, GPIO_PHU_PIN, &stcPortInit);

//     PORT_OE(GPIO_PHU_PORT, GPIO_PHU_PIN, Enable);

//     PORT_SetBits(GPIO_PHU_PORT, GPIO_PHU_PIN);

// }
// void Motor_PHV_IO_Init(void)
// {
//     stc_port_init_t     stcPortInit;
    
//     // 初始化结构体清零
//     MEM_ZERO_STRUCT(stcPortInit);
//     stcPortInit.enPinMode = Pin_Mode_Out;       // 输出模式
//     stcPortInit.enPinDrv = Pin_Drv_H;           // 高驱动能力
//     stcPortInit.enPullUp = Disable;             // 禁用上拉
//     stcPortInit.enExInt = Disable;              // 禁用外部中断
//     stcPortInit.enPinOType = Pin_OType_Cmos;    // CMOS输出类型
 
//     PORT_Init(GPIO_PHV_PORT, GPIO_PHV_PIN, &stcPortInit);

//     PORT_OE(GPIO_PHV_PORT, GPIO_PHV_PIN, Enable);

//     PORT_SetBits(GPIO_PHV_PORT, GPIO_PHV_PIN);    
// }

void Test_An_In_Init(void)
{
    stc_port_init_t stcPortInit;
    
    // 初始化结构体
    MEM_ZERO_STRUCT(stcPortInit);
    
    // 配置引脚参数
    stcPortInit.enPinMode = Pin_Mode_Ana;        // 模拟模式
    stcPortInit.enPinDrv = Pin_Drv_L;            // 驱动能力低（模拟模式下此参数影响不大）
    stcPortInit.enPinOType = Pin_OType_Cmos;     // CMOS输出类型
    stcPortInit.enLatch = Disable;               // 禁用输入锁存
    stcPortInit.enExInt = Disable;               // 禁用外部中断
    stcPortInit.enInvert = Disable;              // 禁用输入反转
    stcPortInit.enPullUp = Disable;              // 禁用上拉电阻
    stcPortInit.enPinSubFunc = Disable;          // 禁用子功能（使用GPIO功能）
    
    // 初始化引脚
    // PORT_Init(PortA, Pin02, &stcPortInit);
    // PORT_Init(PortB, Pin13, &stcPortInit);
}


void PWM_GPIO_Init(void)
{


}
// 硬件初始化入口
void Hardware_Init(void) {
    // 初始化IO
    dev_led_init();
    // Motor_GPIO_Init();
    // Hall_GPIO_Init();    



    ADC_CUR_Init();
    ADC_VM_Init();
    // // USART1_usb_init(115200);

    Power_GPIO_Init();
    // Motor_noPWM_Init();
    // hz_gpio_PULLUP();

    // Motor_noPWM_Init(); // 上臂
    // Test_An_In_Init();
    PWM_GPIO_Init();
    // PushPullOutput_Init();    
    // Motor_PHU_IO_Init();
    // Motor_PHV_IO_Init();

}
