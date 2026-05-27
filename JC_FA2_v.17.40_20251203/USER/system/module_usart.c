#include "module_usart.h"

#include <stdint.h>
// en_result_t USART_UART_Init(M4_USART_TypeDef *USARTx,
//                                 const stc_usart_uart_init_t *pstcInitCfg)

// en_result_t USART_DeInit(M4_USART_TypeDef *USARTx)

// en_result_t USART_FuncCmd(M4_USART_TypeDef *USARTx,
//                                 en_usart_func_t enFunc,
//                                 en_functional_state_t enCmd)


// en_result_t USART_SendData(M4_USART_TypeDef *USARTx, uint16_t u16Data)



typedef enum{
    USART_1,
    USART_MAX
}USER_USART_NAME_t;

uint32_t USART_NAME[USART_MAX]={0};

typedef struct{
    // 串口名
    // 发送引脚
    // 接收引脚
    // 波特率
    // 位数
    // 发送模式
    // 接收模式
    // 缓冲区：环形缓冲区 256字节
}USER_USART_CONFIG_t;

USER_USART_CONFIG_t USER_usart[USART_MAX];

void usart_config_init(void){
    // 根据串口名 ，使能时钟
    // 串口结构体初始化
    // 使能串口
    
}

void module_usart_init(void){
    // 初始化 串口的名称
    // 引脚
    // 波特率
    // 发送模式
    // 接收模式
}

