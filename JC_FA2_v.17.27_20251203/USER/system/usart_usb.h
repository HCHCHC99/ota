#ifndef USART_USB_H
#define USART_USB_H

#include "hc32f46x.h"
#include "common.h"
#include "main.h"

#define hz_usart1_debug 0
#ifdef  hz_usart1_debug
        extern uint8_t                      Enter[3];
        extern uint8_t                      Space[3];
#endif

#define USART1_IRQn_NUM                     (IRQn_Type)(Int008_IRQn)
#define USART1_RX_error_RQ_NUM              (IRQn_Type)(Int013_IRQn)
#define USART1_TX_RQ_NUM                    (IRQn_Type)(Int009_IRQn)
#define USART1_TX_complete_RQ_NUM           (IRQn_Type)(Int014_IRQn)

#define    USART1                           (M4_USART1)
/*USART1 串口 TX*/
#define USART1_TX_PORT                      (PortA)
#define USART1_TX_PIN                       (Pin00)
#define USART1_TX_FUNC                      (Func_Usart1_Tx)

/*USART1 串口 RX*/
#define USART1_RX_PORT                      (PortA)
#define USART1_RX_PIN                       (Pin01)
#define USART1_RX_FUNC                      (Func_Usart1_Rx)
// 测试数据
extern uint8_t test_data[];

bool USART1_Send_Data_IT(uint8_t *data, uint16_t length);
void USART1_usb_init(uint32_t baudtate);
void tx_ring_buffer_init(void);
bool tx_ring_buffer_put(uint8_t data);
bool tx_ring_buffer_get(uint8_t *data);
bool is_tx_ring_buffer_empty(void);
// 发送数字的函数声明
bool USART1_Send_UInt32_IT(uint32_t num);
bool USART1_Send_Int32_IT(int32_t num);
bool USART1_Send_Float_IT(float num, uint8_t decimal_places);
bool USART1_Send_Hex_IT(uint32_t num, uint8_t digits);
void USART1_Send_Test(void);
#endif
