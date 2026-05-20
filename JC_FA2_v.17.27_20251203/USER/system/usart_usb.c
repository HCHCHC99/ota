#include "usart_usb.h"
#include "stdbool.h"
#include "stdint.h"
#include "gpio_adapter.h"
#include "hz_timer.h"
// 环形缓冲区函数声明

static void Usart1RxIrqCallback(void);
static void Usart1TxIrqCallback(void);
static void Usart1TxCmpltIrqCallback(void);
static void Usart1ErrIrqCallback(void);
// 添加环形缓冲区定义
#define TX_RING_BUFFER_SIZE 128
static uint8_t test_data[] = "time:";
#ifdef hz_usart1_debug
    uint8_t          Enter[3] = "\r";
    uint8_t          Space[3] = "  ";
#endif

typedef struct {
    uint8_t buffer[TX_RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} tx_ring_buffer_t;

volatile tx_ring_buffer_t tx_ring_buf = {0};

// 初始化环形缓冲区
void tx_ring_buffer_init(void)
{
    tx_ring_buf.head = 0;
    tx_ring_buf.tail = 0;
}

// 检查环形缓冲区是否为空
bool is_tx_ring_buffer_empty(void)
{
    return (tx_ring_buf.head == tx_ring_buf.tail);
}

// 向环形缓冲区写入数据（原子操作：禁用中断避免被ISR打断）
bool tx_ring_buffer_put(uint8_t data)
{
    bool result = false;
    __disable_irq(); // 禁用全局中断（Cortex-M intrinsic函数）
    {
        uint16_t next_head = (tx_ring_buf.head + 1) % TX_RING_BUFFER_SIZE;
        if(next_head != tx_ring_buf.tail) // 缓冲区未满
        {
            tx_ring_buf.buffer[tx_ring_buf.head] = data;
            tx_ring_buf.head = next_head;
            result = true;
        }
    }
    __enable_irq(); // 恢复全局中断
    return result;
}


bool tx_ring_buffer_get(uint8_t *data)
{
    bool result = false;
    __disable_irq(); // 禁用全局中断
    {
        if(!is_tx_ring_buffer_empty()) // 缓冲区非空
        {
            *data = tx_ring_buf.buffer[tx_ring_buf.tail];
            tx_ring_buf.tail = (tx_ring_buf.tail + 1) % TX_RING_BUFFER_SIZE;
            result = true;
        }
    }
    __enable_irq(); // 恢复全局中断
    return result;
}

// 修改TX中断回调函数
static void Usart1TxIrqCallback(void)
{
    uint8_t data;
    
    // 如果TX数据寄存器为空且有数据要发送
    if(USART_GetStatus(USART1, UsartTxEmpty) == Set)
    {
        // 从环形缓冲区获取数据并发送
        if(tx_ring_buffer_get(&data))
        {
            USART_SendData(USART1, data);
        }
        else
        {
            // 缓冲区为空，禁用TX空中断
            USART_FuncCmd(USART1, UsartTxEmptyInt, Disable);
        }
    }
}

bool USART1_Send_Data_IT(uint8_t *data, uint16_t length)
{
    // 1. 先将数据写入环形缓冲区（循环写入，满则返回失败）
    for(uint16_t i = 0; i < length; i++)
    {
        if(!tx_ring_buffer_put(data[i])) {
            return false; // 缓冲区已满，发送失败
        }
    }

    // 2. 先使能TX空中断（关键：确保后续TX空时能触发中断）
    USART_FuncCmd(USART1, UsartTxEmptyInt, Enable);

    // 3. 若TX寄存器已空，主动发送第一个字节（避免首次中断不触发）
    if(USART_GetStatus(USART1, UsartTxEmpty) == Set)
    {
        uint8_t first_data;
        if(tx_ring_buffer_get(&first_data)) // 从缓冲区取第一个字节
        {
            USART_SendData(USART1, first_data); // 主动写入TX寄存器，启动发送
        }
    }

    return true;
}
static void Usart1RxIrqCallback(void)
{
    uint16_t u16Data = USART_RecData(USART1);
	switch(u16Data)
	{
		// case 0x02 :
        //      break;
	}
}
static void Usart1ErrIrqCallback(void)
{
    if (Set == USART_GetStatus(USART1, UsartFrameErr))
    {
        USART_ClearStatus(USART1, UsartFrameErr);
    }

    if (Set == USART_GetStatus(USART1, UsartParityErr))
    {
        USART_ClearStatus(USART1, UsartParityErr);
    }

    if (Set == USART_GetStatus(USART1, UsartOverrunErr))
    {
        USART_ClearStatus(USART1, UsartOverrunErr);
    }
}

static void Usart1TxCmpltIrqCallback(void)
{

	// USART_FuncCmd(USART1, UsartTx, Disable);
	USART_FuncCmd(USART1, UsartTxCmpltInt, Disable);
}

void USART1_usb_init(uint32_t baudtate)
{
//		en_result_t enRet = Ok;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkOutput,
        UsartClkDiv_1,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSamleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };
	/* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc((en_port_t)USART1_RX_PORT, (en_pin_t)USART1_RX_PIN, Func_Usart1_Rx, Disable);
    PORT_SetFunc((en_port_t)USART1_TX_PORT, (en_pin_t)USART1_TX_PIN, Func_Usart1_Tx, Disable);

    /* Initialize UART */
    USART_UART_Init(USART1, &stcInitCfg);  
    /* Set baudrate */
    USART_SetBaudrate(USART1, baudtate);	
		stc_irq_regi_conf_t stcIrqRegiCfg;
    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = USART1_IRQn_NUM;
    stcIrqRegiCfg.pfnCallback = &Usart1RxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART RX error IRQ */
   stcIrqRegiCfg.enIRQn = USART1_RX_error_RQ_NUM;
    stcIrqRegiCfg.pfnCallback = &Usart1ErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
    /* Set USART TX IRQ */

    stcIrqRegiCfg.enIRQn = USART1_TX_RQ_NUM;
    stcIrqRegiCfg.pfnCallback = &Usart1TxIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART TX complete IRQ */

    stcIrqRegiCfg.enIRQn = USART1_TX_complete_RQ_NUM;
    stcIrqRegiCfg.pfnCallback = &Usart1TxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = INT_USART1_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
    
    // 初始化环形缓冲区
    tx_ring_buffer_init();


    /*Enable RX && RX interupt && timeout interrupt function*/
    USART_FuncCmd(USART1, UsartRx, Enable);
    USART_FuncCmd(USART1, UsartRxInt, Enable);
    USART_FuncCmd(USART1, UsartTimeOut, Enable);
    USART_FuncCmd(USART1, UsartTimeOutInt, Enable);
    USART_FuncCmd(USART1, UsartTx, Enable);
    USART_FuncCmd(USART1, UsartTxEmptyInt, Disable);	
}
// 发送无符号整数（十进制）
bool USART1_Send_UInt32_IT(uint32_t num)
{
    char buffer[11]; // 最大32位无符号整数是10位数字
    uint8_t len = 0;
    uint8_t i;
    
    // 处理0的特殊情况
    if(num == 0)
    {
        buffer[len++] = '0';
    }
    else
    {
        // 数字转换为字符串（逆序）
        while(num > 0)
        {
            buffer[len++] = (num % 10) + '0';
            num /= 10;
        }
        
        // 反转字符串
        for(i = 0; i < len / 2; i++)
        {
            char temp = buffer[i];
            buffer[i] = buffer[len - 1 - i];
            buffer[len - 1 - i] = temp;
        }
    }
    
    // 发送转换后的字符串
    return USART1_Send_Data_IT((uint8_t*)buffer, len);
}

// 发送有符号整数（十进制）
bool USART1_Send_Int32_IT(int32_t num)
{
    char buffer[12]; // 包含符号位，最大11位
    uint8_t len = 0;
    uint8_t i;
    uint32_t n;
    
    // 处理负数
    if(num < 0)
    {
        buffer[len++] = '-';
        n = (uint32_t)(-num);
    }
    else
    {
        n = (uint32_t)num;
    }
    
    // 处理0的特殊情况
    if(n == 0)
    {
        buffer[len++] = '0';
    }
    else
    {
        // 数字转换为字符串（逆序）
        while(n > 0)
        {
            buffer[len++] = (n % 10) + '0';
            n /= 10;
        }
        
        // 反转字符串（跳过符号位）
        uint8_t start = (num < 0) ? 1 : 0;
        for(i = start; i < (len + start) / 2; i++)
        {
            char temp = buffer[i];
            buffer[i] = buffer[len - 1 - (i - start)];
            buffer[len - 1 - (i - start)] = temp;
        }
    }
    
    // 发送转换后的字符串
    return USART1_Send_Data_IT((uint8_t*)buffer, len);
}

// 发送浮点数（保留指定小数位数）
bool USART1_Send_Float_IT(float num, uint8_t decimal_places)
{
    char buffer[20];
    uint8_t len = 0;
    int32_t integer_part;
    float fractional_part;
    uint8_t i;
    
    // 处理负数
    if(num < 0)
    {
        buffer[len++] = '-';
        num = -num;
    }
    
    // 分离整数部分和小数部分
    integer_part = (int32_t)num;
    fractional_part = num - integer_part;
    
    // 处理整数部分为0的情况
    if(integer_part == 0)
    {
        buffer[len++] = '0';
    }
    else
    {
        // 转换整数部分（逆序）
        uint32_t temp = (uint32_t)integer_part;
        uint8_t int_len = 0;
        char int_buffer[10];
        
        while(temp > 0)
        {
            int_buffer[int_len++] = (temp % 10) + '0';
            temp /= 10;
        }
        
        // 反转并复制到缓冲区
        for(i = 0; i < int_len; i++)
        {
            buffer[len++] = int_buffer[int_len - 1 - i];
        }
    }
    
    // 添加小数点
    if(decimal_places > 0)
    {
        buffer[len++] = '.';
        
        // 处理小数部分
        for(i = 0; i < decimal_places; i++)
        {
            fractional_part *= 10;
            uint8_t digit = (uint8_t)fractional_part;
            buffer[len++] = digit + '0';
            fractional_part -= digit;
        }
    }
    
    // 发送转换后的字符串
    return USART1_Send_Data_IT((uint8_t*)buffer, len);
}

// 发送十六进制数
bool USART1_Send_Hex_IT(uint32_t num, uint8_t digits)
{
    char buffer[9]; // 最大32位十六进制数是8位
    uint8_t len = 0;
    uint8_t i;
    const char hex_chars[] = "0123456789ABCDEF";
    
    // 确保位数在有效范围内
    if(digits == 0 || digits > 8)
        digits = 8;
    
    // 转换为十六进制字符串（逆序）
    for(i = 0; i < digits; i++)
    {
        buffer[len++] = hex_chars[(num >> (4 * i)) & 0x0F];
    }
    
    // 反转字符串
    for(i = 0; i < len / 2; i++)
    {
        char temp = buffer[i];
        buffer[i] = buffer[len - 1 - i];
        buffer[len - 1 - i] = temp;
    }
    
    // 发送转换后的字符串
    return USART1_Send_Data_IT((uint8_t*)buffer, len);
}

void USART1_Send_Test(void)
{
    while(!USART1_Send_Data_IT(test_data, sizeof(test_data) - 1))
    {
        // 可选：短暂延时，避免CPU空转
        tickTimer_DelayMs(1);
    }
        tickTimer_DelayMs(2);

        // 发送无符号整数
        USART1_Send_UInt32_IT(12345);tickTimer_DelayMs(2);
        USART1_Send_Data_IT("\n",2);tickTimer_DelayMs(2);
    
        // 发送有符号整数
        USART1_Send_Int32_IT(-6789);tickTimer_DelayMs(2);
        USART1_Send_Data_IT("\r",2);tickTimer_DelayMs(2);
        // 发送浮点数，保留2位小数
        USART1_Send_Float_IT(3.14159, 2);tickTimer_DelayMs(2);

        // 发送十六进制数，使用4位表示
        USART1_Send_Hex_IT(0xABCD, 4);tickTimer_DelayMs(2);
}
