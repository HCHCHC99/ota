/********************************文件说明*************************************
*文件名: debug.c

*作者: Yuchen Tan

*版本: V1.0.0

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "debug.h"
#include "uart_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
#define DEBUG_INFO_LEN  (200)

#define END_OF_STRING   ('\0')
/**************************数据类型及结构定义(私有)***************************
*
*备注: 本文件中,不希望被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
char        g_DebugInfo[DEBUG_INFO_LEN];
uint16_t    g_DebugInfoIndex = 0;

uint8_t     g_TestDebugCmd = 0;
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

*函数功能描述:

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/*将整数number转换成字符串,存储在string中*/
uint16_t debug_Number_2_String(char* string, int32_t number)
{
    uint8_t i = 0;
    char Ch[10] = {0};  //32位最大值4294967295,最多占10位;
    uint8_t WeiNb = 0;
    int32_t numberAbs = 0;

    if(!string)
        return 0;
    if(number == 0)
    {
        string[0] = '0';
        string[1] = END_OF_STRING;
        return 1;
    }else
    {
        /*数字转换成字符串(位序是从低到高的倒序)*/
        numberAbs = abs(number);
        while(numberAbs)
        {
            Ch[i++] = '0' + (numberAbs % 10);
            numberAbs /= 10;
        }
        if(number < 0)
        {
            Ch[i++] = '-';
        }
        WeiNb = i;
        /*恢复正序*/
        //123|----|'3','2','1'|----|3个字符
        //-123|----|'3','2','1','-'|----|4个字符
        for(i=0 ;i<WeiNb; i++)
        {
            string[WeiNb-i-1] = Ch[i];
        }
        return WeiNb;
    }
}
/*将连续数据pData转换成字符串,存储在g_DebugInfoIndex[]中*/
void debug_Data_2_String(const void* pData, uint8_t ItemNb, uint8_t DataType)
{
    int32_t Data = 0;

    if(!pData)
        return;
    g_DebugInfoIndex = 0;
    for(uint8_t i=0; i<ItemNb; i++)
    {
        /**/
        if(DataType == 1)           //按字节
        {
            Data = *(int8_t*)pData;
            pData++;
        }else if(DataType == 2)     //按半字
        {
            Data = *(int16_t*)pData;
            pData += 2;         //pData的类型是空指针,pData++只会加1,因此这里需要手动操作其+2,否则执行*((volatile uint16_t*)u32Addr)会导致地址对齐出错,程序进入HardFault宕机
        }else if(DataType == 4)     //按字
        {
            Data = *(int32_t*)pData;
            pData += 4; //pData的类型是空指针,pData++只会加1,因此这里需要手动操作其+4,否则执行*((volatile uint32_t*)u32Addr)会导致地址对齐出错,程序进入HardFault宕机
        }else
        {
            return;
        }
        /*防止存储越界*/
        if(g_DebugInfoIndex < DEBUG_INFO_LEN - 10)  //-10: 32位数字转字符最多占10个字符
        {
            g_DebugInfoIndex += debug_Number_2_String(&g_DebugInfo[g_DebugInfoIndex], Data);
        }else
        {
            goto STRING_END;
        }
    }
STRING_END:
    g_DebugInfo[g_DebugInfoIndex] = END_OF_STRING;
}
/*调试串口打印字符串*/
void debug_Printf_String(char* pData)
{
    uint16_t ByteToSend = 0;
    char* p = pData;
    while(*p != END_OF_STRING)
    {
        p++;
        ByteToSend++;
    }
    //uart_adapter_Transmit_Polling(DEBUG_UART, (uint8_t *)pData, ByteToSend);
}
/*调试串口以字符串形式打印数据*/
void debug_Printf_Data_InString(const void* pData, uint8_t ItemNb, uint8_t DataType)
{
    debug_Data_2_String(pData, ItemNb, DataType);
    debug_Printf_String(g_DebugInfo);
}
/********************************函数定义************************************
*函数名:

*函数功能描述:

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void debug_Test(void)
{
    uint8_t DataInByte = 123;
    uint16_t DataInHalfWord = 12345;
    uint32_t DataInWord = 1234567890;

    if(g_TestDebugCmd == 1)
    {
        g_TestDebugCmd = 0;
        debug_Printf_String("asdadsasd");
    }
    if(g_TestDebugCmd == 2)
    {
        g_TestDebugCmd = 0;
        debug_Printf_Data_InString(&DataInByte, 1, 1);
    }
    if(g_TestDebugCmd == 3)
    {
        g_TestDebugCmd = 0;
        debug_Printf_Data_InString(&DataInHalfWord, 1, 2);
    }
    if(g_TestDebugCmd == 4)
    {
        g_TestDebugCmd = 0;
        debug_Printf_Data_InString(&DataInWord, 1, 4);
    }
}
