/********************************文件说明*************************************
*文件名: crc.c

*作者: Xiaodong Qu

*版本: V1.0.0

*功能简介: 队列数据结构

*备注:

*修改履历:

*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "crc.h"
#include "uart_adapter.h"
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
uint8_t gtest_falg = 0;
uint16_t CrcData = 0;
uint16_t CrcData2 = 0;
uint32_t CrcData3 = 0;
/* 高位字节的 CRC 值 */

/********************************函数定义************************************
*函数名:

*函数功能描述: CRC数据反转

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
/* uint8数据反转 */ 
void InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf)
{
    int i;
    unsigned char temp = 0;
 
    for(i = 0; i < 8; i++)
    {
        if(SrcBuf[0] & (1 << i))
        {
            temp |= 1<<(7-i);
        }
    }
    DesBuf[0] = temp;
}

/* uint16数据反转 */ 
void InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf)
{
    int i;
    unsigned short temp = 0;
 
    for(i = 0; i < 16; i++)
    {
        if(SrcBuf[0] & (1 << i))
        {
            temp |= 1<<(15 - i);
        }
    }
    DesBuf[0] = temp;
}
/* 计算crc16 */
uint32_t G_WATCH = 0;
uint16_t crc_Cal16(uint8_t *Data, uint32_t Len, uint16_t Polynomial)	
{
	uint16_t pCRCin = CRC_INITIAL_VALUE;
	uint8_t pChar = 0;
	G_WATCH = Len;
	while (Len--)
    {
        pChar = *(Data++);
        InvertUint8(&pChar, &pChar);
        pCRCin ^= (pChar << 8);
 
        for(int i = 0; i < 8; i++)
        {
            if(pCRCin & 0x8000)
            {
                pCRCin = (pCRCin << 1) ^ Polynomial;
            }
            else
            {
                pCRCin = pCRCin << 1;
            }
        }
    }
    InvertUint16(&pCRCin, &pCRCin);
    return (pCRCin) ;
}

/* 计算crc32 */
//uint32_t crc_Cal32(uint8_t *Data, uint32_t Len, uint32_t Polynomial)	
//{
//	uint32_t crc_poly = 0x04C11DB7;  //X^32+X^26+X^23+X^22+X^16+X^12+X^11+X^10+X^8+X^7+X^5+X^4+X^2+X^1+1 total 32 effective bits without X^32. 
//	uint32_t data_t = 0xFFFFFFFF; //CRC register

//    for(uint32_t i = 0; i < Len; i++)
//    {
//    	data_t ^= Data[i]<<24; //8-bit data

//        for (uint8_t j = 0; j < 8; j++)
//        {
//            if (data_t & 0x80000000)
//            	data_t = (data_t << 1) ^ crc_poly;
//            else
//            	data_t <<= 1;
//        }
//    }
//    return (data_t);
//}
/* 查表crc16 */
uint16_t crc_SearchTable16(uint8_t *Data, uint32_t Len, uint8_t* TableHi, uint8_t* TableLo)	
{
	uint8_t uchCRCHi = 0xFF ;   /* CRC 的高字节初始化 */
    uint8_t uchCRCLo = 0xFF ;   /* CRC 的低字节初始化 */
    uint8_t uIndex ;            /* CRC 查询表索引 */
    while (Len--) /* 完成整个报文缓冲区 */
    {
        uIndex = uchCRCLo ^ *Data++ ; /* 计算 CRC */
        uchCRCLo = uchCRCHi ^ TableHi[uIndex];
        uchCRCHi = TableLo[uIndex];
    }
    return (uchCRCLo << 8 | uchCRCHi) ;
}


//uint32_t crc_SearchTable32(uint32_t *Data, uint32_t Len, uint8_t* Table)	//查表crc32
//{
//	
//}



/********************************函数定义************************************
*函数名:

*函数功能描述: 测试函数

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void crc_Test(void)
{
//	uint8_t pRev[20] = {0x01,0x02,0x03,0x04};
//	uint16_t pRevLen = 4;
//	
//	CrcData = crc_Cal16(pRev,pRevLen,0x8005);
//	CrcData2 = crc_SearchTable16(pRev,pRevLen,(uint8_t*)auchCRCHi,(uint8_t*)auchCRCLo);
//	CrcData3 = crc_Cal32(pRev,pRevLen,0x04C11DB7);
}

