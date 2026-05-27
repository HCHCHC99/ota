/********************************文件说明*************************************
*文件名: flash_adapter.c

*作者: Yuchen Tan

*版本: V1.0.2

*功能简介: 实现不同mcu的flash擦,读,写操作抽象接口(介于应用功能和MCU的FLASH底层驱动之间的中间层)

*备注:
1.此模块进行FLASH读写测试的默认地址TEST_ADDR是FLASH的倒数第4页,后续程序写的越多,
    TEST_ADDR就需要向下移动(距FLASH末尾至少要留2页),如果不够2页则必须屏蔽测试功能;
2.写操作只能将位从1变0,故对同一个FLASH单元不擦除情况下重复写(先写data1,在写data2),
    最终此单元的值将变为(data1 & data2)(eg:先写0x02,在写0x04,则最终此单元值=0x00),
    重复写不会导致MCU出现问题或报错,因此写操作前不要忘记擦除,否则写入数据可能会出错;

*修改履历:
------------------------------------V1.0.1------------------------------------
*20220920:
1.增加适配HC32F460芯片的代码.
2.将原先flash锁/解锁的操作从flash擦写操作适配函数中(eg: flash_adapter_Write_Word)
单独提取出来,变成flash_adapter_Unlock()和flash_adapter_Lock()这2个内联函数,放到擦
写接口中调用(eg: flash_adapter_Write),提高连续擦写的调用效率.
------------------------------------V1.0.2------------------------------------
*20230224: 增加接口flash_adapter_Init().
*****************************************************************************/

/*****************************文件包含(私有)**********************************
*
*备注: 无
*
*****************************************************************************/
#include "flash_adapter.h"
/*****************************宏定义(私有)************************************
*
*备注: 本文件中,不希望被外部使用或随意修改的宏在这里定义
*
*****************************************************************************/
/*MCU-ADC外设相关底层定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)

#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
//注1: STM32的ADC-频率/采样时间/转换时间    \
 1.HC32F030F8TA的FLASH范围0X0000000-0X0000FFFF,128页,每页大小0X200(512字节);\
 2.从map文件可以看出,编译后程序存放地址从0X00000000开始向下生长;

#elif (MCU_TYPE == MCU_TYPE_HC32_F4)

#endif

//模块测试宏定义
#define USE_FLASH_TEST  (0)
#define TEST_ADDR       BLOCK_L5_ADDR       //倒数第4页
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
uint8_t g_TestFlashOpCmd = 0;   //用于测试模块功能
/********************************函数定义************************************
*函数名:

*函数功能描述: FLASH适配模块-FLASH底层驱动适配

*函数参数:

*函数返回值: 0-失败  1-成功

*备注:
Todo:
1.其余mcu的flash底层驱动调用;
2.STM32芯片需要进一步细分;
*****************************************************************************/
/*FLASH解锁*/
static inline void flash_adapter_Unlock(void)
{
    __disable_irq();
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_FLASH_Unlock();
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    EFM_Unlock();
#endif
}
/*FLASH上锁*/
static inline void flash_adapter_Lock(void)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    HAL_FLASH_Lock();
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    EFM_Lock();
#endif
    __enable_irq();
}
/*块(扇区)擦除*/
static int8_t flash_adapter_Sector_Erase(uint32_t EraseAddr)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    FLASH_EraseInitTypeDef FlashEraseInit = {0};
    uint32_t  PageError, Page;

    Page = (EraseAddr - FLASH_START_ADDR) / FLASH_PAGE_SIZE;
    FlashEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;   //页擦除
    FlashEraseInit.Page = Page;                         //页号
    FlashEraseInit.NbPages = 1;                         //擦除页数
    if(HAL_FLASHEx_Erase(&FlashEraseInit, &PageError) != HAL_OK)
        return 0;   //擦写失败
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Ok != Flash_SectorErase(EraseAddr))
        return 0;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(Ok != EFM_SectorErase(EraseAddr))
        return 0;
#endif
    return 1;
}
/*全片擦除*/
static int8_t flash_adapter_Chip_Erase(void)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    FLASH_EraseInitTypeDef FlashEraseInit = {0};
    uint32_t  PageError;

    FlashEraseInit.TypeErase = FLASH_TYPEERASE_MASS;    //全片擦除
    if(HAL_FLASHEx_Erase(&FlashEraseInit, &PageError) != HAL_OK)
        return 0;   //擦写失败
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Ok != Flash_ChipErase())
        return 0;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    if(Ok != EFM_MassErase(0x00000000))
        return 0;
#endif
    return 1;
}
/*按字节写入*/
static int8_t flash_adapter_Write_Byte(uint32_t WriteAddr, uint8_t Data)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    return 0;       //G070只有按双字(8byte)写入
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Ok != Flash_WriteByte(WriteAddr, Data))
        return 0;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    return 0;       //F460只能按字(4byte)写入
#endif
}
/*按半字写入*/
static int8_t flash_adapter_Write_HalfWord(uint32_t WriteAddr, uint16_t Data)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    return 0;       //G070只有按双字(8byte)写入
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Ok != Flash_WriteHalfWord(WriteAddr, Data)
        return 0;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    return 0;       //F460只能按字(4byte)写入
#endif
}
/*按字写入*/
static int8_t flash_adapter_Write_Word(uint32_t WriteAddr, uint32_t Data)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    return 0;       //G070只有按双字(8byte)写入
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Ok != Flash_WriteWord(WriteAddr, Data)
        return 0;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    //en_result_t Ret = EFM_SequenceProgram(WriteAddr, 1, &Data);   //抄官方例程调用EFM_SequenceProgram接口,程序死机...
    if(Ok != EFM_SingleProgram(WriteAddr, Data))
        return 0;
#endif
    return 1;
}
/*按双字写入*/
static int8_t flash_adapter_Write_DWord(uint32_t WriteAddr, uint64_t Data)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
    if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, WriteAddr, Data) != HAL_OK)
        return 0;   //擦写失败
    return 1;
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
    if(Ok != Flash_WriteWord(WriteAddr, Data)
        return 0;
    return 1;
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    return 0;       //F460只能按字(4byte)写入
#endif
}
/********************************函数定义************************************
*函数名:

*函数功能描述: FLASH适配模块-FLASH底层驱动适配内部函数

*函数参数:

*函数返回值: 0-失败  1-成功

*备注:
*****************************************************************************/
/*判断地址1,2是否位于同一页*/
static BOOL flash_adapter_If_In_Same_Sector(uint32_t Addr1, uint32_t Addr2)
{
    uint32_t Sector1, Sector2;

    Sector1 = Addr1 / FLASH_PAGE_SIZE;
    Sector2 = Addr2 / FLASH_PAGE_SIZE;
    return (BOOL)(Sector1 == Sector2);
}
/********************************函数定义************************************
*函数名:

*函数功能描述: FLASH适配模块-FLASH初始化

*函数参数:

*函数返回值:

*备注:
*****************************************************************************/
int8_t flash_adapter_Init(void)
{
#if (MCU_TYPE == MCU_TYPE_STM32)
#elif (MCU_TYPE == MCU_TYPE_HC32_F0 || MCU_TYPE == MCU_TYPE_HC32_L1)
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    /* Unlock EFM. */
    EFM_Unlock();

    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
        ;
    }
    /* Lock EFM. */
    EFM_Lock();
#endif
    return 1;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: FLASH适配模块-FLASH格式化操作接口

*函数参数:

*函数返回值:

*备注:
*****************************************************************************/
/*flash擦除*/
/*FlashAtom: flash操作原子类型  PageAddr: 擦除地址(当前页任意地址) PageNum: 连续擦除页数*/
int8_t flash_adapter_Erase(uint8_t FlashAtom, uint32_t PageAddr, uint32_t PageNum)
{
    uint32_t EndAddr, PageStartAddr;    //擦除结束地址,当前擦除操作地址(页起始地址)
    uint16_t Index = 0;
    int8_t Ret = RES_FLASH_SUCCESS;

    /*参数合法性判断*/
    if(FlashAtom < PAGE || FlashAtom > CHIP)
        return RES_FLASH_ERR_CMD;
    if(!PageNum)
        return RES_FLASH_ERR_ADDR;
    PageStartAddr = (PageAddr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE; //计算页起始地址
    EndAddr = PageStartAddr + (PageNum * FLASH_PAGE_SIZE) - 1;
    if(EndAddr > FLASH_END_ADDR)
        return RES_FLASH_ERR_ADDR;

    /*FLASH擦除操作*/
    flash_adapter_Unlock();
    if(FlashAtom == CHIP)       /*全片擦除*/
    {
        if(!flash_adapter_Chip_Erase())
        {
            Ret = RES_FLASH_ERR_ERASE;
        }
    }else if(FlashAtom == PAGE) /*页擦除*/
    {
        while(Index < PageNum)
        {
            if(!flash_adapter_Sector_Erase(PageStartAddr))
            {
                Ret = RES_FLASH_ERR_ERASE;
                break;
            }
            PageStartAddr += FLASH_PAGE_SIZE;
            Index++;
        }
    }
    flash_adapter_Lock();
    return Ret;
}
/*flash编程*/
/*FlashAtom: flash操作原子类型  StartAddr: 写入起始地址 DataMemAtom: 待写入数据原子类型  *Data: 待写入数据  ItemNum: 数据个数(非字节数)*/
/*FlashAtom对应的数据类型必须和*Data指向数据类型及StartAddr的值对应,否则会进入HardFault()中断*/
int8_t flash_adapter_Write(uint8_t FlashAtom, uint32_t StartAddr, uint8_t DataMemAtom, const void *Data, uint32_t ItemNum)
{
    uint32_t EndAddr, WriteAddr;        //写结束flash地址,当前写操作flash地址
    uint8_t AddrJmpFlash, AddrJmpMemory;
    static  uint32_t s_WriteAddr = 0;   //用于判断是否跨页操作
    int8_t Res;
    int8_t Ret = RES_FLASH_SUCCESS;

    /*参数合法性判断*/
    if(FlashAtom < BYTE || FlashAtom > DOUBLE_WORD || DataMemAtom < BYTE || DataMemAtom > DOUBLE_WORD)
        return RES_FLASH_ERR_CMD;
    if(FlashAtom < DataMemAtom)     //eg: 输入数据是int32_t位,至少采用WORD(4字节)操作
        return RES_FLASH_ERR_CMD;
    if(!Data)
        return RES_FLASH_ERR_ADDR;
    EndAddr = StartAddr + (ItemNum << (FlashAtom - BYTE));
    if(EndAddr > FLASH_END_ADDR)
        return RES_FLASH_ERR_ADDR;

    /*FLASH写操作*/
    flash_adapter_Unlock();
    AddrJmpFlash = 1 << (FlashAtom - BYTE);
    AddrJmpMemory = 1 << (DataMemAtom - BYTE);
    WriteAddr = StartAddr;
    s_WriteAddr = WriteAddr;
    while(WriteAddr < EndAddr)
    {
        /*连续写操作跨页时需要先擦除*/
        if(flash_adapter_If_In_Same_Sector(s_WriteAddr, WriteAddr) == FALSE)
        {
            flash_adapter_Sector_Erase(WriteAddr);
            s_WriteAddr = WriteAddr;
        }
        if(FlashAtom == BYTE)               /*按字节写入*/
        {
            Res = flash_adapter_Write_Byte(WriteAddr, *(uint8_t*)Data);
        }else if(FlashAtom == HALF_WORD)    /*按半字写入*/
        {
            if(DataMemAtom == BYTE)
                Res = flash_adapter_Write_HalfWord(WriteAddr, *(uint8_t*)Data);
            else    //(DataMemAtom == HALF_WORD)
                Res = flash_adapter_Write_HalfWord(WriteAddr, *(uint16_t*)Data);
        }else if(FlashAtom == WORD)         /*按字写入*/
        {
            if(DataMemAtom == BYTE)
                Res = flash_adapter_Write_Word(WriteAddr, *(uint8_t*)Data);
            else if(DataMemAtom == HALF_WORD)
                Res = flash_adapter_Write_Word(WriteAddr, *(uint16_t*)Data);
            else    //(DataMemAtom == WORD)
                Res = flash_adapter_Write_Word(WriteAddr, *(uint32_t*)Data);
        }else /*FlashAtom == DOUBLE_WORD*/  /*按双字写入*/
        {
            if(DataMemAtom == BYTE)
                Res = flash_adapter_Write_DWord(WriteAddr, *(uint8_t*)Data);
            else if(DataMemAtom == HALF_WORD)
                Res = flash_adapter_Write_DWord(WriteAddr, *(uint16_t*)Data);
            else if(DataMemAtom == WORD)
                Res = flash_adapter_Write_DWord(WriteAddr, *(uint32_t*)Data);
            else    //(DataMemAtom == DOUBLE_WORD)
                Res = flash_adapter_Write_DWord(WriteAddr, *(uint64_t*)Data);
        }
        if(!Res)
        {
            Ret = RES_FLASH_ERR_WRITE;
            break;
        }
        WriteAddr += AddrJmpFlash;
        //pData是void*类型,pData++只加1.须手动+N(N为uintXX_t类型字节数),防止*(uintXX_t*)pData导致地址对齐出错进入HardFault
        Data += AddrJmpMemory;
    }
    flash_adapter_Lock();
    return Ret;
}
/*flash读取*/
/*FlashAtom: flash操作原子类型  StartAddr: 读取起始地址  DataMemAtom: 读取数据缓存原子类型  *Data：存储读出数据的起始地址  ItemNum：读取的元素数(非字节数)*/
/*FlashAtom对应的数据类型必须和*Data指向数据类型及StartAddr的值对应,否则会进入HardFault()中断*/
int8_t flash_adapter_Read(uint8_t FlashAtom, uint32_t StartAddr, uint8_t DataMemAtom, void *Data, uint32_t ItemNum)
{
    uint32_t EndAddr, ReadAddr; //读结束flash地址,当前读操作flash地址
    uint8_t AddrJmpFlash, AddrJmpMemory;

    /*参数合法性判断*/
    if(FlashAtom < BYTE || FlashAtom > DOUBLE_WORD || DataMemAtom < BYTE || DataMemAtom > DOUBLE_WORD)
        return RES_FLASH_ERR_CMD;
    if(FlashAtom < DataMemAtom)     //eg: 数据读出到int32_t缓存区,至少采用WORD(4字节)操作
        return RES_FLASH_ERR_CMD;
    if(!Data)
        return RES_FLASH_ERR_ADDR;
    EndAddr = StartAddr + (ItemNum << (FlashAtom - BYTE));
    if(EndAddr > FLASH_END_ADDR)
        return RES_FLASH_ERR_ADDR;

    /*FLASH读操作*/
    AddrJmpFlash = 1 << (FlashAtom - BYTE);
    AddrJmpMemory = 1 << (DataMemAtom - BYTE);
    ReadAddr = StartAddr;
    while(ReadAddr < EndAddr)
    {
        if(FlashAtom == BYTE)               /*按字节读*/
        {
            *(uint8_t*)Data = *(uint8_t*)ReadAddr;
        }else if(FlashAtom == HALF_WORD)    /*按半字读*/
        {
            if(DataMemAtom == BYTE)
                *(uint8_t*)Data = *(uint16_t*)ReadAddr;
            else    //(DataMemAtom == HALF_WORD)
                *(uint16_t*)Data = *(uint16_t*)ReadAddr;
        }else if(FlashAtom == WORD)         /*按字读*/
        {
            if(DataMemAtom == BYTE)
                *(uint8_t*)Data = *(uint32_t*)ReadAddr;
            else if(DataMemAtom == HALF_WORD)
                *(uint16_t*)Data = *(uint32_t*)ReadAddr;
            else    //(DataMemAtom == WORD)
                *(uint32_t*)Data = *(uint32_t*)ReadAddr;
        }else /*FlashAtom == DOUBLE_WORD*/  /*按双字读*/
        {
            if(DataMemAtom == BYTE)
                *(uint8_t*)Data = *(uint64_t*)ReadAddr;
            else if(DataMemAtom == HALF_WORD)
                *(uint16_t*)Data = *(uint64_t*)ReadAddr;
            else if(DataMemAtom == WORD)
                *(uint32_t*)Data = *(uint64_t*)ReadAddr;
            else    //(DataMemAtom == DOUBLE_WORD)
                *(uint64_t*)Data = *(uint64_t*)ReadAddr;
        }
        ReadAddr += AddrJmpFlash;
        //pData是void*类型,pData++只加1.须手动+N(N为uintXX_t类型字节数),防止*(uintXX_t*)pData导致地址对齐出错进入HardFault
        Data += AddrJmpMemory;
    }
    return RES_FLASH_SUCCESS;
}
/********************************函数定义************************************
*函数名:

*函数功能描述: FLASH适配模块-模块功能测试

*函数参数: 无

*函数返回值: 无

*备注:
*****************************************************************************/
void flash_adapter_Test(void)
{
#if (USE_FLASH_TEST == 1)
    uint32_t ItemNum = 0;
    int8_t Result;
    static uint32_t g_TestWordWrite[400];
    static uint32_t g_TestWordRead[400];
    static uint16_t g_TestHalfWordWrite[400];
    static uint16_t g_TestHalfWordRead[400];
    static uint8_t g_TestByteWrite[400];
    static uint8_t g_TestByteRead[400];

    /*测试数据初始化*/
    if(g_TestFlashOpCmd == 100)
    {
        g_TestFlashOpCmd = 0;
        memset(g_TestWordWrite, 0, sizeof g_TestWordWrite);
        memset(g_TestWordRead, 0, sizeof g_TestWordRead);
        memset(g_TestHalfWordWrite, 0, sizeof g_TestHalfWordWrite);
        memset(g_TestHalfWordRead, 0, sizeof g_TestHalfWordRead);
        memset(g_TestByteWrite, 0, sizeof g_TestByteWrite);
        memset(g_TestByteRead, 0, sizeof g_TestByteRead);
        for(uint32_t Index = 0; Index < sizeof g_TestWordWrite / sizeof g_TestWordWrite[0]; Index++)
        {
            g_TestWordWrite[Index] = Index;
        }
        for(uint32_t Index = 0; Index < sizeof g_TestHalfWordWrite / sizeof g_TestHalfWordWrite[0]; Index++)
        {
            g_TestHalfWordWrite[Index] = Index;
        }
        for(uint32_t Index = 0; Index < sizeof g_TestByteWrite / sizeof g_TestByteWrite[0]; Index++)
        {
            g_TestByteWrite[Index] = Index;
        }
    }
    /*以下测试指令用于测试FLASH操作-FLASH擦除*/
    if(g_TestFlashOpCmd == 1)
    {
        Result = flash_adapter_Erase(PAGE, TEST_ADDR, 1);
        g_TestFlashOpCmd = 0;
    }
    if(g_TestFlashOpCmd == 2)
    {
        Result = flash_adapter_Erase(PAGE, TEST_ADDR, 2);
        g_TestFlashOpCmd = 0;
    }
#if (MCU_TYPE == MCU_TYPE_STM32)
    /*以下测试指令用于测试FLASH操作-FLASH按Byte读写*/
    if(g_TestFlashOpCmd == 3)
    {
        ItemNum = sizeof g_TestByteWrite / sizeof g_TestByteWrite[0];
        Result = flash_adapter_Write(DOUBLE_WORD, TEST_ADDR, BYTE, g_TestByteWrite, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    if(g_TestFlashOpCmd == 4)
    {
        ItemNum = sizeof g_TestByteRead / sizeof g_TestByteRead[0];
        memset(g_TestByteRead, 0, sizeof g_TestByteRead);
        Result = flash_adapter_Read(DOUBLE_WORD, TEST_ADDR, BYTE, g_TestByteRead, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    /*以下测试指令用于测试FLASH操作-FLASH按HalfWord读写*/
    if(g_TestFlashOpCmd == 5)
    {
        ItemNum = sizeof g_TestHalfWordWrite / sizeof g_TestHalfWordWrite[0];
        Result = flash_adapter_Write(DOUBLE_WORD, TEST_ADDR, HALF_WORD, g_TestHalfWordWrite, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    if(g_TestFlashOpCmd == 6)
    {
        ItemNum = sizeof g_TestHalfWordRead / sizeof g_TestHalfWordRead[0];
        memset(g_TestHalfWordRead, 0, sizeof g_TestHalfWordRead);
        Result = flash_adapter_Read(DOUBLE_WORD, TEST_ADDR, HALF_WORD, g_TestHalfWordRead, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    /*以下测试指令用于测试FLASH操作-FLASH按Word读写*/
    if(g_TestFlashOpCmd == 7)
    {
        ItemNum = sizeof g_TestWordWrite / sizeof g_TestWordWrite[0];
        Result = flash_adapter_Write(DOUBLE_WORD, TEST_ADDR, WORD, g_TestWordWrite, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    if(g_TestFlashOpCmd == 8)
    {
        ItemNum = sizeof g_TestWordRead / sizeof g_TestWordRead[0];
        memset(g_TestWordRead, 0, sizeof g_TestWordRead);
        Result = flash_adapter_Read(DOUBLE_WORD, TEST_ADDR, WORD, g_TestWordRead, ItemNum);
        g_TestFlashOpCmd = 0;
    }
#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
    /*以下测试指令用于测试FLASH操作-FLASH按Byte读写*/
    if(g_TestFlashOpCmd == 3)
    {
        ItemNum = sizeof g_TestByteWrite / sizeof g_TestByteWrite[0];
        Result = flash_adapter_Write(WORD, TEST_ADDR, BYTE, g_TestByteWrite, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    if(g_TestFlashOpCmd == 4)
    {
        ItemNum = sizeof g_TestByteRead / sizeof g_TestByteRead[0];
        memset(g_TestByteRead, 0, sizeof g_TestByteRead);
        Result = flash_adapter_Read(WORD, TEST_ADDR, BYTE, g_TestByteRead, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    /*以下测试指令用于测试FLASH操作-FLASH按HalfWord读写*/
    if(g_TestFlashOpCmd == 5)
    {
        ItemNum = sizeof g_TestHalfWordWrite / sizeof g_TestHalfWordWrite[0];
        Result = flash_adapter_Write(WORD, TEST_ADDR, HALF_WORD, g_TestHalfWordWrite, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    if(g_TestFlashOpCmd == 6)
    {
        ItemNum = sizeof g_TestHalfWordRead / sizeof g_TestHalfWordRead[0];
        memset(g_TestHalfWordRead, 0, sizeof g_TestHalfWordRead);
        Result = flash_adapter_Read(WORD, TEST_ADDR, HALF_WORD, g_TestHalfWordRead, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    /*以下测试指令用于测试FLASH操作-FLASH按Word读写*/
    if(g_TestFlashOpCmd == 7)
    {
        ItemNum = sizeof g_TestWordWrite / sizeof g_TestWordWrite[0];
        Result = flash_adapter_Write(WORD, TEST_ADDR, WORD, g_TestWordWrite, ItemNum);
        g_TestFlashOpCmd = 0;
    }
    if(g_TestFlashOpCmd == 8)
    {
        ItemNum = sizeof g_TestWordRead / sizeof g_TestWordRead[0];
        memset(g_TestWordRead, 0, sizeof g_TestWordRead);
        Result = flash_adapter_Read(WORD, TEST_ADDR, WORD, g_TestWordRead, ItemNum);
        g_TestFlashOpCmd = 0;
    }
#endif
#endif
}
