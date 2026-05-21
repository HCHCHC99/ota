/********************************文件说明*************************************
*文件名: flash_adapter.h

*作者: Yuchen Tan

*版本: V1.0.2

*功能简介:

*备注: 无

*修改履历:
*****************************************************************************/
#ifndef FLASH_ADAPTER_H_
#define FLASH_ADAPTER_H_
/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*与MCU平台相关的抽象类型定义*/
#if (MCU_TYPE == MCU_TYPE_STM32)
#define FLASH_START_ADDR    (0x08000000u)
#define FLASH_END_ADDR      (0x0801FFFFu)   //必须根据实际的stm32型号填写
#if 0   /*HAL库有FLASH_PAGE_SIZE和FLASH_PAGE_NB的定义*/
#define FLASH_PAGE_SIZE     (0x800)         //2KB
#define FLASH_PAGE_NB       (64)
#endif

#elif (MCU_TYPE == MCU_TYPE_HC32_F0)
#define FLASH_START_ADDR    (0x00000000u)
#define FLASH_END_ADDR      (0x0000FFFFu)
#define FLASH_PAGE_SIZE     (0x200)         //512
#define FLASH_PAGE_NB       (128)

#elif (MCU_TYPE == MCU_TYPE_HC32_L1)

#elif (MCU_TYPE == MCU_TYPE_HC32_F4)
#define FLASH_START_ADDR    (0x00000000u)
#define FLASH_END_ADDR      (0x0007FFFFu)
#define FLASH_PAGE_SIZE     (0x2000)        //8k
#define FLASH_PAGE_NB       (64)

#endif

/*用户可使用的flash扇区起始地址定义(倒序)*/
#define BLOCK_L1_ADDR       ((FLASH_END_ADDR + 1) - 1 * FLASH_PAGE_SIZE)    //倒数第1页
#define BLOCK_L2_ADDR       ((FLASH_END_ADDR + 1) - 2 * FLASH_PAGE_SIZE)    //倒数第2页
#define BLOCK_L3_ADDR       ((FLASH_END_ADDR + 1) - 3 * FLASH_PAGE_SIZE)    //倒数第3页
#define BLOCK_L4_ADDR       ((FLASH_END_ADDR + 1) - 4 * FLASH_PAGE_SIZE)    //倒数第4页
#define BLOCK_L5_ADDR       ((FLASH_END_ADDR + 1) - 5 * FLASH_PAGE_SIZE)    //倒数第5页
#define BLOCK_L6_ADDR       ((FLASH_END_ADDR + 1) - 6 * FLASH_PAGE_SIZE)    //倒数第6页
#define BLOCK_L7_ADDR       ((FLASH_END_ADDR + 1) - 7 * FLASH_PAGE_SIZE)    //倒数第6页

/*flash操作接口参数定义(数据原子类型)*/
#define PAGE                (1)
#define CHIP                (2)
#define BYTE                (3)
#define HALF_WORD           (4)
#define WORD                (5)
#define DOUBLE_WORD         (6)

/*flash操作接口返回值定义(Todo: 可继续细分)*/
#define RES_FLASH_ERR_PARAM     (-1)
#define RES_FLASH_ERR_CMD       (-2)
#define RES_FLASH_ERR_ADDR      (-3)
#define RES_FLASH_ERR_ERASE     (-4)
#define RES_FLASH_ERR_WRITE     (-5)
#define RES_FLASH_ERR_READ      (-6)
#define RES_FLASH_SUCCESS       (1)
#define RES_FLASH_BUSY          (0)
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/

/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*FLASH适配模块-FLASH初始化*/
int8_t flash_adapter_Init(void);
/*FLASH适配模块-FLASH擦除操作*/
int8_t flash_adapter_Erase(uint8_t FlashAtom, uint32_t PageAddr, uint32_t PageNum);
/*FLASH适配模块-FLASH写入操作*/
int8_t flash_adapter_Write(uint8_t FlashAtom, uint32_t StartAddr, uint8_t DataMemAtom, const void *Data, uint32_t ItemNum);
/*FLASH适配模块-FLASH读取操作*/
int8_t flash_adapter_Read(uint8_t FlashAtom, uint32_t StartAddr, uint8_t DataMemAtom, void *Data, uint32_t ItemNum);

void flash_adapter_Test(void);
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
