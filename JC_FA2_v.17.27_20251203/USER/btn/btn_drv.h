/********************************文件说明*************************************
*文件名: btn_drv.h

*作者: Yuchen Tan

*版本: V1.1.2

*功能简介:

*备注: 无

*修改履历:

*****************************************************************************/
#ifndef BTN_DRV_IO_GEN_H_
#define BTN_DRV_IO_GEN_H_

/*****************************文件包含(公开)**********************************
*
*备注: 可在此包含底层的库文件
*
*****************************************************************************/
#include "common.h"
#include "gpio_adapter.h"
/******************************宏定义(公开)***********************************
*
*备注: 需要被外部使用或可任意修改的宏在这里定义
*
*****************************************************************************/
/*按钮驱动器类型(根据按钮信号产生方式划分)*/
#define TYPE_IO_GENERAL         (0)     //数字IO键盘
#define TYPE_IO_MATRIX          (1)     //矩阵IO键盘
#define INVALID_TYPE            (2)     //注: 必须==类型总数

/*按钮驱动器索引(根据需要按顺序增加,必须从0开始!)*/
#define BTN_DRV_1               (0)
#define BTN_DRV_2               (1)
#define BTN_DRV_NB              (1)     //已使用的按钮驱动器个数
#if (BTN_DRV_NB > 2)	//和BTN_DRV_MASK_t类型的位数对应
#error "btn_driver number is limited within 2! and it won`t exceed 4 before I change the code to created btn_driver instance dynamically instead of static!"
#endif

/*按钮驱动器参数定义*/
#define BTN_IO_LI_NB_MAX        (16)    /*输入线数最大值(需要和BTN_VALUE_t对应)*/
#define BTN_IO_DEBOUNCE         (5)     /*IO类型按钮消抖时间(ms)*/

/*按钮驱动接口返回值定义*/
#define RES_ERR_INIT_PARAM      (-1)
#define RES_ERR_NEW_OBJ         (-2)
#define RES_ERR_PARAM           (-3)
#define RES_BUSY                (0)
#define RES_SUCCESS             (1)

/*按钮端口操作定义*/
#define BTN_SET_LO(PORT, PIN)    gpio_adapter_Set_Pin((GPIO_PORT_t)PORT, (GPIO_PIN_t)PIN)   /*输出端口置位*/
#define BTN_CLR_LO(PORT, PIN)    gpio_adapter_Reset_Pin((GPIO_PORT_t)PORT, (GPIO_PIN_t)PIN) /*输出端口清零*/
#define BTN_READ_LI(PORT, PIN)   gpio_adapter_Read_Pin((GPIO_PORT_t)PORT, (GPIO_PIN_t)PIN)  /*读取输入端口*/

/*IO按钮键值输出顺序*/
#define IO_BTNVALUE_OUTPUT_SEQ  (0)    //0:按端口初始化顺序输出  1:按端口初始化逆序输出
/**************************数据类型及结构定义(公开)***************************
*
*备注: 需要被外部使用的数据结构及类型在这里定义
*
*****************************************************************************/
/*按钮键值类型定义*/
typedef uint16_t    BTN_VALUE_t;      //(16线输入)需要和BTN_IO_LI_NB_MAX对应

/*按钮选择掩码类型定义*/
typedef uint8_t    	BTN_DRV_MASK_t;		//uint8_t有8位,最多支持8个按钮驱动器

/*GPIO信号的按钮端口类型定义*/
typedef struct
{
    GPIO_PORT_t     BtnPort;
    GPIO_PIN_t      BtnPin;
}BTN_IO_t;

/*常规IO按钮驱动器*/
typedef struct
{
    /*输入输出引脚定义*/
    uint8_t         IOInputNb;          /*输入信号线数*/
    BTN_IO_t*       IOInput;            /*输入信号GPIO端口指针*/
    /*引脚状态读取*/
    uint8_t         DebounceCnt;        /*消抖计数*/
    BTN_VALUE_t     IOInputValue;       /*当前端口值*/
    BTN_VALUE_t     IOInputValuePrev;   /*上次端口值*/
}BTN_IO_GEN_t;

/*矩阵IO按钮驱动器*/
typedef struct
{
    /*输入输出引脚定义*/
    uint8_t         IOInputNb;          /*输入信号线数*/
    uint8_t         IOOutputNb;         /*输出信号线数*/
    BTN_IO_t*       IOInput;            /*输入信号GPIO端口指针*/
    BTN_IO_t*       IOOutput;           /*输出信号GPIO端口指针*/
    /*引脚状态读取*/
    uint8_t         DebounceCnt;        //消抖计数
    BTN_VALUE_t     IOInputValue;
    BTN_VALUE_t     IOInputValuePrev;
}BTN_IO_MT_t;

/*按钮设备驱动器类型定义*/
typedef union
{
    //(不能放到union里面)uint8_t         Type;               /*按钮驱动器类型*/
    BTN_IO_GEN_t*   BtnDriverGen;       //常规IO按钮驱动器句柄
    BTN_IO_MT_t*    BtnDriverMT;        //矩阵IO按钮驱动器句柄
}BTN_DRIVER_t;
/*****************************函数声明(公开)**********************************
*
*备注: 需要被外部使用的接口函数在这里声明
*
*****************************************************************************/
/*常规IO按钮驱动器*/
int8_t btn_drv_IOGen_Create(uint8_t Index, BTN_IO_t *IOInput, uint8_t IOInputNb);
int8_t btn_drv_IOGen_Delete(uint8_t Index);
BTN_IO_GEN_t* btn_drv_IOGen_Get_Handle(uint8_t Index);
int8_t btn_drv_IOGen_Set_IO(BTN_IO_GEN_t* pGen, BTN_IO_t *IOInput, uint8_t IOInputNb);
int8_t btn_drv_IOGen_Update_Value(uint8_t Index);
BTN_VALUE_t btn_drv_IOGen_Get_Value(BTN_DRIVER_t Driver);
/*矩阵IO按钮驱动器*/
int8_t btn_drv_IOMat_Create(uint8_t Index, BTN_IO_t *IOInputOutput, uint8_t IOInputNb, uint8_t IOOutputNb);
int8_t btn_drv_IOMat_Delete(uint8_t Index);
int8_t btn_drv_IOMat_Set_IO(BTN_IO_MT_t* pMat, BTN_IO_t *IOInputOutput, uint8_t IOInputNb, uint8_t IOOutputNb);
BTN_IO_MT_t* btn_drv_IOMat_Get_Handle(uint8_t Index);
int8_t btn_drv_IOMat_Update_Value(uint8_t Index);
BTN_VALUE_t btn_drv_IOMat_Get_Value(BTN_DRIVER_t Driver);
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
