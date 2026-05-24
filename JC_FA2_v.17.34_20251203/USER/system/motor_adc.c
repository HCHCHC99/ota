#include "motor_adc.h"
#include "ring_buffer.h"
#include "main.h"
#include "usart_usb.h"
#include "hz_timer.h"
/******************************应用层变量定义*******************************/
// 电机采样句柄实例（原adc_adapter.c中的全局变量移到应用层）
ADC_CH_CTRL_t    hADCChannalMBV; // 母线电压采样
ADC_CH_CTRL_t    hADCChannalIVM; // 电流采样

// 采样缓冲区实例
uint16_t sample_buffer_vm[SAMPLE_BUFFER_SIZE_vm] = {0};
uint16_t sample_buffer_cur[SAMPLE_BUFFER_SIZE_cur] = {0};

// 电机测量状态变量（应用层内部维护）
static float s_motor_current = 0.0f;
static float s_motor_voltage = 0.0f;
static bool s_motor_measure_valid = false;  // 测量有效性标记
static bool s_motor_current_over = false;   // 过流标记
uint16_t    time_sample_current_over = 0;   // 过流计数
static NonBlockingDelay_t Send_Usart_Delay;
static int Send_label = 0;

extern RingBuffer current_buffer;
extern SystemContext g_sys_ctx;

/******************************应用层接口实现*******************************/
/**
 * @brief 电机ADC初始化（包含电流、电压通道）
 */
void ADC_Init_hz(void)
{
    ADC_CUR_Init();  // 初始化电流采样通道
    ADC_VM_Init();   // 初始化电压采样通道
}

/**
 * @brief 获取电机测量值
 * @param current 输出电流值
 * @param voltage 输出电压值
 * @param valid 测量有效性标志
 * @param current_over_valid 过流标志
 */
void MotorProtect_GetMeasure(float *current, float *voltage, bool *valid, bool *current_over_valid)
{
    if (current != NULL)
        *current = s_motor_current;
    if (voltage != NULL)
        *voltage = s_motor_voltage;
    if (valid != NULL)
        *valid = s_motor_measure_valid;
    if (current_over_valid != NULL)
        *current_over_valid = s_motor_current_over;    
}

/**
 * @brief 电机保护功能处理
 * @param motor_cmd 电机命令状态
 * @return 系统错误码
 */
SystemError MotorProtect_Generate(MotorCmdState motor_cmd)
{
    SystemError err = ERROR_NONE;
    s_motor_measure_valid = false;  // 重置测量有效性
    ADC_Compare_Result adc_cur_result = ADC_COMPARE_INITIALIZING;

    switch (motor_cmd)
    {
        case MOTOR_STOP:
            // 采集并比较电机电流
            // adc_cur_result = ADC_CUR_ARRAY_COMPARE(&s_motor_current);
            adc_cur_result = ADC_CUR_ARRAY_COMPARE_OPAMP(&s_motor_current);
            // ADC_VM(&s_motor_voltage);
            // 电流过流判断
            if (adc_cur_result == ADC_COMPARE_ALL_ABOVE)
            {
                time_sample_current_over++;
                if(time_sample_current_over >= TIME_SAMPLE_CURRENT_OVER)
                {
                    s_motor_current_over = true;
                    err = ERROR_OVER_CURRENT;              
                }
            }
            // 未过流
            else if (adc_cur_result == ADC_COMPARE_NOT_ALL_ABOVE)
            {
                s_motor_current_over = false;
                time_sample_current_over = 0;
            }
            // 无效状态不处理
            else if (adc_cur_result == ADC_COMPARE_NOT_ENOUGH_SAMPLES  || 
                     adc_cur_result == ADC_COMPARE_NOT_READY           || 
                     adc_cur_result == ADC_COMPARE_DISABLED            || 
                     adc_cur_result == ADC_COMPARE_INITIALIZING)
            {
            }

            // 无错误时标记测量有效
            if (err == ERROR_NONE && adc_cur_result == ADC_COMPARE_NOT_ALL_ABOVE)
            {
                s_motor_measure_valid = true;
            }
            break;

        case MOTOR_RUN_EXTEND:
        case MOTOR_RUN_RETRACT:
            // 采集并比较电机电流
            adc_cur_result = ADC_CUR_ARRAY_COMPARE_OPAMP(&s_motor_current);
            // 采集电机电压
            // ADC_VM(&s_motor_voltage);

            // 电流过流判断
            if (adc_cur_result == ADC_COMPARE_ALL_ABOVE)
            {
                time_sample_current_over++;
                if(time_sample_current_over >= TIME_SAMPLE_CURRENT_OVER)
                {
                    s_motor_current_over = true;
                    err = ERROR_OVER_CURRENT;              
                }
            }
            // 未过流
            else if (adc_cur_result == ADC_COMPARE_NOT_ALL_ABOVE)
            {
                s_motor_current_over = false;
                time_sample_current_over = 0;
            }
            // 无效状态不处理
            else if (adc_cur_result == ADC_COMPARE_NOT_ENOUGH_SAMPLES  || 
                     adc_cur_result == ADC_COMPARE_NOT_READY           || 
                     adc_cur_result == ADC_COMPARE_DISABLED            || 
                     adc_cur_result == ADC_COMPARE_INITIALIZING)
            {
            }

            // // 过压判断
            // if (err == ERROR_NONE && s_motor_voltage > THRESHOLD_VOLTAGE_OVER)
            // {
            //     s_motor_measure_valid = false;
            //     err = ERROR_OVER_VOLTAGE;
            // }

            // // 欠压判断
            // if (err == ERROR_NONE && s_motor_voltage < THRESHOLD_VOLTAGE_UNDER)
            // {
            //     s_motor_measure_valid = false;
            //     err = ERROR_UNDER_VOLTAGE;
            // }
            

            // 无错误时标记测量有效
            if (err == ERROR_NONE && adc_cur_result == ADC_COMPARE_NOT_ALL_ABOVE)
            {
                s_motor_measure_valid = true;
            }
            break;

        default:
            // 未知指令重置状态
            s_motor_current = 0.0f;
            s_motor_voltage = 0.0f;
            s_motor_measure_valid = false;
            s_motor_current_over = false;
            break;
    }

    return err;
}

/**
 * @brief 电流采样通道初始化
 */
void ADC_CUR_Init(void)
{
    stc_adc_init_t stcAdcInit;
    MEM_ZERO_STRUCT(stcAdcInit);

    // ADC硬件初始化（HC32F4平台）
    CLK_SetPeriClkSource(ClkPeriSrcPclk);
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

    stcAdcInit.enResolution = AdcResolution_12Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = AdcMode_SAOnce;
    ADC_Init(M4_ADC1, &stcAdcInit);

    // 注册电流通道到适配器层
    ADCH_t  VM_ADIVM = {ADCH_IVM_ADC, ADCH_IVM_ADCH, ADCH_IVM_PORT, ADCH_IVM_PIN};
    adc_adapter_hInit(&hADCChannalIVM, &VM_ADIVM);
    adc_adapter_Set_Channal_SmpTime(ADCH_IVM_ADCH, SampleTime_cur);
    adc_adapter_Set_Channal_SmpIntervalTime(ADCH_IVM_ADCH, SampleIntervalTime_cur);
    adc_adapter_Channel_Enable(ADCH_IVM_ADCH);



    //
    nbDelay_Init(&Send_Usart_Delay,1);
}

/**
 * @brief 电流采样数组比较（判断是否过流）
 * @param CUR_Voltage_ptr 输出电流值
 * @return 比较结果（枚举）
 */
ADC_Compare_Result ADC_CUR_ARRAY_COMPARE(float* CUR_Voltage_ptr)
{
    static bool firstCall = true;
    static float converted_voltages_cur[SAMPLE_BUFFER_SIZE_cur] = {0};
    static float converted_voltages_cur_convert[SAMPLE_BUFFER_SIZE_cur] = {0}; 
    static uint32_t converted_count = 0;
    
    // 首次调用初始化
    if (firstCall) {
        firstCall = false;
        adc_adapter_Reset_Sampling_Stats(ADCH_IVM_ADCH);
        return ADC_COMPARE_INITIALIZING;
    }
    
    // 执行电流采样
    int ADC_result = hz_adc_adapter_SCM_1Ch_Convert_CUR(ADCH_IVM_ADCH);
    
    if (1 == ADC_result) 
    {
        uint16_t buffer_cur[SAMPLE_BUFFER_SIZE_cur];
        uint32_t count;
        bool all_above_threshold = true;

        // 获取采样缓冲区数据
        adc_adapter_Get_Sample_Buffer(ADCH_IVM_ADCH, buffer_cur, &count, SAMPLE_BUFFER_SIZE_cur);
        
        // 转换采样数据并判断是否过流
        converted_count = 0;
        for (uint32_t i = 0; i < count; i++) {
            float AD_CUR_Voltage_sample = (float)buffer_cur[i] * ADC_REF_VOLTAGE / ADC_FULL_SCALE * 1000;
            // float CUR_Current_sample = (AD_CUR_Voltage_sample - 1650.0f) / 66 + hz_IVM_OFFSET;
            float CUR_Current_sample = (AD_CUR_Voltage_sample - 1650.0f) / 26.4 + hz_IVM_OFFSET;

            converted_voltages_cur[converted_count] = AD_CUR_Voltage_sample;
            converted_voltages_cur_convert[converted_count] = CUR_Current_sample;
            converted_count++;
            ring_buffer_put(&current_buffer, CUR_Current_sample, RB_PUT_OVERWRITE);

            // 判断是否超过电流阈值
            if ((CUR_Current_sample <= g_sys_ctx.Current_Limit_Upper) && 
                (CUR_Current_sample >= (-1)*(g_sys_ctx.Current_Limit_Upper))) {
                all_above_threshold = false; 
            }
        }
        
        // 计算平均电流值
        uint16_t ADData_Current = adc_adapter_Get_Sampling_Average(ADCH_IVM_ADCH);
        float ADCUR_Voltage = (float)ADData_Current * ADC_REF_VOLTAGE / ADC_FULL_SCALE * 1000;
        // *CUR_Voltage_ptr = (ADCUR_Voltage - 1650.0f) / 66 + hz_IVM_OFFSET;
        *CUR_Voltage_ptr = (ADCUR_Voltage - 1650.0f) / 26.4 + hz_IVM_OFFSET;        

        // 重置采样状态
        adc_adapter_Reset_Sampling_Stats(ADCH_IVM_ADCH);
        
        return all_above_threshold ? ADC_COMPARE_ALL_ABOVE : ADC_COMPARE_NOT_ALL_ABOVE;
    }
    else if (0 == ADC_result)
    {
        return ADC_COMPARE_NOT_ENOUGH_SAMPLES;
    }
    else if (-1 == ADC_result)
    {
        return ADC_COMPARE_NOT_READY;
    }
    else if (-2 == ADC_result)
    {
        return ADC_COMPARE_DISABLED;
    }
    
    return ADC_COMPARE_DISABLED;
}
/**
 * @brief 电流采样数组比较（判断是否过流）
 * @param CUR_Voltage_ptr 输出电流值
 * @return 比较结果（枚举）
 */
ADC_Compare_Result ADC_CUR_ARRAY_COMPARE_OPAMP(float* CUR_Voltage_ptr)
{
    static bool firstCall = true;
    static float converted_voltages_cur[SAMPLE_BUFFER_SIZE_cur] = {0};
    static float converted_voltages_cur_convert[SAMPLE_BUFFER_SIZE_cur] = {0}; 
    static uint32_t converted_count = 0;
    
    // 首次调用初始化
    if (firstCall) {
        firstCall = false;
        adc_adapter_Reset_Sampling_Stats(ADCH_IVM_ADCH);
        return ADC_COMPARE_INITIALIZING;
    }
    
    // 执行电流采样
    int ADC_result = hz_adc_adapter_SCM_1Ch_Convert_CUR(ADCH_IVM_ADCH);
    
    if (1 == ADC_result) 
    {
        uint16_t buffer_cur[SAMPLE_BUFFER_SIZE_cur];
        uint32_t count;
        bool all_above_threshold = true;

        // 获取采样缓冲区数据
        adc_adapter_Get_Sample_Buffer(ADCH_IVM_ADCH, buffer_cur, &count, SAMPLE_BUFFER_SIZE_cur);
        
        // 转换采样数据并判断是否过流
        converted_count = 0;
        for (uint32_t i = 0; i < count; i++) {
            // float AD_CUR_Voltage_sample = (float)buffer_cur[i] * ADC_REF_VOLTAGE / ADC_FULL_SCALE * 1000;

            float AD_CUR_Voltage_sample = (float)buffer_cur[i]/100*0.8;//*9.52381/1000;
            // float CUR_Current_sample = (AD_CUR_Voltage_sample - 1650.0f) / 66 + hz_IVM_OFFSET;

            // float CUR_Current_sample = (AD_CUR_Voltage_sample - 1650.0f) / 26.4 + hz_IVM_OFFSET;
             float CUR_Current_sample = AD_CUR_Voltage_sample;

            converted_voltages_cur[converted_count] = AD_CUR_Voltage_sample;
            converted_voltages_cur_convert[converted_count] = CUR_Current_sample;
            converted_count++;
            ring_buffer_put(&current_buffer, CUR_Current_sample, RB_PUT_OVERWRITE);

            // 判断是否超过电流阈值
            if ((CUR_Current_sample <= g_sys_ctx.Current_Limit_Upper) && 
                (CUR_Current_sample >= (-1)*(g_sys_ctx.Current_Limit_Upper))) {
                all_above_threshold = false; 
            }

            // USART
            // if(Send_label ==0 )
            // {
            //     nbDelay_Start(&Send_Usart_Delay);
            //     Send_label = 1;
            // }
            
            // if(nbDelay_IsComplete(&Send_Usart_Delay))
            // {
            //     USART1_Send_Data_IT((uint8_t*)"cur:", 5);
            //     USART1_Send_Float_IT(AD_CUR_Voltage_sample, 5);
            //     USART1_Send_Data_IT("\n",2);
            //     Send_label = 0;
            // }

        }
        
        // 计算平均电流值
        uint16_t ADData_Current = adc_adapter_Get_Sampling_Average(ADCH_IVM_ADCH);
        // float ADCUR_Voltage = (float)ADData_Current * ADC_REF_VOLTAGE / ADC_FULL_SCALE * 1000;
        float ADCUR_Voltage = (float)ADData_Current/100*0.8;// *9.52381/1000;
        // *CUR_Voltage_ptr = (ADCUR_Voltage - 1650.0f) / 66 + hz_IVM_OFFSET;
        // *CUR_Voltage_ptr = (ADCUR_Voltage - 1650.0f) / 26.4 + hz_IVM_OFFSET;        
        *CUR_Voltage_ptr =ADCUR_Voltage;
        // 重置采样状态
        adc_adapter_Reset_Sampling_Stats(ADCH_IVM_ADCH);
        
        return all_above_threshold ? ADC_COMPARE_ALL_ABOVE : ADC_COMPARE_NOT_ALL_ABOVE;
    }
    else if (0 == ADC_result)
    {
        return ADC_COMPARE_NOT_ENOUGH_SAMPLES;
    }
    else if (-1 == ADC_result)
    {
        return ADC_COMPARE_NOT_READY;
    }
    else if (-2 == ADC_result)
    {
        return ADC_COMPARE_DISABLED;
    }
    
    return ADC_COMPARE_DISABLED;
}
/**
 * @brief 电压采样通道初始化
 */
void ADC_VM_Init()
{
    stc_adc_init_t stcAdcInit;
    MEM_ZERO_STRUCT(stcAdcInit);

    // ADC硬件初始化（HC32F4平台）
    CLK_SetPeriClkSource(ClkPeriSrcPclk);
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

    stcAdcInit.enResolution = AdcResolution_12Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = AdcMode_SAOnce;
    ADC_Init(M4_ADC1, &stcAdcInit);

    // 注册电压通道到适配器层
    ADCH_t  VM_ADCH = {ADCH_VM_ADC, ADCH_VM_ADCH, ADCH_VM_PORT, ADCH_VM_PIN};
    adc_adapter_hInit(&hADCChannalMBV, &VM_ADCH);
    adc_adapter_Set_Channal_SmpTime(ADCH_VM_ADCH, SampleTime_vm);
    adc_adapter_Set_Channal_SmpIntervalTime(ADCH_VM_ADCH, SampleIntervalTime_vm);
    adc_adapter_Channel_Enable(ADCH_VM_ADCH);
}

/**
 * @brief 电压采样处理
 * @param VM_Voltage_ptr 输出电压值
 */
void ADC_VM(float* VM_Voltage_ptr)
{
    static bool firstCall = true;
    static float converted_voltages_vm[SAMPLE_BUFFER_SIZE_vm] = {0};
    static uint32_t converted_count = 0;

    // 首次调用初始化
    if (firstCall) {
        firstCall = false;
        adc_adapter_Reset_Sampling_Stats(ADCH_VM_ADCH);
    }
    
    // 执行电压采样
    if (hz_adc_adapter_SCM_1Ch_Convert_VM(ADCH_VM_ADCH)) 
    {
#ifdef hz_adc_debug
        uint16_t buffer_vm[SAMPLE_BUFFER_SIZE_vm];
        uint32_t count;
        adc_adapter_Get_Sample_Buffer(ADCH_VM_ADCH, buffer_vm, &count, SAMPLE_BUFFER_SIZE_vm);
        
        // 转换采样数据（调试用）
        converted_count = 0;
        for (uint32_t i = 0; i < count; i++) {
            float ADVM_Voltage_sample = (float)buffer_vm[i] * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
            float Voltage_off_set = (ADVM_Voltage_sample * (VM_R1 + VM_R2) / VM_R1)/ 11.0f;
            float VM_Voltage_sample = (ADVM_Voltage_sample * (VM_R1 + VM_R2) / VM_R1) + Voltage_off_set;
            converted_voltages_vm[converted_count++] = VM_Voltage_sample;
        }
#endif

        // 计算平均电压值
        uint16_t ADData_Voltage = adc_adapter_Get_Sampling_Average(ADCH_VM_ADCH);
        float ADVM_Voltage = (float)ADData_Voltage * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
        float Voltage_off_set = (ADVM_Voltage * (VM_R1 + VM_R2) / VM_R1) / 11.0f;
        *VM_Voltage_ptr = (ADVM_Voltage * (VM_R1 + VM_R2) / VM_R1) + Voltage_off_set;

        // 重置采样状态
        adc_adapter_Reset_Sampling_Stats(ADCH_VM_ADCH);
    }
}

