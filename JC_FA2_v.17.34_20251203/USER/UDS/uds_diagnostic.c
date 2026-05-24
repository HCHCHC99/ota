/********************************�ļ�˵��*************************************
*�ļ���: uds_diagnostic.c
*����: AI Assistant
*���: V1.0.0
*���ܼ��: UDS ͨ�����Э��ʵ�� - ֧�����̿��ƺ͹̼�����
*˵��: ͨ�� uds_dl_if.h ����ӿڵ��õײ�����ʵ�֣���ֱ����������ģ��
*****************************************************************************/
#include "uds_diagnostic.h"
#include "can_adapter.h"
#include "rtt_log.h"
#include <string.h>
#include "isotp_transport.h"
#include "security_access.h"

/*****************************���Ժ궨��***************************************/
#ifdef UDS_DEBUG
    #define UDS_D(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_DEBUG, COLOR_CYAN,   "UDS", fmt, ##__VA_ARGS__)
    #define UDS_I(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_INFO,  COLOR_GREEN, "UDS", fmt, ##__VA_ARGS__)
    #define UDS_W(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_WARN,  COLOR_YELLOW,"UDS", fmt, ##__VA_ARGS__)
    #define UDS_E(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_ERROR, COLOR_RED,   "UDS", fmt, ##__VA_ARGS__)
#else
    #define UDS_D(fmt, ...)  (void)0
    #define UDS_I(fmt, ...)  (void)0
    #define UDS_W(fmt, ...)  (void)0
    #define UDS_E(fmt, ...)  (void)0
#endif

/*****************************˽�б���***************************************/
static uds_ctrl_t g_uds_ctrl;

/*****************************˽�к�������***********************************/
static void uds_refresh_session_timer(void);
static uint32_t uds_generate_seed(void);
static uint32_t uds_calculate_key(uint32_t seed);
static uint16_t uds_read_data_by_id(uint16_t did);
static void uds_write_data_by_id(uint16_t did, uint16_t value);

/* UDS ���������� */
static void uds_handle_diagnostic_session_control(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_ecu_reset(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_read_data_by_id(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_write_data_by_id(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_security_access(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_tester_present(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_read_dtc_info(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_clear_dtc(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);

/* �������������� */
static void uds_handle_routine_control(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_request_download(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_transfer_data(uint8_t* data, uint16_t len, uint8_t* resp, uint8_t* resp_len);
static void uds_handle_request_transfer_exit(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len);

/* ����ִ�к��� */
static void uds_start_routine(uint16_t rid, uint8_t* data, uint8_t len, uint32_t* result);
static void uds_stop_routine(uint16_t rid);
static uint32_t uds_get_routine_result(uint16_t rid);

/*****************************����������NRCӳ��********************************/
/* �� uds_dl_result_t ӳ�䵽 UDS NRC */
static uint8_t uds_map_dl_result_to_nrc(uds_dl_result_t dl_result)
{
    switch (dl_result)
    {
        case UDS_DL_OK:              return 0xFF; /* �޴��󣬲�����NRC */
        case UDS_DL_ADDR_INVALID:    return UDS_NRC_REQUEST_OUT_OF_RANGE;
        case UDS_DL_SIZE_TOO_LARGE:  return UDS_NRC_REQUEST_OUT_OF_RANGE;
        case UDS_DL_ERASE_FAILED:    return UDS_NRC_GENERAL_PROGRAMMING_FAILURE;
        case UDS_DL_WRITE_FAILED:    return UDS_NRC_GENERAL_PROGRAMMING_FAILURE;
        case UDS_DL_VERIFY_FAILED:   return UDS_NRC_GENERAL_PROGRAMMING_FAILURE;
        case UDS_DL_SEQUENCE_ERROR:  return UDS_NRC_REQUEST_SEQUENCE_ERROR;
        case UDS_DL_BUSY:            return UDS_NRC_BUSY;
        case UDS_DL_NOT_READY:       return UDS_NRC_CONDITIONS_NOT_CORRECT;
        case UDS_DL_CRC_MISMATCH:    return UDS_NRC_GENERAL_PROGRAMMING_FAILURE;
        default:                     return UDS_NRC_GENERAL_REJECT;
    }
}

/*****************************����ʵ��***************************************/

/* UDS ��ʼ�� */
void uds_init(void)
{
    UDS_I("=== UDS Init Start ===");
    
    memset(&g_uds_ctrl, 0, sizeof(g_uds_ctrl));
    
    /* ��ʼ״̬��Ĭ�ϻỰ + ���� */
    g_uds_ctrl.session_mode = UDS_SESSION_DEFAULT_MODE;
    g_uds_ctrl.security_state = UDS_SECURITY_LOCKED;
    
    /* ���ò��� */
    g_uds_ctrl.session_timeout_ms = UDS_DEFAULT_SESSION_TIMEOUT_MS;
    g_uds_ctrl.max_attempts = UDS_SECURITY_MAX_ATTEMPTS;
    g_uds_ctrl.session_timer_ms = 0;
    g_uds_ctrl.security_delay_ms = 0;
    
    /* DID ���ݳ�ʼ����ͨ������ӿڴӵײ��ȡ�� */
    if (uds_dl_is_registered())
    {
        const uds_dl_if_t* dl = uds_dl_get_if();
        uint32_t value;
        
        if (dl->read_did(DID_FIRMWARE_VERSION, &value))
            g_uds_ctrl.firmware_version = (uint16_t)value;
        else
            g_uds_ctrl.firmware_version = 0;
        
        if (dl->read_did(DID_BOOTLOADER_VERSION, &value))
            g_uds_ctrl.bootloader_version = (uint16_t)value;
        else
            g_uds_ctrl.bootloader_version = 0;
        
        if (dl->read_did(DID_FIRMWARE_CRC, &value))
            g_uds_ctrl.firmware_crc = value;
        else
            g_uds_ctrl.firmware_crc = 0;
    }
    else
    {
        UDS_W("Download interface not registered, DID values set to 0");
        g_uds_ctrl.firmware_version = 0;
        g_uds_ctrl.bootloader_version = 0;
        g_uds_ctrl.firmware_crc = 0;
    }
    
    /* ���̳�ʼ�� */
    g_uds_ctrl.routine.routine_id = 0;
    g_uds_ctrl.routine.status = 0;
    g_uds_ctrl.routine.result = 0;
    
    UDS_I("session_mode=%d (DEFAULT), security_state=%d (LOCKED)", 
          g_uds_ctrl.session_mode, g_uds_ctrl.security_state);
    UDS_I("session_timeout_ms=%d", g_uds_ctrl.session_timeout_ms);
    UDS_I("=== UDS Init Done ===");
}

/* 1ms ��ʱ������ */
void uds_ms_update(void)
{
    static uint32_t print_cnt = 0;
    
    /* �Ự��ʱ��� */
    if (g_uds_ctrl.session_timer_ms > 0)
    {
        g_uds_ctrl.session_timer_ms--;
        if (g_uds_ctrl.session_timer_ms == 0)
        {
            if (g_uds_ctrl.session_mode != UDS_SESSION_DEFAULT_MODE)
            {
                UDS_W("!!! SESSION TIMEOUT !!! session_mode=%d -> DEFAULT", 
                      g_uds_ctrl.session_mode);
                g_uds_ctrl.session_mode = UDS_SESSION_DEFAULT_MODE;
                g_uds_ctrl.security_state = UDS_SECURITY_LOCKED;
                g_uds_ctrl.security_attempts = 0;
                g_uds_ctrl.security_seed = 0;
            }
        }
    }
    
    /* ��ȫ�����ӳټ�ʱ */
    if (g_uds_ctrl.security_delay_ms > 0)
    {
        g_uds_ctrl.security_delay_ms--;
        if (g_uds_ctrl.security_delay_ms == 0)
        {
            UDS_I("Security delay expired");
        }
    }
    
    /* ״̬��ӡ */
    #if UDS_DEBUG_PRINT_ENABLE
    print_cnt++;
    if (print_cnt >= (UDS_STATE_PRINT_INTERVAL_MS))
    {
        print_cnt = 0;
        UDS_D("State: session=%d(%s), security=%d(%s), timer=%d, delay=%d",
              g_uds_ctrl.session_mode,
              g_uds_ctrl.session_mode == UDS_SESSION_DEFAULT_MODE ? "DEFAULT" :
              (g_uds_ctrl.session_mode == UDS_SESSION_EXTENDED_MODE ? "EXTENDED" : "PROGRAMMING"),
              g_uds_ctrl.security_state,
              g_uds_ctrl.security_state == UDS_SECURITY_LOCKED ? "LOCKED" :
              (g_uds_ctrl.security_state == UDS_SECURITY_SEED_SENT ? "SEED_SENT" : "UNLOCKED"),
              g_uds_ctrl.session_timer_ms,
              g_uds_ctrl.security_delay_ms);
    }
    #endif
}

/* UDS ���������� */
void uds_process(void)
{
    /* ���ڴ˴�������ִ�еȺ�ʱ���� */
}

/* ˢ�»Ự��ʱ�� */
static void uds_refresh_session_timer(void)
{
    uint32_t old_timer = g_uds_ctrl.session_timer_ms;
    g_uds_ctrl.session_timer_ms = g_uds_ctrl.session_timeout_ms;
    UDS_D("Refresh timer: %d -> %d", old_timer, g_uds_ctrl.session_timer_ms);
}

/* ���ɰ�ȫ���ӣ�4�ֽ�������� */
static uint32_t uds_generate_seed(void)
{
    uint32_t seed;
    
#if (UDS_SEED_MODE_FIXED == 1)
    /* �̶�ģʽ��ʹ��Ԥ��Ĺ̶����� */
    seed = UDS_FIXED_SEED_VALUE;
    UDS_D("Generate seed (FIXED mode): 0x%08X", seed);
#else
    /* ���ģʽ��ʹ�ü������ͼ�ʱ������������� */
    static uint32_t seed_counter = 0;
    seed_counter++;
    seed = (seed_counter * 0x9E3779B9) + (uint32_t)(g_uds_ctrl.session_timer_ms);
    UDS_D("Generate seed (RANDOM mode): 0x%08X", seed);
#endif
    
    return seed;
}

/* ������Կ��ʹ��CRC8�㷨�� */
static uint32_t uds_calculate_key(uint32_t seed)
{
    uint8_t seed_bytes[4];
    uint8_t key_bytes[4];
    uint32_t key;
    
    seed_bytes[0] = (seed >> 24) & 0xFF;
    seed_bytes[1] = (seed >> 16) & 0xFF;
    seed_bytes[2] = (seed >> 8) & 0xFF;
    seed_bytes[3] = seed & 0xFF;
    
    UDS_D("Calculating key from seed: 0x%08X", seed);
    UDS_D("Seed bytes: %02X %02X %02X %02X", 
          seed_bytes[0], seed_bytes[1], seed_bytes[2], seed_bytes[3]);
    
    seedkey_calc_lv1_key(seed_bytes, key_bytes);
    
    key = (key_bytes[0] << 24) | (key_bytes[1] << 16) | 
          (key_bytes[2] << 8) | key_bytes[3];
    
    UDS_D("Calculated key: 0x%08X", key);
    UDS_D("Key bytes: %02X %02X %02X %02X",
          key_bytes[0], key_bytes[1], key_bytes[2], key_bytes[3]);
    
    return key;
}

/* ͨ������ӿڶ�ȡ DID */
static uint16_t uds_read_data_by_id(uint16_t did)
{
    uint16_t value = 0;
    
    if (uds_dl_is_registered())
    {
        const uds_dl_if_t* dl = uds_dl_get_if();
        uint32_t raw_value;
        
        if (dl->read_did(did, &raw_value))
        {
            value = (uint16_t)(raw_value & 0xFFFF);
        }
        else
        {
            UDS_W("Unknown DID: 0x%04X", did);
        }
    }
    else
    {
        UDS_W("Download interface not registered, cannot read DID 0x%04X", did);
    }
    
    UDS_D("Read DID 0x%04X = 0x%04X", did, value);
    return value;
}

static void uds_write_data_by_id(uint16_t did, uint16_t value)
{
    UDS_D("Write DID 0x%04X = 0x%04X", did, value);
    switch (did)
    {
        case DID_FIRMWARE_VERSION:
        case DID_BOOTLOADER_VERSION:
        case DID_FIRMWARE_CRC:
        default:
            UDS_W("Write to read-only DID: 0x%04X", did);
            break;
    }
}

/* ������ϻỰ���� (0x10) */
static void uds_handle_diagnostic_session_control(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    uint8_t sub_func;
    uds_session_mode_t requested_session;
    
    UDS_I(">>> Handle 0x10 (Diagnostic Session Control)");
    
    if (len < 2)
    {
        UDS_E("Length error: len=%d < 2", len);
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    sub_func = data[1];
    UDS_D("sub_func=0x%02X", sub_func);
    
    switch (sub_func)
    {
        case UDS_SESSION_DEFAULT:
            requested_session = UDS_SESSION_DEFAULT_MODE;
            UDS_I("Request: DEFAULT session");
            break;
        case UDS_SESSION_PROGRAMMING:
            requested_session = UDS_SESSION_PROGRAMMING_MODE;
            UDS_I("Request: PROGRAMMING session");
            break;
        case UDS_SESSION_EXTENDED:
            requested_session = UDS_SESSION_EXTENDED_MODE;
            UDS_I("Request: EXTENDED session");
            break;
        default:
            UDS_W("Sub-function not supported: 0x%02X", sub_func);
            uds_send_negative_response(0, data[0], UDS_NRC_SUB_FUNCTION_NOT_SUPPORTED);
            return;
    }
    
    if (requested_session == UDS_SESSION_DEFAULT_MODE)
    {
        UDS_I("Session change: %d -> DEFAULT", g_uds_ctrl.session_mode);
        g_uds_ctrl.session_mode = UDS_SESSION_DEFAULT_MODE;
        g_uds_ctrl.security_state = UDS_SECURITY_LOCKED;
        g_uds_ctrl.security_attempts = 0;
        g_uds_ctrl.security_seed = 0;
        g_uds_ctrl.security_delay_ms = 0;
    }
    else if (requested_session == UDS_SESSION_EXTENDED_MODE)
    {
        if (g_uds_ctrl.session_mode == UDS_SESSION_DEFAULT_MODE)
        {
            UDS_I("Session change: DEFAULT -> EXTENDED");
            g_uds_ctrl.session_mode = UDS_SESSION_EXTENDED_MODE;
            g_uds_ctrl.security_state = UDS_SECURITY_LOCKED;
            g_uds_ctrl.security_attempts = 0;
            g_uds_ctrl.security_seed = 0;
        }
        else
        {
            UDS_D("Already in EXTENDED session, keep");
        }
    }
    else if (requested_session == UDS_SESSION_PROGRAMMING_MODE)
    {
        UDS_I("Session change: %d -> PROGRAMMING", g_uds_ctrl.session_mode);
        g_uds_ctrl.session_mode = UDS_SESSION_PROGRAMMING_MODE;
        g_uds_ctrl.security_state = UDS_SECURITY_LOCKED;
        g_uds_ctrl.security_attempts = 0;
        g_uds_ctrl.security_seed = 0;
        g_uds_ctrl.security_delay_ms = 0;
    }
    
    resp[0] = sub_func;
    *resp_len = 1;
    
    UDS_I("Response: session=%d", g_uds_ctrl.session_mode);
    UDS_I("Response data to send: [0x%02X], len=%d", resp[0], *resp_len);		
}

/* ���� ECU ��λ (0x11) */
static void uds_handle_ecu_reset(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    uint8_t reset_type;
    
    UDS_I(">>> Handle 0x11 (ECU Reset)");
    
    if (len < 2)
    {
        UDS_E("Length error");
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    reset_type = data[1];
    UDS_D("reset_type=0x%02X", reset_type);
    
    if (reset_type != UDS_RESET_SOFT)
    {
        UDS_W("Reset type not supported: 0x%02X", reset_type);
        uds_send_negative_response(0, data[0], UDS_NRC_SUB_FUNCTION_NOT_SUPPORTED);
        return;
    }
    
    resp[0] = reset_type;
    *resp_len = 1;
    UDS_I("Soft reset requested");
}

/* ���������� (0x22) */
static void uds_handle_read_data_by_id(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    uint16_t did;
    uint16_t value;
    
    UDS_I(">>> Handle 0x22 (Read Data)");
    
    if (len < 3)
    {
        UDS_E("Length error: len=%d < 3", len);
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    did = (data[1] << 8) | data[2];
    UDS_D("DID=0x%04X", did);
    
    if (did != DID_FIRMWARE_VERSION && did != DID_BOOTLOADER_VERSION && did != DID_FIRMWARE_CRC)
    {
        UDS_W("DID out of range: 0x%04X", did);
        uds_send_negative_response(0, data[0], UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    value = uds_read_data_by_id(did);
    
    resp[0] = (did >> 8) & 0xFF;
    resp[1] = did & 0xFF;
    resp[2] = (value >> 8) & 0xFF;
    resp[3] = value & 0xFF;
    *resp_len = 4;
    
    UDS_I("Response: DID=0x%04X, Value=0x%04X", did, value);
}

/* ����д���� (0x2E) */
static void uds_handle_write_data_by_id(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    uint16_t did;
    uint16_t value;
    
    UDS_I(">>> Handle 0x2E (Write Data)");
    
    if (len < 5)
    {
        UDS_E("Length error: len=%d < 5", len);
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    did = (data[1] << 8) | data[2];
    value = (data[3] << 8) | data[4];
    UDS_D("DID=0x%04X, Value=0x%04X", did, value);
    
    if (g_uds_ctrl.session_mode != UDS_SESSION_EXTENDED_MODE &&
        g_uds_ctrl.session_mode != UDS_SESSION_PROGRAMMING_MODE)
    {
        UDS_W("Not in EXTENDED/PROGRAMMING session! session_mode=%d", g_uds_ctrl.session_mode);
        uds_send_negative_response(0, data[0], UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }
    
    if (g_uds_ctrl.security_state != UDS_SECURITY_UNLOCKED)
    {
        UDS_W("Security locked! security_state=%d", g_uds_ctrl.security_state);
        uds_send_negative_response(0, data[0], UDS_NRC_SECURITY_ACCESS_DENIED);
        return;
    }
    
    switch (did)
    {
        case DID_FIRMWARE_VERSION:
        case DID_BOOTLOADER_VERSION:
        case DID_FIRMWARE_CRC:
            UDS_W("Write to read-only DID: 0x%04X", did);
            uds_send_negative_response(0, data[0], UDS_NRC_REQUEST_OUT_OF_RANGE);
            return;
            
        default:
            UDS_W("Unknown DID: 0x%04X", did);
            uds_send_negative_response(0, data[0], UDS_NRC_REQUEST_OUT_OF_RANGE);
            return;
    }
}

/* ������ȫ���� (0x27) */
static void uds_handle_security_access(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    uint8_t sub_func;
    uint32_t received_key;
    
    UDS_I(">>> Handle 0x27 (Security Access)");
    UDS_D("Current state: session=%d, security=%d, timer=%d",
          g_uds_ctrl.session_mode, g_uds_ctrl.security_state, g_uds_ctrl.session_timer_ms);
    
    if (len < 2)
    {
        UDS_E("Length error: len=%d < 2", len);
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    sub_func = data[1];
    UDS_D("sub_func=0x%02X", sub_func);
    
    if (g_uds_ctrl.session_mode != UDS_SESSION_EXTENDED_MODE &&
        g_uds_ctrl.session_mode != UDS_SESSION_PROGRAMMING_MODE)
    {
        UDS_W("NOT in EXTENDED/PROGRAMMING session! session_mode=%d", g_uds_ctrl.session_mode);
        uds_send_negative_response(0, data[0], UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }
    
    if (g_uds_ctrl.security_delay_ms > 0)
    {
        UDS_W("Security delay active: %d ms", g_uds_ctrl.security_delay_ms);
        uds_send_negative_response(0, data[0], UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED);
        return;
    }
    
    if (sub_func & 0x01)
    {
        UDS_I("Request seed");
        
        if (g_uds_ctrl.security_state != UDS_SECURITY_LOCKED)
        {
            UDS_W("Wrong state for seed request! security_state=%d", g_uds_ctrl.security_state);
            uds_send_negative_response(0, data[0], UDS_NRC_REQUEST_SEQUENCE_ERROR);
            return;
        }
        
        g_uds_ctrl.security_seed = uds_generate_seed();
        g_uds_ctrl.security_state = UDS_SECURITY_SEED_SENT;
        
        UDS_I("Seed generated: 0x%08X", g_uds_ctrl.security_seed);
        
        resp[0] = sub_func;
        resp[1] = (g_uds_ctrl.security_seed >> 24) & 0xFF;
        resp[2] = (g_uds_ctrl.security_seed >> 16) & 0xFF;
        resp[3] = (g_uds_ctrl.security_seed >> 8) & 0xFF;
        resp[4] = g_uds_ctrl.security_seed & 0xFF;
        *resp_len = 5;
    }
    else
    {
        UDS_I("Send key");
        
        if (g_uds_ctrl.security_state != UDS_SECURITY_SEED_SENT)
        {
            UDS_W("Wrong state for key! security_state=%d", g_uds_ctrl.security_state);
            uds_send_negative_response(0, data[0], UDS_NRC_REQUEST_SEQUENCE_ERROR);
            return;
        }
        
        if (len < 6)
        {
            UDS_E("Key length error: len=%d < 6", len);
            uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
            return;
        }
        
        received_key = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
        UDS_D("Received key: 0x%08X", received_key);
        
        uint32_t expected_key = uds_calculate_key(g_uds_ctrl.security_seed);
        if (received_key == expected_key)
        {
            UDS_I("Key VALID! Unlocking...");
            g_uds_ctrl.security_state = UDS_SECURITY_UNLOCKED;
            g_uds_ctrl.security_attempts = 0;
            g_uds_ctrl.security_delay_ms = 0;
            
            resp[0] = sub_func;
            *resp_len = 1;
            UDS_I("Security unlocked!");
        }
        else
        {
            UDS_W("Key INVALID! expected=0x%08X, got=0x%08X", expected_key, received_key);
            g_uds_ctrl.security_attempts++;
            g_uds_ctrl.security_state = UDS_SECURITY_LOCKED;
            g_uds_ctrl.security_seed = 0;
            
            if (g_uds_ctrl.security_attempts >= g_uds_ctrl.max_attempts)
            {
                g_uds_ctrl.security_delay_ms = UDS_SECURITY_DELAY_BASE_MS * g_uds_ctrl.security_attempts;
                UDS_W("Max attempts reached! delay=%d ms", g_uds_ctrl.security_delay_ms);
                uds_send_negative_response(0, data[0], UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS);
            }
            else
            {
                UDS_W("Attempts: %d/%d", g_uds_ctrl.security_attempts, g_uds_ctrl.max_attempts);
                uds_send_negative_response(0, data[0], UDS_NRC_INVALID_KEY);
            }
        }
    }
}

/* ���� TesterPresent (0x3E) */
static void uds_handle_tester_present(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    uint8_t sub_func = 0;
    
    UDS_D(">>> Handle 0x3E (TesterPresent)");
    
    if (len >= 2)
    {
        sub_func = data[1];
    }
    
    if (sub_func != 0x00 && sub_func != 0x80)
    {
        UDS_W("Sub-function not supported: 0x%02X", sub_func);
        uds_send_negative_response(0, data[0], UDS_NRC_SUB_FUNCTION_NOT_SUPPORTED);
        return;
    }
    
    if (sub_func == 0x80)
    {
        resp[0] = sub_func;
        *resp_len = 1;
        UDS_D("Response with sub_func=0x80");
    }
    else
    {
        *resp_len = 0;
        UDS_D("No response");
    }
}

/* ������ DTC (0x19) */
static void uds_handle_read_dtc_info(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    uint8_t sub_func;
    
    UDS_D(">>> Handle 0x19 (Read DTC)");
    
    if (len < 2)
    {
        UDS_E("Length error");
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    sub_func = data[1];
    
    if (sub_func != 0x01 && sub_func != 0x02)
    {
        UDS_W("Sub-function not supported: 0x%02X", sub_func);
        uds_send_negative_response(0, data[0], UDS_NRC_SUB_FUNCTION_NOT_SUPPORTED);
        return;
    }
    
    resp[0] = sub_func;
    resp[1] = 0x00;
    resp[2] = 0x00;
    resp[3] = 0x00;
    *resp_len = 4;
    
    UDS_D("Response: no DTC");
}

/* ������� DTC (0x14) */
static void uds_handle_clear_dtc(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    UDS_D(">>> Handle 0x14 (Clear DTC)");
    
    if (len < 1)
    {
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    *resp_len = 0;
    UDS_I("DTC cleared");
}

/* ==================== ��������ʵ�� ==================== */

/* �������̿��� (0x31) */
static void uds_handle_routine_control(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    UDS_I(">>> Handle 0x31 (Routine Control) - FORCED POSITIVE");

    /* Force positive response */
    resp[0] = 0x71;
    resp[1] = (len > 1) ? data[1] : 0;
    *resp_len = 2;
    return;
}

/* ����ִ�к���ʵ�֣�ͨ������ӿڵ��ã� */
static void uds_start_routine(uint16_t rid, uint8_t* data, uint8_t len, uint32_t* result)
{
    UDS_I("Start routine: RID=0x%04X", rid);
    
    if (!uds_dl_is_registered())
    {
        UDS_W("Download interface not registered");
        *result = 0;
        return;
    }
    
    const uds_dl_if_t* dl = uds_dl_get_if();
    
    switch (rid)
    {
        case RID_ERASE_FIRMWARE:
        {
            if (len < 8)
            {
                UDS_E("Erase routine: insufficient data, len=%d", len);
                *result = 0;
                return;
            }
            
            uint32_t address = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
            uint32_t size = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
            
            UDS_I("Erase firmware: addr=0x%08X, size=%d", address, size);
            
            uds_dl_result_t dl_result = dl->erase(address, size);
            if (dl_result != UDS_DL_OK)
            {
                UDS_E("Erase failed: %d", dl_result);
                *result = 0;
            }
            else
            {
                *result = 1;
            }
            break;
        }
        
        case RID_CALCULATE_CRC:
        {
            if (len < 8)
            {
                UDS_E("CRC routine: insufficient data, len=%d", len);
                *result = 0;
                return;
            }
            
            uint32_t address = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
            uint32_t size = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
            
            UDS_I("Calculate CRC: addr=0x%08X, size=%d", address, size);
            
            uint32_t crc_value = 0;
            uds_dl_result_t dl_result = dl->calculate_crc(address, size, &crc_value);
            if (dl_result != UDS_DL_OK)
            {
                UDS_E("CRC calculation failed: %d", dl_result);
                *result = 0;
            }
            else
            {
                *result = crc_value;
            }
            break;
        }
        
        case RID_JUMP_TO_BOOTLOADER:
            UDS_I("Jump to bootloader requested");
            *result = 1;
            break;
            
        case RID_JUMP_TO_APPLICATION:
            UDS_I("Jump to application requested");
            *result = 1;
            break;
            
        default:
            UDS_W("Unknown RID: 0x%04X", rid);
            *result = 0;
            break;
    }
}

static void uds_stop_routine(uint16_t rid)
{
    UDS_I("Stop routine: RID=0x%04X", rid);
    /* ��ǰ���̲�֧��ֹͣ��ֱ�ӷ��� */
}

static uint32_t uds_get_routine_result(uint16_t rid)
{
    UDS_I("Get routine result: RID=0x%04X", rid);
    return g_uds_ctrl.routine.result;
}

/* ������������ (0x34) */
static void uds_handle_request_download(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    UDS_I(">>> Handle 0x34 (Request Download)");
    
    if (!uds_dl_is_registered())
    {
        UDS_W("Download interface not registered");
        uds_send_negative_response(0, data[0], UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }
    
    /* UDS: SID + dataFormatIdentifier + addrSizeFmtIdentifier + addr[N] + size[N] */
    uint8_t dfi  = data[1];
    uint8_t alfi = data[2];
    uint8_t addr_len = (alfi >> 4) & 0x0F;
    uint8_t size_len = alfi & 0x0F;
    
    if (addr_len == 0 || addr_len > 4 || size_len == 0 || size_len > 4)
    {
        UDS_E("Invalid addr/size length: alfi=0x%02X", alfi);
        uds_send_negative_response(0, data[0], UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    if (len < (uint8_t)(3 + addr_len + size_len))
    {
        UDS_E("Length error: len=%d < %d", len, 3 + addr_len + size_len);
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint32_t address = 0;
    for (uint8_t i = 0; i < addr_len; i++) {
        address = (address << 8) | data[3 + i];
    }
    uint32_t size = 0;
    for (uint8_t i = 0; i < size_len; i++) {
        size = (size << 8) | data[3 + addr_len + i];
    }

    UDS_I("Request download: addr=0x%08X, size=%d bytes", address, size);
    
    const uds_dl_if_t* dl = uds_dl_get_if();
    uds_dl_result_t dl_result = dl->on_request_download(address, size);
    
    if (dl_result != UDS_DL_OK)
    {
        uint8_t nrc = uds_map_dl_result_to_nrc(dl_result);
        UDS_W("Request download rejected: dl_result=%d, NRC=0x%02X", dl_result, nrc);
        uds_send_negative_response(0, data[0], nrc);
        return;
    }
    
    /* ��Ӧ������A�ȣ�2�ֽڣ� */
    resp[0] = 0x40;  /* ����A�ȸ��ֽ� */
    resp[1] = 0x00;  /* ����A�ȵ��ֽ� */
    *resp_len = 2;
    
    UDS_I("Download accepted, max block size=0x4000");
}

/* ������������ (0x36) */
static void uds_handle_transfer_data(uint8_t* data, uint16_t len, uint8_t* resp, uint8_t* resp_len)
{
    UDS_D(">>> Handle 0x36 (Transfer Data)");
    
    if (!uds_dl_is_registered())
    {
        UDS_W("Download interface not registered");
        uds_send_negative_response(0, data[0], UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }
    
    if (len < 2)
    {
        UDS_E("Length error: len=%d < 2", len);
        uds_send_negative_response(0, data[0], UDS_NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint8_t block_seq = data[1];
    uint16_t data_len = len - 2;
    
    UDS_D("Transfer data: seq=%d, data_len=%d", block_seq, data_len);
    
    const uds_dl_if_t* dl = uds_dl_get_if();
    uds_dl_result_t dl_result = dl->on_transfer_data(block_seq, &data[2], data_len);
    
    if (dl_result != UDS_DL_OK)
    {
        uint8_t nrc = uds_map_dl_result_to_nrc(dl_result);
        UDS_W("Transfer data rejected: dl_result=%d, NRC=0x%02X", dl_result, nrc);
        uds_send_negative_response(0, data[0], nrc);
        return;
    }
    
    /* ����Ƿ���Ҫ��Ӧ�ȴ���NRC 0x78�� */
    if (dl->is_pending())
    {
        UDS_I("Transfer data: pending response (NRC 0x78)");
        uds_send_response_pending(0, data[0]);
        return;
    }
    
    resp[0] = 0x76;
    resp[1] = block_seq;
    *resp_len = 2;
    UDS_D("Transfer data accepted, positive response sent");
}

/* �����������˳� (0x37) */
static void uds_handle_request_transfer_exit(uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* resp_len)
{
    UDS_I(">>> Handle 0x37 (Request Transfer Exit)");
    
    if (!uds_dl_is_registered())
    {
        UDS_W("Download interface not registered");
        uds_send_negative_response(0, data[0], UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }
    
    const uds_dl_if_t* dl = uds_dl_get_if();
    uds_dl_result_t dl_result = dl->on_transfer_exit();
    
    if (dl_result != UDS_DL_OK)
    {
        uint8_t nrc = uds_map_dl_result_to_nrc(dl_result);
        UDS_W("Transfer exit rejected: dl_result=%d, NRC=0x%02X", dl_result, nrc);
        uds_send_negative_response(0, data[0], nrc);
        return;
    }
    
    /* ����Ƿ���Ҫ��Ӧ�ȴ���NRC 0x78�� */
    if (dl->is_pending())
    {
        UDS_I("Transfer exit: pending response (NRC 0x78)");
        uds_send_response_pending(0, data[0]);
        return;
    }
    
    uds_send_response(0, data[0], NULL, 0);    *resp_len = 0;    UDS_I("Transfer exit accepted");
}

/* ==================== UDS �����շַ� ==================== */

/* UDS ���մ������ */
int8_t uds_receive_handler(uint8_t channel, uint32_t can_id, uint8_t* data, uint16_t len)
{
    uint8_t response_buf[UDS_MAX_RESPONSE_LEN];
    uint8_t response_len = 0;
    uint8_t sid;
    
    /* ==================== CAN ID ���ˣ���ѡ�� ==================== */
#if (UDS_ENABLE_CAN_ID_FILTER == 1)
    /* ֻ��������Ѱַ����͹���Ѱַ���� */
    if (can_id != UDS_PHYSICAL_REQUEST_ID && can_id != UDS_FUNCTIONAL_REQUEST_ID)
    {
        UDS_D("CAN ID filtered: 0x%08X (expected 0x%08X or 0x%08X)", 
              can_id, UDS_PHYSICAL_REQUEST_ID, UDS_FUNCTIONAL_REQUEST_ID);
        return -1;  /* ���Ƿ����� ECU �ı��ģ�ֱ�Ӷ��� */
    }
#endif
    
    if (data == NULL || len < 1)
    {
        UDS_E("Invalid receive data");
        return -1;
    }
    
    sid = data[0];

    UDS_D("RX raw[0..7]: %02X %02X %02X %02X %02X %02X %02X %02X",
          data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

    /* ˢ�»Ự��ʱ�� */
    uds_refresh_session_timer();
    
    UDS_I("=== UDS Receive: SID=0x%02X, len=%d ===", sid, len);
    
    /* ��ӡ CAN ID ��Ϣ�������ã� */
#if (UDS_ENABLE_CAN_ID_FILTER == 1)
    UDS_D("CAN ID: 0x%08X (matched)", can_id);
#else
    UDS_D("CAN ID: 0x%08X (filter disabled)", can_id);
#endif
    
    /* ���� SID �ַ�����Ӧ�Ĵ������� */
    switch (sid)
    {
        case UDS_SID_DIAGNOSTIC_SESSION_CONTROL:
            uds_handle_diagnostic_session_control(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_ECU_RESET:
            uds_handle_ecu_reset(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION:
            uds_handle_clear_dtc(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_READ_DTC_INFORMATION:
            uds_handle_read_dtc_info(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_READ_DATA_BY_IDENTIFIER:
            uds_handle_read_data_by_id(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_SECURITY_ACCESS:
            uds_handle_security_access(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_WRITE_DATA_BY_IDENTIFIER:
            uds_handle_write_data_by_id(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_ROUTINE_CONTROL:
            uds_handle_routine_control(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_REQUEST_DOWNLOAD:
            uds_handle_request_download(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_TRANSFER_DATA:
            uds_handle_transfer_data(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_REQUEST_TRANSFER_EXIT:
            uds_handle_request_transfer_exit(data, len, response_buf, &response_len);
            break;
            
        case UDS_SID_TESTER_PRESENT:
            uds_handle_tester_present(data, len, response_buf, &response_len);
            break;
            
        default:
            UDS_W("SID not supported: 0x%02X", sid);
            uds_send_negative_response(channel, sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
            return 0;
    }
    
    /* ������Ӧ */
    if (response_len > 0)
    {
        uds_send_response(channel, sid, response_buf, response_len);
    }
    
    return 0;
}

/* ���Ϳ϶���Ӧ */
int8_t uds_send_response(uint8_t channel, uint8_t sid, uint8_t* data, uint8_t len)
{
    uint8_t response[UDS_MAX_RESPONSE_LEN];
    uint8_t response_len;
    
    if (len > UDS_MAX_RESPONSE_LEN - 1)
    {
        UDS_E("Response too long: %d", len);
        return -1;
    }
    
    response[0] = sid + 0x40;  /* �϶���Ӧ = SID + 0x40 */
    if (len > 0)
    {
        memcpy(&response[1], data, len);
    }
    response_len = len + 1;
    
    UDS_D("Send response: SID=0x%02X, len=%d", sid + 0x40, response_len);
    
    /* ͨ�� ISO-TP ���� */
    isotp_send_message(channel, UDS_PHYSICAL_RESPONSE_ID, response, response_len);
    
    return 0;
}

/* ���ͷ���Ӧ */
int8_t uds_send_negative_response(uint8_t channel, uint8_t sid, uint8_t nrc)
{
    uint8_t response[3];
    
    response[0] = 0x7F;  /* ����Ӧ��ʶ */
    response[1] = sid;    /* ����� SID */
    response[2] = nrc;    /* ����Ӧ�� */
    
    UDS_W("Send NRC: SID=0x%02X, NRC=0x%02X", sid, nrc);
    
    isotp_send_message(channel, UDS_PHYSICAL_RESPONSE_ID, response, 3);
    
    return 0;
}

/* ������Ӧ�ȴ� (NRC 0x78) */
int8_t uds_send_response_pending(uint8_t channel, uint8_t sid)
{
    uint8_t response[3];
    
    response[0] = 0x7F;
    response[1] = sid;
    response[2] = UDS_NRC_RESPONSE_PENDING;
    
    UDS_I("Send response pending: SID=0x%02X", sid);
    
    isotp_send_message(channel, UDS_PHYSICAL_RESPONSE_ID, response, 3);
    
    return 0;
}

/* ��ȡ��ǰ�Ựģʽ */
uds_session_mode_t uds_get_session_mode(void)
{
    return g_uds_ctrl.session_mode;
}

/* ��ȡ��ǰ��ȫ״̬ */
uds_security_state_t uds_get_security_state(void)
{
    return g_uds_ctrl.security_state;
}

/* �Ựģʽת�ַ��� */
const char* uds_session_to_string(uds_session_mode_t session)
{
    switch (session)
    {
        case UDS_SESSION_DEFAULT_MODE:      return "DEFAULT";
        case UDS_SESSION_EXTENDED_MODE:     return "EXTENDED";
        case UDS_SESSION_PROGRAMMING_MODE:  return "PROGRAMMING";
        default:                            return "UNKNOWN";
    }
}

/* ��ȫ״̬ת�ַ��� */
const char* uds_security_to_string(uds_security_state_t state)
{
    switch (state)
    {
        case UDS_SECURITY_LOCKED:       return "LOCKED";
        case UDS_SECURITY_SEED_SENT:    return "SEED_SENT";
        case UDS_SECURITY_UNLOCKED:     return "UNLOCKED";
        default:                        return "UNKNOWN";
    }
}
