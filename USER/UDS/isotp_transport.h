/********************************�ļ�˵��*************************************
*�ļ���: isotp_transport.h
*����: AI Assistant
*�汾: V1.0.0
*���ܼ��: ISO 15765-2 �����Э��ʵ��
*��ע: ֧�ֳ����ĵķְ����ͺͽ���
*****************************************************************************/
#ifndef ISOTP_TRANSPORT_H_
#define ISOTP_TRANSPORT_H_

/*****************************�ļ�����(����)*********************************/
#include "stdint.h"
#include "stdbool.h"

/*****************************���Ժ궨��***************************************/
// #define ISO_DEBUG
#ifdef ISO_DEBUG
    #define ISOTP_D(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_DEBUG, COLOR_CYAN,   "ISOTP", fmt, ##__VA_ARGS__)
    #define ISOTP_I(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_INFO,  COLOR_GREEN, "ISOTP", fmt, ##__VA_ARGS__)
    #define ISOTP_W(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_WARN,  COLOR_YELLOW,"ISOTP", fmt, ##__VA_ARGS__)
    #define ISOTP_E(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_ERROR, COLOR_RED,   "ISOTP", fmt, ##__VA_ARGS__)
#else
    #define ISOTP_D(fmt, ...)  (void)0
    #define ISOTP_I(fmt, ...)  (void)0
    #define ISOTP_W(fmt, ...)  (void)0
    #define ISOTP_E(fmt, ...)  (void)0
#endif

/* OTA ���Դ�ӡ����ӡ��ע�� CAN ID ֡������ţ� */
#define OTA_DEBUG
#ifdef OTA_DEBUG
    #define OTA_D(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_DEBUG, COLOR_CYAN,   "OTA", fmt, ##__VA_ARGS__)
    #define OTA_I(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_INFO,  COLOR_GREEN, "OTA", fmt, ##__VA_ARGS__)
    #define OTA_W(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_WARN,  COLOR_YELLOW,"OTA", fmt, ##__VA_ARGS__)
    #define OTA_E(fmt, ...)  LOG_CH(LOG_CH_MAIN, LOG_LEVEL_ERROR, COLOR_RED,   "OTA", fmt, ##__VA_ARGS__)
#else
    #define OTA_D(fmt, ...)  (void)0
    #define OTA_I(fmt, ...)  (void)0
    #define OTA_W(fmt, ...)  (void)0
    #define OTA_E(fmt, ...)  (void)0
#endif

/***************************** CAN ID ���˼�¼���� ****************************/
/* �Ƿ����� CAN ID ���˼�¼����
 * 0: ����
 * 1: ����
 */
#define ISOTP_ENABLE_FILTER_RECORD    1


/* �Զ�����֡�ظ��������ض������� */
// #define ISOTP_AUTO_FC  /* Disabled: use normal ISO-TP path */

/* ��ע�� CAN ID �б������������ӣ� */
#define ISOTP_FILTER_CAN_ID_COUNT     3
#define ISOTP_FILTER_CAN_ID_LIST      {0x18DA03F1, 0x18DAF103, 0x18FF8118}

/* ���˼�¼��������С�����λ�������*/
#define ISOTP_FILTER_BUFFER_SIZE      64

/******************************�궨��(����)***********************************/

/* ISO-TP ֡���ͱ�ʶ */
#define ISOTP_FRAME_SINGLE           0x00    /* ��֡ (SF) */
#define ISOTP_FRAME_FIRST            0x10    /* ��֡ (FF) */
#define ISOTP_FRAME_CONSECUTIVE      0x20    /* ����֡ (CF) */
#define ISOTP_FRAME_FLOW_CONTROL     0x30    /* ����֡ (FC) */

/* ����״̬ (Flow Status) */
#define ISOTP_FC_CTS                 0x00    /* Continue to Send, �������� */
#define ISOTP_FC_WAIT                0x01    /* Wait, �ȴ� */
#define ISOTP_FC_OVERFLOW            0x02    /* Overflow, ��� */

/* ����ֵ */
#define ISOTP_OK                     0       /* �ɹ� */
#define ISOTP_BUSY                   1       /* æ�������У� */
#define ISOTP_ERROR                 -1       /* ���� */
#define ISOTP_TIMEOUT               -2       /* ��ʱ */
#define ISOTP_INCOMPLETE            -3       /* ���������ȴ��������ݣ� */

/* ���ò��� */
#define ISOTP_BUFFER_SIZE            8192    /* Receive buffer (8KB) */
#define ISOTP_DEFAULT_RESPONSE_ID   0x18DAF103
#define ISOTP_RX_TIMEOUT_MS          65535   /* ���ճ�ʱʱ�� (ms) */

/* ���ز��� (����ģʽ) */
#define ISOTP_DEFAULT_BLOCK_SIZE     0       /* BS = 0 (send all at once) */
#define ISOTP_DEFAULT_ST_MIN_MS      5       /* STmin = 5ms */

/* ���֧�ֵ���Ϣ���� */
#define ISOTP_MAX_MESSAGE_LEN        (4095)  /* ISO-TP ��֡���֧�� 4095 �ֽ� */

/**************************�������ͼ��ṹ����(����)***************************/

/* ISO-TP ����״̬�� */
typedef enum
{
    ISOTP_RX_IDLE = 0,          /* ���У��ȴ���֡ */
    ISOTP_RX_ACTIVE,            /* �����У����ڽ�������֡ */
    ISOTP_RX_WAIT_FC,           /* ����ģʽ���ȴ�����֡ */
    ISOTP_RX_COMPLETE,          /* ������� */
    ISOTP_RX_TIMEOUT            /* ���ճ�ʱ */
} isotp_rx_state_t;

/* ISO-TP ����״̬�� */
typedef enum
{
    ISOTP_TX_IDLE = 0,          /* ���� */
    ISOTP_TX_SENDING_FF,        /* �ѷ�����֡���ȴ����ػ�������֡ */
    ISOTP_TX_SENDING_CF,        /* ��������֡�� */
    ISOTP_TX_COMPLETE,          /* ������� */
    ISOTP_TX_TIMEOUT            /* ���ͳ�ʱ */
} isotp_tx_state_t;

/* ISO-TP ���ӽṹ�� (�����ӣ���֧�ֲ���) */
typedef struct
{
    /* ������� */
    isotp_rx_state_t rx_state;          /* ����״̬ */
    uint32_t rx_src_id;                 /* ����Դ CAN ID (����) */
    uint32_t rx_dst_id;                 /* ����Ŀ�� CAN ID (��Ӧ��) */
    uint8_t* rx_buffer;                 /* ���ջ�����ָ�� */
    uint16_t rx_total_len;              /* ��Ϣ�ܳ��� */
    uint16_t rx_received_len;           /* �ѽ��ճ��� */
    uint8_t rx_expected_seq;            /* ��������һ������֡��� (1-15) */
    uint8_t rx_cf_count_in_block;       /* ��ǰ�����ѽ��յ�����֡�� */
    uint16_t rx_timeout_counter;        /* ���ճ�ʱ������ (ms) */
    
    /* ������� */
    isotp_tx_state_t tx_state;          /* ����״̬ */
    uint32_t tx_dst_id;                 /* ����Ŀ�� CAN ID */
    uint8_t* tx_buffer;                 /* ���ͻ�����ָ�� */
    uint16_t tx_total_len;              /* ������Ϣ�ܳ��� */
    uint16_t tx_sent_len;               /* �ѷ��ͳ��� */
    uint8_t tx_seq;                     /* ��һ������֡��� */
    uint8_t tx_cf_count_in_block;       /* ��ǰ�����ѷ��͵�����֡�� */
    uint16_t tx_timeout_counter;        /* ���ͳ�ʱ������ (ms) */
    uint8_t tx_bs;                      /* �Է�Ҫ��Ŀ��С */
    uint8_t tx_st_min;                  /* �Է�Ҫ�����С���ʱ�� */
    uint16_t tx_st_min_counter;         /* STmin �ӳټ����� */
    
    /* ���ò��� */
    uint8_t local_bs;                   /* ���˿��С (��������ʱʹ��) */
    uint8_t local_st_min;               /* ������С���ʱ�� (��������ʱʹ��) */
    uint16_t timeout_ms;                /* ��ʱʱ�� */
    
    /* �ص����� */
    uint8_t channel;                    /* CAN ͨ���� */
} isotp_connection_t;

/*****************************��������(����)**********************************/

/* ��ʼ�� ISO-TP �� */
void isotp_init(uint8_t channel);

/* 1ms ��ʱ�����º��� (���ⲿ 1ms �ж��е���) */
void isotp_ms_update(void);

/* ���� CAN ֡ (�� CAN ���ջص��е���) */
int8_t isotp_receive_frame(uint8_t channel, uint32_t can_id, uint8_t* frame_data, 
                            uint8_t frame_len, uint8_t* out_data, uint16_t* out_len);

/* ����������Ϣ (�Զ����Ϊ FF/CF) */
int8_t isotp_send_message(uint8_t channel, uint32_t dst_id, uint8_t* data, uint16_t len);

/* ���ʹ������� (��Ҫ����ѭ���е��ã���������״̬��) */
void isotp_tx_process(void);

/* ���ý���״̬ (���ڴ���ָ�) */
void isotp_reset_rx(void);

/* ���÷���״̬ */
void isotp_reset_tx(void);

/* ��ȡ��ǰ����״̬ */
isotp_rx_state_t isotp_get_rx_state(void);

/* ��ȡ��ǰ����״̬ */
isotp_tx_state_t isotp_get_tx_state(void);

/* �������յ�������֡ (�� isotp_receive_frame �е���) */
void isotp_handle_flow_control(uint8_t flow_status, uint8_t block_size, uint8_t st_min);

/* ==================== CAN ID ���˼�¼���Խӿ� ==================== */
#if (ISOTP_ENABLE_FILTER_RECORD == 1)
/* ��ȡ�ܼ�¼���� */
uint32_t isotp_get_filter_record_count(void);

/* ��ȡ����¼�� CAN ID */
uint32_t isotp_get_last_filtered_can_id(void);

/* ��ȡ����¼������ */
void isotp_get_last_filtered_data(uint8_t* out_data, uint8_t* out_len);

/* ��ȡָ�������ļ�¼��0=���£�1=����...��*/
bool isotp_get_filter_record(uint16_t index, uint32_t* can_id, uint8_t* data, uint8_t* len);
#endif

#endif /* ISOTP_TRANSPORT_H_ */
