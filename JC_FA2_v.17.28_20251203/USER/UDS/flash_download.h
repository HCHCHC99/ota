#ifndef FLASH_DOWNLOAD_H
#define FLASH_DOWNLOAD_H

#include <stdint.h>
#include <stdbool.h>

// ============ Éı¼¶×´Ì¬¶¨Òå ============
typedef enum
{
    FW_UPDATE_IDLE = 0,         // ¿ÕÏĞ£¬ÎŞÉı¼¶ÈÎÎñ
    FW_UPDATE_PREPARING,        // ×¼±¸ÖĞ£¨²Á³ıFlash£©
    FW_UPDATE_READY,            // ÒÑ¾ÍĞ÷£¬µÈ´ıÊı¾İ
    FW_UPDATE_TRANSFERRING,     // Êı¾İ´«ÊäÖĞ
    FW_UPDATE_VERIFYING,        // Ğ£ÑéÖĞ
    FW_UPDATE_COMPLETE,         // Éı¼¶Íê³É
    FW_UPDATE_ERROR             // Éı¼¶Ê§°Ü
} FlashDownloadState_t;

// ============ Éı¼¶½á¹û¶¨Òå ============
typedef enum
{
    FW_RESULT_OK = 0,           // ³É¹¦
    FW_RESULT_ADDR_INVALID,     // µØÖ·ÎŞĞ§
    FW_RESULT_SIZE_TOO_LARGE,   // Êı¾İ¹ı´ó
    FW_RESULT_ERASE_FAILED,     // ²Á³ıÊ§°Ü
    FW_RESULT_WRITE_FAILED,     // Ğ´ÈëÊ§°Ü
    FW_RESULT_VERIFY_FAILED,    // Ğ£ÑéÊ§°Ü
    FW_RESULT_SEQUENCE_ERROR,   // ĞòÁĞºÅ´íÎó
    FW_RESULT_BUSY,             // Ã¦
    FW_RESULT_NOT_READY         // Î´¾ÍĞ÷
} FlashDownloadResult_t;

// ============ Éı¼¶½ø¶ÈĞÅÏ¢ ============
typedef struct
{
    uint32_t total_size;        // ×Ü´óĞ¡£¨×Ö½Ú£©
    uint32_t received_size;     // ÒÑ½ÓÊÕ´óĞ¡£¨×Ö½Ú£©
    uint32_t target_address;    // Ä¿±êµØÖ·
    uint8_t progress_percent;   // ½ø¶È°Ù·Ö±È
} FlashDownloadProgress_t;

// ============ Éı¼¶ÅäÖÃ ============
typedef struct
{
    uint32_t max_firmware_size;     // ×î´ó¹Ì¼ş´óĞ¡£¨×Ö½Ú£©
    uint32_t flash_sector_size;     // FlashÉÈÇø´óĞ¡£¨×Ö½Ú£©
    uint32_t user_start_addr;       // ÓÃ»§¿ÉÓÃÆğÊ¼µØÖ·
    uint32_t user_end_addr;         // ÓÃ»§¿ÉÓÃ½áÊøµØÖ·
    uint8_t verify_enabled;         // ÊÇ·ñÆôÓÃĞ£Ñé
    uint8_t auto_reset_on_complete; // Íê³Éºó×Ô¶¯¸´Î»
} FlashDownloadConfig_t;

// ============ DID/RID ¶¨ÒåÒÑÒÆÖÁ uds_did_rid.h ============
// Çë°üº¬ "uds_did_rid.h" ÒÔÊ¹ÓÃ DID_FIRMWARE_VERSION µÈºê

// ============ FlashµØÖ·ÅäÖÃ ============
#define FW_APP_START_ADDR           0x00034000  /* Ó¦ÓÃ³ÌĞòÆğÊ¼µØÖ· */
#define FW_APP_MAX_SIZE             0x00020000  /* ×î´ó128KB */
#define FW_BOOTLOADER_START_ADDR    0x00000000  /* BootloaderÆğÊ¼µØÖ· */

// ============ ·½°¸B£ºRAM»º³åÇøÅäÖÃ ============
#define FW_RAM_BUFFER_SIZE          (60 * 1024)  /* 60KB RAM»º³åÇø£¬ÓÃÓÚÔİ´æ¹Ì¼şÊı¾İ */

/* Flashå†™å…¥å¼€å…³: 0=ä»…æ—¥å¿—ä¸å†™Flash, 1=å®é™…å†™å…¥Flash */
#define FW_FLASH_WRITE_ENABLED      0

// ============ »Øµ÷º¯ÊıÀàĞÍ£¨¹©UDS²ãµ÷ÓÃ£© ============

/**
 * @brief ÇëÇóÏÂÔØ»Øµ÷£¨¶ÔÓ¦ UDS 0x34£©
 * @param address Ä¿±êµØÖ·
 * @param size ¹Ì¼ş×Ü´óĞ¡
 * @return ´¦Àí½á¹û
 */
FlashDownloadResult_t FlashDownload_OnRequestDownload(uint32_t address, uint32_t size);

/**
 * @brief ´«ÊäÊı¾İ»Øµ÷£¨¶ÔÓ¦ UDS 0x36£©
 * @param block_sequence_number ¿éĞòÁĞºÅ£¨1-255£©
 * @param data Êı¾İÖ¸Õë
 * @param len Êı¾İ³¤¶È£¨×Ö½Ú£©
 * @return ´¦Àí½á¹û
 */
FlashDownloadResult_t FlashDownload_OnTransferData(uint8_t block_sequence_number, 
                                                    uint8_t* data, 
                                                    uint8_t len);

/**
 * @brief ÇëÇó´«ÊäÍË³ö»Øµ÷£¨¶ÔÓ¦ UDS 0x37£©
 * @return ´¦Àí½á¹û
 */
FlashDownloadResult_t FlashDownload_OnTransferExit(void);

/**
 * @brief ²Á³ı¹Ì¼şÇø£¨¶ÔÓ¦ UDS 0x31 RID_ERASE_FIRMWARE£©
 * @param address ÆğÊ¼µØÖ·
 * @param size ²Á³ı´óĞ¡
 * @return ´¦Àí½á¹û
 */
FlashDownloadResult_t FlashDownload_Erase(uint32_t address, uint32_t size);

/**
 * @brief ¼ÆËã¹Ì¼şCRC£¨¶ÔÓ¦ UDS 0x31 RID_CALCULATE_CRC£©
 * @param address ÆğÊ¼µØÖ·
 * @param size ¼ÆËã´óĞ¡
 * @param crc_result Êä³öCRC½á¹û
 * @return ´¦Àí½á¹û
 */
FlashDownloadResult_t FlashDownload_CalculateCRC(uint32_t address, uint32_t size, uint32_t* crc_result);

// ============ ×´Ì¬²éÑ¯½Ó¿Ú ============

/**
 * @brief »ñÈ¡µ±Ç°Éı¼¶×´Ì¬
 */
FlashDownloadState_t FlashDownload_GetState(void);

/**
 * @brief »ñÈ¡Éı¼¶½ø¶È
 */
void FlashDownload_GetProgress(FlashDownloadProgress_t* progress);

/**
 * @brief »ñÈ¡×îºóÒ»´Î´íÎó
 */
FlashDownloadResult_t FlashDownload_GetLastError(void);

/**
 * @brief »ñÈ¡¹Ì¼ş°æ±¾ºÅ
 */
uint16_t FlashDownload_GetFirmwareVersion(void);

/**
 * @brief »ñÈ¡Bootloader°æ±¾ºÅ
 */
uint16_t FlashDownload_GetBootloaderVersion(void);

/**
 * @brief »ñÈ¡µ±Ç°¹Ì¼şCRC
 */
uint32_t FlashDownload_GetFirmwareCRC(void);

// ============ ¿ØÖÆ½Ó¿Ú ============

/**
 * @brief ³õÊ¼»¯¹Ì¼şÉı¼¶Ä£¿é
 * @param config ÅäÖÃ²ÎÊı£¨¿ÉÎªNULL£¬Ê¹ÓÃÄ¬ÈÏÖµ£©
 */
void FlashDownload_Init(const FlashDownloadConfig_t* config);

/**
 * @brief È¡Ïûµ±Ç°Éı¼¶ÈÎÎñ
 */
void FlashDownload_Cancel(void);

/**
 * @brief ÖØÖÃÉı¼¶Ä£¿é£¨Çå¿ÕËùÓĞ×´Ì¬£©
 */
void FlashDownload_Reset(void);

// ============ Ö÷Ñ­»·µ÷ÓÃ ============

/**
 * @brief Éı¼¶Ä£¿éÖ÷Ñ­»·´¦Àí£¨´¦ÀíºÄÊ±²Ù×÷£©
 * @note ĞèÒªÔÚÖ÷Ñ­»·ÖĞÖÜÆÚĞÔµ÷ÓÃ
 */
void FlashDownload_Task(void);

/**
 * @brief »ñÈ¡ÊÇ·ñĞèÒªÏìÓ¦µÈ´ı£¨NRC 0x78£©
 * @return true=ĞèÒª·µ»ØÏìÓ¦µÈ´ı
 */
bool FlashDownload_IsPending(void);

#endif /* FLASH_DOWNLOAD_H */
