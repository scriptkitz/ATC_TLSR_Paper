#include <stdint.h>
#include "drivers/8258/compiler.h"

// Ref: https://github.com/canokeys/canokey-core.git

#define SW_NO_ERROR 0x9000
#define SW_TERMINATED 0x6285
#define SW_PIN_RETRIES 0x63C0
#define SW_WRONG_LENGTH 0x6700
#define SW_UNABLE_TO_PROCESS 0x6900
#define SW_SECURITY_STATUS_NOT_SATISFIED 0x6982
#define SW_AUTHENTICATION_BLOCKED 0x6983
#define SW_DATA_INVALID 0x6984
#define SW_CONDITIONS_NOT_SATISFIED 0x6985
#define SW_COMMAND_NOT_ALLOWED 0x6986
#define SW_WRONG_DATA 0x6A80
#define SW_FILE_NOT_FOUND 0x6A82
#define SW_NOT_ENOUGH_SPACE 0x6A84
#define SW_WRONG_P1P2 0x6A86
#define SW_REFERENCE_DATA_NOT_FOUND 0x6A88
#define SW_INS_NOT_SUPPORTED 0x6D00
#define SW_CLA_NOT_SUPPORTED 0x6E00
#define SW_CHECKING_ERROR 0x6F00
#define SW_ERROR_WHILE_RECEIVING 0x6600

// APDU 
#define APDU_CLA                                 (1)
#define APDU_INS                                 (2)
#define APDU_P1                                  (3)    
#define APDU_P2                                  (4)
#define APDU_LC                                  (5)
#define APDU_DATA                                (6)

typedef struct {
  uint8_t *data;
  uint8_t cla;
  uint8_t ins;
  uint8_t p1;
  uint8_t p2;
  uint32_t le; // Le can be 65536 bytes long as per ISO7816-3
  uint16_t lc;
} _attribute_packed_ CAPDU;

typedef struct {
  uint8_t *data;
  uint16_t len;
  uint16_t sw;
} _attribute_packed_ RAPDU;


int fill_capdu(CAPDU* pCAPDU, uint8_t* buff, uint16_t bufflen);