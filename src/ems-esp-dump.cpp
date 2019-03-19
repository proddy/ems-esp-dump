/*
 * Simple test to read EMS bus. Connect Rx to D7 and leave Tx free.
 */
#include <Arduino.h>
#include <user_interface.h>

void ems_parseTelegram(uint8_t * telegram, uint8_t length);
#define EMSUART_UART 0      // UART 0
#define EMSUART_CONFIG 0x1c // 8N1 (8 bits, no stop bits, 1 parity)
#define EMSUART_BAUD 9600   // uart baud rate for the EMS circuit

#define EMS_ID_ME 0x0B // Fixed - our device, hardcoded as the "Service Key"

#define EMS_MAXBUFFERS 4      // 4 buffers for circular filling to avoid collisions
#define EMS_MAXBUFFERSIZE 128 // max size of the buffer. packets are max 32 bytes

#define EMSUART_recvTaskPrio 1
#define EMSUART_recvTaskQueueLen 64

typedef struct {
    uint8_t writePtr;
    uint8_t buffer[EMS_MAXBUFFERSIZE];
} _EMSRxBuf;

_EMSRxBuf * pEMSRxBuf;
_EMSRxBuf * paEMSRxBuf[EMS_MAXBUFFERS];
uint8_t     emsRxBufIdx = 0;

os_event_t recvTaskQueue[EMSUART_recvTaskQueueLen]; // our Rx queue

const uint8_t ems_crc_table[] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22,
                                 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E, 0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E, 0x40, 0x42, 0x44, 0x46,
                                 0x48, 0x4A, 0x4C, 0x4E, 0x50, 0x52, 0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60, 0x62, 0x64, 0x66, 0x68, 0x6A,
                                 0x6C, 0x6E, 0x70, 0x72, 0x74, 0x76, 0x78, 0x7A, 0x7C, 0x7E, 0x80, 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0x8E,
                                 0x90, 0x92, 0x94, 0x96, 0x98, 0x9A, 0x9C, 0x9E, 0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC, 0xAE, 0xB0, 0xB2,
                                 0xB4, 0xB6, 0xB8, 0xBA, 0xBC, 0xBE, 0xC0, 0xC2, 0xC4, 0xC6, 0xC8, 0xCA, 0xCC, 0xCE, 0xD0, 0xD2, 0xD4, 0xD6,
                                 0xD8, 0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8, 0xEA, 0xEC, 0xEE, 0xF0, 0xF2, 0xF4, 0xF6, 0xF8, 0xFA,
                                 0xFC, 0xFE, 0x19, 0x1B, 0x1D, 0x1F, 0x11, 0x13, 0x15, 0x17, 0x09, 0x0B, 0x0D, 0x0F, 0x01, 0x03, 0x05, 0x07,
                                 0x39, 0x3B, 0x3D, 0x3F, 0x31, 0x33, 0x35, 0x37, 0x29, 0x2B, 0x2D, 0x2F, 0x21, 0x23, 0x25, 0x27, 0x59, 0x5B,
                                 0x5D, 0x5F, 0x51, 0x53, 0x55, 0x57, 0x49, 0x4B, 0x4D, 0x4F, 0x41, 0x43, 0x45, 0x47, 0x79, 0x7B, 0x7D, 0x7F,
                                 0x71, 0x73, 0x75, 0x77, 0x69, 0x6B, 0x6D, 0x6F, 0x61, 0x63, 0x65, 0x67, 0x99, 0x9B, 0x9D, 0x9F, 0x91, 0x93,
                                 0x95, 0x97, 0x89, 0x8B, 0x8D, 0x8F, 0x81, 0x83, 0x85, 0x87, 0xB9, 0xBB, 0xBD, 0xBF, 0xB1, 0xB3, 0xB5, 0xB7,
                                 0xA9, 0xAB, 0xAD, 0xAF, 0xA1, 0xA3, 0xA5, 0xA7, 0xD9, 0xDB, 0xDD, 0xDF, 0xD1, 0xD3, 0xD5, 0xD7, 0xC9, 0xCB,
                                 0xCD, 0xCF, 0xC1, 0xC3, 0xC5, 0xC7, 0xF9, 0xFB, 0xFD, 0xFF, 0xF1, 0xF3, 0xF5, 0xF7, 0xE9, 0xEB, 0xED, 0xEF,
                                 0xE1, 0xE3, 0xE5, 0xE7};

//
// Main interrupt handler
// Important: do not use ICACHE_FLASH_ATTR !
//
static void emsuart_rx_intr_handler(void * para) {
    static uint16_t length;
    static uint8_t  uart_buffer[EMS_MAXBUFFERSIZE];
    static bool     emsRxBusy;

    // is a new buffer? if so init the thing for a new telegram
    if (!emsRxBusy) {
        emsRxBusy = true; // status set to busy
        length    = 0;
    }

    // fill IRQ buffer, by emptying Rx FIFO
    if (U0IS & ((1 << UIFF) | (1 << UITO) | (1 << UIBD))) {
        while ((USS(EMSUART_UART) >> USRXC) & 0xFF) {
            uart_buffer[length++] = USF(EMSUART_UART);
        }

        // clear Rx FIFO full and Rx FIFO timeout interrupts
        U0IC = (1 << UIFF);
        U0IC = (1 << UITO);
    }

    // BREAK detection = End of EMS data block
    if (USIS(EMSUART_UART) & ((1 << UIBD))) {
        ETS_UART_INTR_DISABLE(); // disable all interrupts and clear them

        U0IC = (1 << UIBD); // INT clear the BREAK detect interrupt

        // copy data into transfer buffer
        pEMSRxBuf->writePtr = length;
        os_memcpy((void *)pEMSRxBuf->buffer, (void *)&uart_buffer, length);

        // set the status flag stating BRK has been received and we can start a new package
        emsRxBusy = false;

        // call emsuart_recvTask() at next opportunity
        system_os_post(EMSUART_recvTaskPrio, 0, 0);

        // re-enable UART interrupts
        ETS_UART_INTR_ENABLE();
    }
}

/*
 * system task triggered on BRK interrupt
 * Read commands are all asynchronous
 * When a buffer is full it is sent to the ems_parseTelegram() function in ems.cpp. This is the hook
 */
static void ICACHE_FLASH_ATTR emsuart_recvTask(os_event_t * events) {
    // get next free EMS Receive buffer
    _EMSRxBuf * pCurrent = pEMSRxBuf;
    pEMSRxBuf            = paEMSRxBuf[++emsRxBufIdx % EMS_MAXBUFFERS];

    // transmit EMS buffer, excluding the BRK
    if (pCurrent->writePtr > 1) {
        ems_parseTelegram((uint8_t *)pCurrent->buffer, (pCurrent->writePtr) - 1);
    }
}

/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR emsuart_init() {
    ETS_UART_INTR_DISABLE();
    ETS_UART_INTR_ATTACH(NULL, NULL);

    // allocate and preset EMS Receive buffers
    for (int i = 0; i < EMS_MAXBUFFERS; i++) {
        _EMSRxBuf * p = (_EMSRxBuf *)malloc(sizeof(_EMSRxBuf));
        paEMSRxBuf[i] = p;
    }
    pEMSRxBuf = paEMSRxBuf[0]; // preset EMS Rx Buffer

    // pin settings
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0RXD);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);

    // set 9600, 8 bits, no parity check, 1 stop bit
    USD(EMSUART_UART)  = (ESP8266_CLOCK / EMSUART_BAUD);
    USC0(EMSUART_UART) = EMSUART_CONFIG; // 8N1

    // flush everything left over in buffer, this clears both rx and tx FIFOs
    uint32_t tmp = ((1 << UCRXRST) | (1 << UCTXRST)); // bit mask
    USC0(EMSUART_UART) |= (tmp);                      // set bits
    USC0(EMSUART_UART) &= ~(tmp);                     // clear bits

    // conf1 params
    // UCTOE = RX TimeOut enable (default is 1)
    // UCTOT = RX TimeOut Threshold (7bit) = want this when no more data after 2 characters. (default is 2)
    // UCFFT = RX FIFO Full Threshold (7 bit) = want this to be 31 for 32 bytes of buffer. (default was 127).
    USC1(EMSUART_UART) = 0;                                              // reset config first
    USC1(EMSUART_UART) = (31 << UCFFT) | (0x02 << UCTOT) | (1 << UCTOE); // enable interupts

    // set interrupts for triggers
    USIC(EMSUART_UART) = 0xffff; // clear all interupts
    USIE(EMSUART_UART) = 0;      // disable all interrupts

    // enable rx break, fifo full and timeout.
    // not frame error UIFR (because they are too frequent) or overflow UIOF because our buffer is only max 32 bytes
    USIE(EMSUART_UART) = (1 << UIBD) | (1 << UIFF) | (1 << UITO);

    // set up interrupt callbacks for Rx
    system_os_task(emsuart_recvTask, EMSUART_recvTaskPrio, recvTaskQueue, EMSUART_recvTaskQueueLen);

    // disable esp debug which will go to Tx and mess up the line
    // system_set_os_print(0); // https://github.com/espruino/Espruino/issues/655

    ETS_UART_INTR_ATTACH(emsuart_rx_intr_handler, NULL);
    ETS_UART_INTR_ENABLE();

    // swap Rx and Tx pins to use GPIO13 (D7) and GPIO15 (D8) respectively
    system_uart_swap();
}

// calculate CRC
uint8_t _crcCalculator(uint8_t * data, uint8_t len) {
    uint8_t crc = 0;

    // read data and stop before the CRC
    for (uint8_t i = 0; i < len - 1; i++) {
        crc = ems_crc_table[crc];
        crc ^= data[i];
    }

    return crc;
}

// like itoa but for hex, and quick
char * _hextoa(uint8_t value, char * buffer) {
    char * p    = buffer;
    byte   nib1 = (value >> 4) & 0x0F;
    byte   nib2 = (value >> 0) & 0x0F;
    *p++        = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    *p++        = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
    *p          = '\0'; // null terminate just in case
    return buffer;
}

// entry point
void ems_parseTelegram(uint8_t * telegram, uint8_t length) {
    if (length == 1) {
        Serial.printf("Poll %02X\n", telegram[0]);
        return;
    }

    char raw[300]   = {0};
    char buffer[16] = {0};
    for (int i = 0; i < length; i++) {
        strlcat(raw, _hextoa(telegram[i], buffer), sizeof(raw));
        strlcat(raw, " ", sizeof(raw)); // add space
    }

    if (telegram[length - 1] != _crcCalculator(telegram, length)) {
        strlcat(raw, "(BAD)", sizeof(raw));
    } else {
        strlcat(raw, "(OK)", sizeof(raw));
    }

    Serial.println(raw);
}

void setup() {
    emsuart_init();
    Serial.begin(115200);
    Serial.println("Ready...");
}


void loop() {
    // optional delay
    // delay(1);
}
