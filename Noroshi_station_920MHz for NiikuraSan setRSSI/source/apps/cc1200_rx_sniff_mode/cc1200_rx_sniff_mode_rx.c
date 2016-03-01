//******************************************************************************
//! @file       noroshi_920mhz_cc1200_rx.c
//! @brief      
//
//*****************************************************************************/


/*******************************************************************************
* INCLUDES
*/
#include "msp430.h"
#include "lcd_dogm128_6.h"
#include "hal_spi_rf_trxeb.h"
#include "cc120x_spi.h"
#include "stdlib.h"
#include "bsp.h"
#include "bsp_key.h"
#include "io_pin_int.h"
#include "bsp_led.h"
#include "cc1200_rx_sniff_mode_reg_config.h"
#include <string.h>
#include "driverlib.h"
#include "uart.h"
#include "uart2.h"


/*******************************************************************************
* DEFINES
*/
#define ISR_ACTION_REQUIRED     1
#define ISR_IDLE                0
// start add 2015.11.11 nishiyama
#define PKTLEN                  30 // 1 < PKTLEN < 126

#define GPIO3                   0x04
#define GPIO2                   0x08
#define GPIO0                   0x80

#define TXLED                   BIT0
#define RXLED                   BIT6
#define TXD                     BIT2
#define RXD                     BIT1
#define RSSI_LOW                -127
#define SAVE_DATA_LIMIT         1
#define LEN_TAG_DATA            12
#define LEN_STATION_DATA        30
#define POS_PARITY              29
#define LIST_SIZE               3000
#define INF                     -99999999
#define UART_DELAY              1000000
// RSSI : Low=102/Middle=84/High=84
#define RSSI_OFFSET             84
#define LIST_SIZE_TAG           10
#define SIZE_GET_BLE            28
#define SIZE_UART_BUFFER        13
#define SIZE_LOG                30
#define SIZE_LOG_LIST           300

// Error Code
#define CODE_HEARTBEAT          1
#define CODE_RECEIVE_SIZE       2
#define CODE_DID_NUMBER         3
#define CODE_PARITY_CHECK       4
#define CODE_TRANSMIT_DATA      5

// end   add 2015.11.11 nishiyama


/*
 * revision:
 * 2015/11/20 01/01 First
 * 2015/11/25 01/02 Add UART To GW
 * 2015/12/16 01/03 Add UART From BLE(NORDIC)
 */
typedef unsigned char byte;
typedef unsigned int word;

static uint16 major = 1;                // major number
static uint16 minor = 1;                // minor number

static uint32 uiMyStID = 1;             // My Station ID Default(0x01)
static uint32 uiToStID = 0;             // To Station ID Master=0 Slave=1->
static uint8 uiRTtype = 0;              // 0=920, 1=BLE
static uint8 uiLog = 0xFF;              // ID:Log
unsigned long long timerCount_1000 = 0; // UNIX Time Stamp (UTC)
unsigned long long timerCount_50 = 0;   // not use


/*******************************************************************************
* LOCAL VARIABLES
*/
static uint8  packetSemaphore;
static uint8  packetSemaphoreTX;
static uint32 packetCounter = 0;

// start add 2015.11.11 nishiyama
static byte save_list[LIST_SIZE] = {0};
static word save_list_start = 0;
static word save_list_end = 0;
//static byte save_list_tag[LIST_SIZE_TAG] = {0};
static byte ble_list[SIZE_GET_BLE] = {0};
static word ble_list_start = 0;
static word ble_list_end = 0;

static byte log_list[SIZE_LOG_LIST] = {0};
static word log_list_start = 0;
static word log_list_end = 0;

static uint8 rxBuffer[LEN_STATION_DATA] = {0};
static uint8 rxBytes;
static uint8 txBuf[LEN_STATION_DATA] = {0};
static uint8 txBytes;
static uint16 ui16TXCounter = 0;
unsigned long long uartStartCounter = 0;
static uint8 tx_counter = 0;
static uint8 txBuffer[PKTLEN + 2] = {0};
unsigned char uartRxBuf2[LEN_STATION_DATA] = {0};
static uint16 transmit_size = 0;
static uint16 error_log_size = 0;
// Initialize packet buffer of size PKTLEN + 1
static uint32 rx_counter = 0;

static byte uartRxBufA0[SIZE_UART_BUFFER] = {0};
static byte uartRxBufA1[SIZE_UART_BUFFER] = {0};
static byte uartRxBufA2[SIZE_UART_BUFFER] = {0};
static uint16 uartRxBufA0_cnt_start = 0;
static uint16 uartRxBufA0_cnt_end = 0;
static uint16 uartRxBufA1_cnt_start = 0;
static uint16 uartRxBufA1_cnt_end = 0;
static uint16 uartRxBufA2_cnt_start = 0;
static uint16 uartRxBufA2_cnt_end = 0;

// for Ring Buffer
typedef char ELEM;        // 
ELEM ringbuff[LEN_STATION_DATA]; // 
int front;                // 
int rear;                 // 

// UART Port Configuration parameters and registers
UARTConfig cnf;
USCIUARTRegs uartUsciRegs;
USARTUARTRegs uartUsartRegs;

/*******************************************************************************
* STATIC FUNCTIONS
*/
// System
static void initMCU(void);
static void registerConfig(void);

// 920MHz
static void radioRxISR(void);
static void calibrateRCOsc(void);
static void initRX(void);
static void initTX(void);
static void runRX(void);
static void runTX(void);
static void finTX(void);
static void radioTxISR(void);
static int8 getRSSI(void);
static void updateLcd(void);

// Original Function
static void createPacket(uint8 randBuffer[]);
static uint8 judgeParity(uint8 *, uint8);
static uint8 MakeParity(uint8 *);
static uint8 MakeTransmitData(uint8 *, uint8);
static void MakeSlaveDataFromTag(uint8 *);
static void MakeSlaveDataFromSlave(uint8 *);
static void MakeMasterDataFromSlave(uint8 *);
static void saveTransmitData(uint8 *);
static uint8 judgeToID(uint8 *, uint8);
static uint8 chkRXdataAndMakeTXdata(uint8 *, uint8);
static void calcTransmitSize(void);
static void makeLog(uint8, uint8 *);

// UART
static void initUART(void);
static void init_uart(void);
//static void uart_transmit(void);
static void uart_transmit(uint8_t *, uint16);
static void sendUart(uint8_t *, uint16);

// i2c
static void init_i2c(void);
static void write_i2c(unsigned char, unsigned char);

// Timer
static void initTimer(void);


/*******************************************************************************
*   @fn         main
*
*   @brief      Runs the main routine
*
*   @param      none
*
*   @return     none
*/
void main(void) {

    // Initialize MCU and peripherals
    initMCU();

    // Write radio registers
    registerConfig();

    // Enter runRX, never coming back
    runRX();
}


/*******************************************************************************
*   @fn         runRX
*
*   @brief      Puts radio in RX Sniff Mode and waits for packets. A packet
*               counter is incremented for each packet received and the LCD is
*               updated
*
*   @param      none
*
*   @return     none
*/
static void runRX(void) {

//    uint8 rxBuffer[32] = {0};
    uint8 rxBytes;
    uint8 rxBytes2;
    uint8 marcState;
    unsigned long long tc_backup = 0;
    
    int cnt = 0;

    // Connect ISR function to GPIO2
    ioPinIntRegister(IO_PIN_PORT_1, GPIO2, &radioRxISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, GPIO2, IO_PIN_RISING_EDGE);

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO2);

    // Enable interrupt
    ioPinIntEnable(IO_PIN_PORT_1, GPIO2);

    // Update LCD
    updateLcd();

    // Calibrate radio
    trxSpiCmdStrobe(CC120X_SCAL);

    // Wait for calibration to be done (radio back in IDLE state)
    do {
        cc120xSpiReadReg(CC120X_MARCSTATE, &marcState, 1);
    } while (marcState != 0x41);

    // Calibrate the RCOSC
    calibrateRCOsc();
    
    // UART config
    initUART();
    
    initTimer();

    // Infinite loop
    while(TRUE) {

        // Set radio in RX Sniff Mode
        trxSpiCmdStrobe(CC120X_SWOR);

        // Wait for packet to be received
        while(packetSemaphore != ISR_ACTION_REQUIRED);
    
        // Clear semaphore flag
        packetSemaphore = ISR_IDLE;

        // Read number of bytes in RX FIFO
        rxBytes = 0;
        tc_backup = timerCount_50;
        do {
            cc120xSpiReadReg(CC120X_NUM_RXBYTES, &rxBytes, 1);
            __delay_cycles(8000);
            cc120xSpiReadReg(CC120X_NUM_RXBYTES, &rxBytes2, 1);
//        } while ((rxBytes<16) && ((timerCount_50 - tc_backup) < 6));
        } while (rxBytes!=rxBytes2);
        
        rxBytes = rxBytes2;
        if ( rxBytes > 2 ){
            rxBytes -= 2;
        }

        // Read all the bytes in the RX FIFO
        memset( rxBuffer, 0, sizeof( rxBuffer ) );
        tc_backup = timerCount_1000;
        do {
            cc120xSpiReadRxFifo( rxBuffer, rxBytes );
        } while ((rxBuffer[0]==0) && ((timerCount_50 - tc_backup) < 6));

        // RSSI setting
        rxBuffer[0] = getRSSI();

        // ASCII convert
        uart_transmit(rxBuffer, rxBytes);
        
        // Update LCD
        updateLcd();
        
    }
}


/*******************************************************************************
*   @fn         getRSSI
*
*   @brief      get RSSI value from register
*
*   @param      none
*
*   @return     none
*/
int8 getRSSI() 
{
  /* Radio API that the PER test uses */
  int8 rssi = 0;
  uint8 rssi2compl,rssiValid;
  int16 rssiConverted;
  
  cc120xSpiReadReg(CC120X_RSSI0, &rssiValid, 1);
  if(rssiValid & 0x01)
  {
    /* Read RSSI from MSB register */
    cc120xSpiReadReg(CC120X_RSSI1, &rssi2compl,1);
    rssiConverted = (int16)((int8)rssi2compl) - RSSI_OFFSET;
    return rssiConverted;
  }
  /* keep last value since new value is not valid */
  return (rssiValid);
}


/*******************************************************************************
*   @fn         calibrateRcOsc
*
*   @brief      Calibrates the RC oscillator used for the eWOR timer. When this
*               function is called, WOR_CFG0.RC_PD must be 0
*
*   @param      none
*
*   @return     none
*/
static void calibrateRCOsc(void) {

    uint8 temp;

    // Read current register value
    cc120xSpiReadReg(CC120X_WOR_CFG0, &temp,1);

    // Mask register bit fields and write new values
    temp = (temp & 0xF9) | (0x02 << 1);

    // Write new register value
    cc120xSpiWriteReg(CC120X_WOR_CFG0, &temp,1);

    // Strobe IDLE to calibrate the RCOSC
    trxSpiCmdStrobe(CC120X_SIDLE);

    // Disable RC calibration
    temp = (temp & 0xF9) | (0x00 << 1);
    cc120xSpiWriteReg(CC120X_WOR_CFG0, &temp, 1);
}


/*******************************************************************************
*   @fn         radioRxISR
*
*   @brief      ISR for packet handling in RX. Sets packet semaphore
*               and clears interrupt flag
*
*   @param      none
*
*   @return     none
*/
static void radioRxISR(void) {

    // Set packet semaphore
    packetSemaphore = ISR_ACTION_REQUIRED;

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO2);
}


/*******************************************************************************
*   @fn         initMCU
*
*   @brief      Initialize MCU and board peripherals
*
*   @param      none
*
*   @return     none
*/
static void initMCU(void) {

    // Init clocks and I/O
    bspInit(BSP_SYS_CLK_8MHZ);

    // Init LEDs
    bspLedInit();

    // Init buttons
    bspKeyInit(BSP_KEY_MODE_POLL);

    // Initialize SPI interface to LCD (shared with SPI flash)
    bspIoSpiInit(BSP_FLASH_LCD_SPI, BSP_FLASH_LCD_SPI_SPD);

    // Init LCD
    lcdInit();

    // Instantiate transceiver RF SPI interface to SCLK ~ 4 MHz
    // Input parameter is clockDivider
    // SCLK frequency = SMCLK/clockDivider
    trxRfSpiInterfaceInit(2);

    // Enable global interrupt
    _BIS_SR(GIE);
}


/*******************************************************************************
*   @fn         registerConfig
*
*   @brief      Write register settings as given by SmartRF Studio found in
*               cc1200_rx_sniff_mode_reg_config.h
*
*   @param      none
*
*   @return     none
*/
static void registerConfig(void) {

    uint8 writeByte;

    // Reset radio
    trxSpiCmdStrobe(CC120X_SRES);

    // Write registers to radio
    for(uint16 i = 0;
        i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
        writeByte = preferredSettings[i].data;
        cc120xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
    }
}


/*******************************************************************************
*   @fn         updateLcd
*
*   @brief      Updates LCD buffer and sends buffer to LCD module
*
*   @param      none
*
*   @return     none
*/
static void updateLcd(void) {

    // Update LDC buffer and send to screen
    lcdBufferClear(0);
    lcdBufferPrintString(0, "RX Sniff Mode", 0, eLcdPage0);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, 7);
    lcdBufferPrintString(0, "Received OK:", 0, eLcdPage3);
    lcdBufferPrintInt(0, packetCounter++, 70, eLcdPage4);
    lcdBufferPrintString(0, "RX", 0, eLcdPage7);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, 55);
    lcdBufferInvertPage(0, 0, LCD_COLS, eLcdPage7);
    lcdSendBuffer(0);
}


/*******************************************************************************
*   @fn         initTimer
*
*   @brief      timer 1sec. cycle setting
*
*   @param      none
*
*   @return     none
*/
void initTimer(void)
{
  // 32767 -> 1sec
  // 3277  -> 0.1sec
  // 1638  -> 0.05sec

  TA0CCTL0 &= ~CCIE;                    // Disable timer Interrupt  
  TA0CCR0 = 32767;                      // PWM Period
  TA0CCTL1 = OUTMOD_3;                  // TACCR1 set/reset
  TA0CCR1 = 1;                          // TACCR1 PWM Duty Cycle   
  TA0CTL = TASSEL_1 + MC_1;             // ACLK, up mode
  TA0CCTL0 = CCIE;                      // TA0CCR0 interrupt enabled

  TB0CCTL0 &= ~CCIE;                    // Disable timer Interrupt
  TB0CCR0 = 1638;                       // PWM Period
  TB0CCTL1 = OUTMOD_3;                  // TACCR1 set/reset
  TB0CCR1 = 1;                          // TACCR1 PWM Duty Cycle   
  TB0CTL = TASSEL_1 + MC_1;             // ACLK, up mode
  TB0CCTL0 = CCIE;                      // TA1CCR0 interrupt enabled
}


/*******************************************************************************
*   @fn         Initialize UART port
*
*   @brief      UART to Gateway Initialize
*
*   @param      none
*
*   @return     none
*/
void initUART(void)
{
  /********************************
   *
   * UART Specific Configuration
   *
   ********************************/

    // Buffers to be used by UART Driver
    unsigned char uartTxBuf[2000];

    initUartDriver();

    // Configure UART Module on USCIA1
    cnf.moduleName = USCI_A1;

    // Use UART Pins P5.7 and P5.6
    cnf.portNum = PORT_5;
    cnf.RxPinNum = PIN7;
    cnf.TxPinNum = PIN6;

    // 115200 Baud from 8MHz SMCLK
    cnf.clkRate = 8000000L;
    cnf.baudRate = 115200L;
    cnf.clkSrc = UART_CLK_SRC_SMCLK;

    // 8N1
    cnf.databits = 8;
    cnf.parity = UART_PARITY_NONE;
    cnf.stopbits = 1;

    int res = configUSCIUart(&cnf,&uartUsciRegs);
    if(res != UART_SUCCESS)
    {
            // Failed to initialize UART for some reason
            __no_operation();
    }
    setUartTxBuffer(&cnf, uartTxBuf, sizeof(uartTxBuf));
    __enable_interrupt(); // Enable Global Interrupts
    
    // Send the string hello using interrupt driven
//    uartSendDataInt(&cnf,(unsigned char *)"Hello\r\n", strlen("Hello\r\n"));
//    __delay_cycles(100000);
}


/*******************************************************************************
*   @fn         uart_transmit
*
*   @brief      Transmit data (UART)
*
*   @param      none
*
*   @return     none
*/
static void uart_transmit(uint8* pData, uint16 len) 
{
  // ASCII convert
  char dat[256] = {0};
  char ch[] = "0123456789ABCDEF";
  char c[62] = {0};
  int16 j = 0;
//  c[60] = '\r';
//  c[61] = '\n';
  
  memcpy( dat, pData, len );
  for ( j=0; j<len; j++ )
  {
    c[j*2] = ch[(rxBuffer[j]>>4)&0x0f];
    c[j*2+1] = ch[rxBuffer[j]&0x0f];
  }
  c[len*2  ] = '\r';
  c[len*2+1] = '\n';
  
  uartSendDataInt( &cnf, c, len*2+2 );
}


/*******************************************************************************
*   @fn         sendUart
*
*   @brief      UART Send
*
*   @param      none
*
*   @return     none
*/
void sendUart(uint8* pData, uint16 len)
{
  uartSendDataInt( &cnf, pData, len );
}


/*******************************************************************************
*   @fn         Timer A0
*
*   @brief      1sec timer / cycle
*
*   @param      none
*
*   @return     none
*/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{
  timerCount_1000++;
}


/*******************************************************************************
*   @fn         Timer B0
*
*   @brief      33msec timer / cycle
*
*   @param      none
*
*   @return     none
*/
#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer_B0(void)
{
    timerCount_50++;
}
