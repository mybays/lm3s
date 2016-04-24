#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "drivers/rit128x96x4.h"
#include "drivers/mydelay.h"

#include "nRF24L01.h"
#include <inttypes.h>
#include "uartstdio.h"

#define NUM_SSI_DATA 3

#define GPIO_PA2_SSI0CLK        0x00000801
#define GPIO_PA3_SSI0FSS        0x00000C01
#define GPIO_PA4_SSI0RX         0x00001001
#define GPIO_PA5_SSI0TX         0x00001401

#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

extern bool p_variant;

static const char rf24_datarate_e_str_0[] = "1MBPS";
static const char rf24_datarate_e_str_1[] = "2MBPS";
static const char rf24_datarate_e_str_2[] = "250KBPS";
static const char * const rf24_datarate_e_str_P[]  = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};


static const char rf24_crclength_e_str_0[]  = "Disabled";
static const char rf24_crclength_e_str_1[]  = "8 bits";
static const char rf24_crclength_e_str_2[]  = "16 bits" ;
static const char * const rf24_crclength_e_str_P[]  = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};

static const char rf24_pa_dbm_e_str_0[]  = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[]  = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[]  = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[]  = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[]  = {
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};

static const char rf24_model_e_str_0[]  = "nRF24L01";
static const char rf24_model_e_str_1[]  = "nRF24L01+";
static const char * const rf24_model_e_str_P[]  = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};


#define pgm_read_word(p) (*(p))


int value=0;

const uint64_t addresses[2] = { 0xABABCDCD71LL, 0x544d52687CLL };

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UART0IntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);

    //
    // Initialize the OLED display and write status.
    //
    RIT128x96x4Init(1000000);
    RIT128x96x4StringDraw("UART Echo",            36,  0, 15);
    RIT128x96x4StringDraw("Port:   Uart 0",       12, 16, 15);
    RIT128x96x4StringDraw("Baud:   115,200 bps",  12, 24, 15);
    RIT128x96x4StringDraw("Data:   8 Bit",        12, 32, 15);
    RIT128x96x4StringDraw("Parity: None",         12, 40, 15);
    RIT128x96x4StringDraw("Stop:   1 Bit",        12, 48, 15);

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //PC5,PC7 EN,CSN
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,1<<5|1<<7);

    //SPI配置
    unsigned long ulDataTx[NUM_SSI_DATA];
    unsigned long ulDataRx[NUM_SSI_DATA];
    unsigned long ulindex;
    unsigned long ultemp=0;


    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 4000000, 8);
    /*
    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT); 
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_4MA,
                     GPIO_PIN_TYPE_STD_WPU);  
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0 , GPIO_STRENGTH_4MA,
                     GPIO_PIN_TYPE_STD_WPU);
    */
    SSIEnable(SSI0_BASE);


    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Prompt for text to be entered.
    //
    UARTStdioInit(0);
    UARTSend((unsigned char *)"Enter text:\n\r", 12);
    UARTSend((unsigned char *)"Enter text:\n\r", 12);

    //清零接收缓冲区
    while(SSIDataGetNonBlocking(SSI0_BASE, &ulDataRx[0]))
    {
    }
    ulDataTx[0] = 's';
    ulDataTx[1] = 'p';
    ulDataTx[2] = 'i';

    set_nrf24l01_csn_l();
    /*
    for(ulindex = 0; ulindex < NUM_SSI_DATA; ulindex++)
    {
    	UARTprintf("'%c' ", ulDataTx[ulindex]);
    	SSIDataPut(SSI0_BASE, ulDataTx[ulindex]);
    }
    */
    set_nrf24l01_csn_h();
    _delay_ms(1);

    if( setDataRate( RF24_250KBPS ) )
    {
        p_variant = true ;
    }

    //初始化NRF24L01
    set_module_tx();
    nrf_write_reg(NRF_CONFIG,0x0a);
    print_byte_register("CONFIG\t",NRF_CONFIG,1);

    init_NRF24L01();
    set_module_tx();

    unsigned char transfer_value[]="EEWORLD_MSP430_00";

    //set_module_tx();

    //读不出来spi数据的原因是,原来里面有没读取完的数据,需要先清理,再读写.



    setChannel(74);
    UARTprintf("getchannel:%d\r\n",getChannel());
 //   setChannel(24);
 //   UARTprintf("getchannel:%d\r\n",getChannel());

    //写地址
    nrf_write_buf(TX_ADDR,(uint8_t*)&addresses[0],5);

    uint8_t recvbuf[5];
    nrf_read_buf(TX_ADDR,&recvbuf[0],5);

    for(int i=0;i<5;i++)
    {
        UARTprintf("%d:%d ",i,recvbuf[i]);
    }
    UARTprintf("\r\n");
    //end of test write address


    uint8_t data[32];

    for(int i=0;i<32;i++)
    {
        data[i]=i;
    }


    

    UARTprintf("\r\n");
    //print_status(get_status());
    //while(SSIDataGetNonBlocking(SSI0_BASE, &ulDataRx[0]))
    //{
    //}

    //重新发送前,避免写缓冲区满
    flush_tx();
    spi_write_reg(STATUS, ( spi_read_reg(STATUS) ) | _BV(MAX_RT) );
    //role=role_ping_out
    openWritingPipe(addresses[1]);
    openReadingPipe(1,addresses[0]);

    unsigned char test;
    //while(1)
    {
        test=spi_read_reg(0x05);
        UARTprintf("test:%d\r\n",test);
        _delay_ms(1000);
    }

    //调试关闭
    //nrf_write_reg(EN_AA,0x00);
    nrf_write_reg(EN_RXADDR,0x03);
    //nrf_write_reg(SETUP_RETR,0x00);
    nrf_write_reg(RX_PW_P1,0x20);


    print_status(get_status());
    print_address_register("RX_ADDR_P0-1",RX_ADDR_P0,2);
    print_byte_register("RX_ADDR_P2-5",RX_ADDR_P2,4);

    print_address_register("TX_ADDR\t",TX_ADDR,1);

    print_byte_register("RX_PW_P0-6",RX_PW_P0,6);


    print_byte_register("EN_AA\t",EN_AA,1);
    print_byte_register("EN_RXADDR",EN_RXADDR,1);
    print_byte_register("RF_CH\t",RF_CH,1);
    print_byte_register("RF_SETUP",RF_SETUP,1);
    set_module_tx();
    nrf_write_reg(NRF_CONFIG,0x0a);
    print_byte_register("CONFIG\t",NRF_CONFIG,1);
    print_byte_register("DYNPD/FEATURE",DYNPD,2);


    UARTprintf("Data Rate\t = %s\r\n", pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
    UARTprintf("Model\t\t = %s\r\n",   pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
    UARTprintf("CRC Length\t = %s\r\n",pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
    UARTprintf("PA Power\t = %s\r\n",  pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));




    //将业务数据写入:WR_TX_PLOAD
    while(1)
    {
        nrf_tx_packet(data);
        //_delay_ms(50);
        /*
        set_nrf24l01_ce_l();
        nrf_write_buf(WR_TX_PLOAD,data,TX_PLOAD_WIDTH);
        set_nrf24l01_ce_h();
        _delay_ms(30);
        */
    }
    


    


    while(1)
    {
        spi_write_reg(0x26,0x07);
        test=spi_read_reg(0x06);
        //UARTprintf("test:%d\r\n",test);
        _delay_ms(100);
    }
    while(1)
    {
    	transfer_value[16]=value&0x00ff;
    	transfer_value[15]=((value&0xff00)>>8);
    	//transfer_string(transfer_value, 17);
    	value++;
    	_delay_ms(20);
    }

    while(1)
    {
    	for(ulindex=0;ulindex<sizeof(transfer_value);ulindex++)
    	{
    		SSIDataPut(SSI0_BASE, transfer_value[ulindex]);
    		UARTprintf("-%x|", transfer_value[ulindex]);
    		_delay_ms(1);
    	}
    }
    while(1)
    {
    	ultemp++;
    	SSIDataPut(SSI0_BASE, ultemp);
    	UARTprintf("-%x|", ultemp);
    	
    	if(ultemp == 255)
    	{
    		ultemp=0;
    	}
    	
    	_delay_ms(1);
    }
    /*
    while(SSIBusy(SSI0_BASE))
    {
    }

    for(ulindex = 0; ulindex < NUM_SSI_DATA; ulindex++)
    {
    	SSIDataGet(SSI0_BASE, &ulDataRx[ulindex]);
    	ulDataRx[ulindex] &= 0x00FF;
    	UARTprintf("'%c' ", ulDataRx[ulindex]);
    }
    */



    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
    }
}
