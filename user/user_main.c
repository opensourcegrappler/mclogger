#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "user_interface.h"
#include "httpclient.h"
#include "easygpio/easygpio.h"

#include "../include/uart.h"
#include "../include/espmissingincludes.h"

volatile int intr_arg = 0;

LOCAL int state = 1;
int configgps = 0;

#define config_gpsPrio        1
#define config_gpsQueueLen    1
os_event_t config_gpsQueue[config_gpsQueueLen];
static void config_gps(os_event_t *events);

//config the gps
static void ICACHE_FLASH_ATTR
config_gps(os_event_t *events)
{
   
    UART_SetPrintPort(1);
  
    if (configgps==0){

        //silence some nmea sentences
        os_printf("$PUBX,40,GLL,0,0,0,0*5C\r\n");
        os_printf("$PUBX,40,VTG,0,0,0,0*5E\r\n");
        os_printf("$PUBX,40,GSV,0,0,0,0*59\r\n");
        os_printf("$PUBX,40,GGA,0,0,0,0*5A\r\n");
        os_printf("$PUBX,40,GSA,0,0,0,0*4E\r\n");
        //is this required?
        os_delay_us(1000000);
        
//        UART_ResetFifo(1);

        //set location fix rate to 5Hz
        uint8_t rate[] = {0xB5,0x62,0x06,0x08,
                          0x06,0x00,0xC8,0x00,
                          0x01,0x00,0x00,0x00,
                          0xDD,0x68};
        
        //set mode to automotive
        uint8_t mode[] = {0xB5,0x62,0x06,0x24,
                          0x24,0x00,0xFF,0xFF,
                          0x04,0x03,0x00,0x00,
                          0x00,0x00,0x10,0x27,
                          0x00,0x00,0x05,0x00,
                          0xFA,0x00,0xFA,0x00,
                          0x64,0x00,0x2C,0x01,
                          0x00,0x00,0x00,0x00,
                          0x10,0x27,0x00,0x00,
                          0x00,0x00,0x00,0x00,
                          0x00,0x00,0x4B,0x97};

        
        int i;
        for (i=0;i<14;i++)
        {
            os_printf("%c",rate[i]);
        }

        int j;
        for (j=0; j<44; j++){
            os_printf("%c",mode[j]);
        }
        
        os_printf("$PUBX,41,1,0002,0002,115200,0*1C\r\n");

        configgps=1;
    }

    UART_SetPrintPort(0);
}

int ICACHE_FLASH_ATTR parse_sentence (char senti[256])
{
    char word[10];
    int wl = 0;
    int wc = 0;
    int state = 0;
    int checksum = 0;
    int chk = 0;

    char *c = senti;
        
    //scan characters from start to end of a sentence
    while (*c)
    {


        if ((*c != ',') && (*c != '\r')) //&& (*c!='*'))
        {
            word[wl] = *c;
            wl++;
        }
        else
        {
            //terminate the word character array
            word[wl++] = '\0';
            //reset the wordlength and increment the wordcounter
            wl = 0;
            wc++;
                
            //check if it's an rmc sentence
            if ((wc==1) &&
                (word[0]=='$') &&
                (word[1]=='G') &&
                (word[2]=='P') &&
                (word[3]=='R') &&
                (word[4]=='M') &&
                (word[5]=='C'))
            {
                state = 1;


            }
            if ((state==1) && (wc <13))
            {
                os_printf("%s,",word);
            }


            if ((state==1) && (wc==13))
            {
                os_printf("%s\n",word);
            }
        }

        //increment to the next character in the sentence
        c++;
    }

}

/*
 * Receives the characters from the serial port.
 */
void ICACHE_FLASH_ATTR uart_rx_task(os_event_t *events) {
    if (events->sig == 0) {
        // Sig 0 is a normal receive. Get how many bytes have been received.
        uint8_t rx_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT;

        // Parse the characters, taking any digits as the new timer interval.
        char rx_char[256];
        
        uint8_t ii;
        for (ii=0; ii < rx_len; ii++) {
            rx_char[ii] = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
        }

        //pointer to the input array
        char *ptr = rx_char;
        int counter = 0;
        char sent[256];
        while (*ptr != '\0')
        {
            sent[counter] = *ptr;
            if (*ptr == '\n')
            {
                sent[counter] = *ptr;
                sent[counter++] = '\0';
                //found the end of a sentence call the sentence parser

                if (intr_arg >0){
                    parse_sentence(sent);
                }
            }
            ptr++;
            counter++;
        }
// Clear the interrupt condition flags and re-enable the receive interrupt.
        WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
        uart_rx_intr_enable(UART0);
    }
}



// interrupt handler
// this function will be executed on any positive edge of GPIO4
LOCAL void  gpio_intr_handler(int * pps_count)
{
  //disable interupt after the sync flashes are finished
    if ((*pps_count>=4))
    {
        ETS_GPIO_INTR_DISABLE();
    }

    //flash gpio 5 for sync flashes
    if((*pps_count > 0) && (*pps_count < 4))
    {
        state = !state;
        GPIO_OUTPUT_SET(5,state);
        (*pps_count)++;
    }
    
// clear gpio status. Say ESP8266EX SDK Programming Guide in  5.1.6. GPIO interrupt handler

    uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);

// if the interrupt was by GPIO0
    if (gpio_status & BIT(4))
    {
// disable interrupt for GPIO0
        gpio_pin_intr_state_set(GPIO_ID_PIN(4), GPIO_PIN_INTR_DISABLE);

// Do something
        uint8 connected = wifi_station_get_connect_status();

        if ((connected == 5) && (*pps_count<1))
        {
            os_printf("Video start %d\n",*(pps_count));
            GPIO_OUTPUT_SET(5,state);

            //build command for starting video recording
            char startvid[] = "";
            os_sprintf(startvid,"http://10.5.5.9/camera/SH?t=%s&p=\%01",SSID_PASSWORD);

            //send command to start recording
            http_get(startvid,"",http_callback_example);
            (*pps_count)=1;
        }

//clear interrupt status for GPIO0
        GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(4));

// Reactivate interrupts for GPIO0
        gpio_pin_intr_state_set(GPIO_ID_PIN(4), GPIO_PIN_INTR_POSEDGE);
    }
}

//Init function 
void ICACHE_FLASH_ATTR
user_init()
{

    // Initialize UARTs, Uart0 logging, Uart1 GPS configure
    uart_init(BIT_RATE_115200, BIT_RATE_9600);
    //point printf to uart1 in prep for gps configure
    UART_SetPrintPort(1);

    //setup network info
    char ssid[32] = SSID;
    char password[64] = SSID_PASSWORD;
    struct station_config stationConf;

    //Set mode station
    wifi_set_opmode(0x1);

    //Set station settings and connect
    os_memcpy(&stationConf.ssid, ssid, 32);
    os_memcpy(&stationConf.password, password, 64);
    wifi_station_set_config(&stationConf);

    //init gpio
    gpio_init();

    //setup pins as required
    GPIO_OUTPUT_SET(4,0); // is this required? setting as output only to then set as input
    //Sync led
    GPIO_OUTPUT_SET(5,0);
    //PPS from GPS input
    easygpio_pinMode(4, EASYGPIO_NOPULL, EASYGPIO_INPUT);
    //start/stop recording button
    easygpio_pinMode(15, EASYGPIO_NOPULL, EASYGPIO_INPUT);
    
    //set up interrupt on PPS signal, point to gpio_intr_handler function
    ETS_GPIO_INTR_DISABLE();
    ETS_GPIO_INTR_ATTACH(gpio_intr_handler, &intr_arg);
    gpio_pin_intr_state_set(GPIO_ID_PIN(4), GPIO_PIN_INTR_POSEDGE);
    ETS_GPIO_INTR_ENABLE();

    //setup and post the config_gps function
    system_os_task(config_gps, config_gpsPrio, config_gpsQueue, config_gpsQueueLen);
    system_os_post(config_gpsPrio,0,0);
 
}
