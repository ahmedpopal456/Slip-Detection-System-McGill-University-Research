//
/* Author: Ahmed Popal */
//
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_esb.h"
#include "nrf.h"
#include <time.h>
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "sensor_h.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include <string.h>
#include "nrf_gpio.h"
#include "sensor_h.h"
#include "nrf_drv_twi.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

FRESULT ff_result;

#define SAMPLES_IN_BUFFER 7
float conversion_factor = 206.868687; // =

//static volatile bool SD_Card = true;
//static volatile bool lIsFirstTime = true;
static volatile bool m_tx_done = false; ///< Indicates if writing operation from accelerometer has ended.
static volatile bool m_rx_done = false; ///< Indicates if reading operation from accelerometer has ended.
static const nrf_drv_twi_t m_twi_device = NRF_DRV_TWI_INSTANCE(1);/* TWI instance. */
//static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];
#define NRF_SCL_PIN     17  ///< SCL Line for I2C.
#define NRF_SDA_PIN     16  ///< SDA Line for I2C.
#define SDC_SCK_PIN     12  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    13  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    11  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      14  ///< SDC chip select (CS) pin.
#define RESET_MEMORY_TEST_BYTE  (0x0DUL)        /**< Known sequence written to a special register to check if this wake up is from System OFF. */
#define RAM_RETENTION_OFF       (0x00000003UL)  /**< The flag used to turn off RAM retention on nRF52. */
#define BTN_PRESSED     0                       /**< Value of a pressed button. */
#define BTN_RELEASED    1                       /**< Value of a released button. */

#define FILE_NAME   "data.csv"
volatile uint8_t state = 1;
volatile uint8_t saadc_use_buffer = 0;
static volatile bool SD_Card = true;
static uint8_t  RTC1_skip_increment_counter;
static uint8_t  m_fromI2Cdevice[20];
static int16_t  gyroVals[3];

static int16_t  accelVals[3*200];
static int      accelCounter = 0;

static int16_t  accelVals1[3*200];
static int      accelCounter1 = 0;

static int16_t adc_result1[SAMPLES_IN_BUFFER*200];
static int adcCounter1 = 0;
//
static int16_t  accelVals2[3*200];
static int      accelCounter2 = 0;

static int16_t adc_result2[SAMPLES_IN_BUFFER*200];
static int adcCounter2 = 0;
//
static uint32_t timeinms[200];
static int timeCounter = 0;

static uint32_t RTC1_Milliseconds;
//static int16_t magVals[3];

static volatile bool lFillBuffer1 = false;
static volatile bool lFillBuffer2 = false;

static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00);
static nrf_esb_payload_t rx_payload;

/////////////////////////////////////////////////// SD CARD /////////////////////////////////////////////

NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);



static void write_to_sd(bool lIsFirstTime)
{

    static FIL file;
    uint32_t bytes_written;

    NRF_LOG_INFO("Writing to file " FILE_NAME "...\r\n");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);

    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".\r\n");
        return;
    }

    if(!lIsFirstTime)
    {

    	for(int i=0; i < 200; i++)
    	{

    	 char pValuesToWrite[100];
    	 memset(pValuesToWrite, ' ', 99);
    	 pValuesToWrite[99]='\0';

    	 sprintf(pValuesToWrite, "%ld%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%s", //%ld%c...%d%c%d%c%d%c%d%c%d%c%d%c%d
    			(long int)timeinms[i],',',
    		    (int)accelVals1[i*3],',',(int)accelVals1[i*3 + 1],',',(int)accelVals1[i*3 + 2],',',
				(int)adc_result1[i*7],',',
				(int)adc_result1[i*7 + 1],',',
				(int)adc_result1[i*7 + 2],',',
				(int)adc_result1[i*7 + 3],',',
				(int)adc_result1[i*7 + 4],',',
				(int)adc_result1[i*7 + 5],',',
				(int)adc_result1[i*7 + 6],',',
    		    (int)accelVals2[i*3],',',(int)accelVals2[i*3 + 1],',',(int)accelVals2[i*3 + 2],',',
				(int)adc_result2[i*7],',',
				(int)adc_result2[i*7 + 1],',',
				(int)adc_result2[i*7 + 2],',',
				(int)adc_result2[i*7 + 3],',',
				(int)adc_result2[i*7 + 4],',',
				(int)adc_result2[i*7 + 5],',',
				(int)adc_result2[i*7 + 6],',',
    		    (int)accelVals[i*3],',',(int)accelVals[i*3 + 1],',',(int)accelVals[i*3 + 2],
				"\n");

        ff_result = f_write(&file, pValuesToWrite , sizeof(pValuesToWrite) - 1, (UINT *) &bytes_written);
    	}

		if (ff_result != FR_OK)
		{
			NRF_LOG_INFO("Write failed\r\n.");
		}
		else
		{
			NRF_LOG_INFO("%d bytes written.\r\n", bytes_written);
		}

    }
    else if(lIsFirstTime)
    {
       	char buffer1[430] = "Counter (ms), LH - Accel: X,LH - Accel: Y,LH - Accel: Z,LH - ADC Channel 1,LH - ADC Channel 2,LH - ADC Channel 3,LH - ADC Channel 4,LH - ADC Channel 5,LH - ADC Channel 6,LH - ADC Channel 7, RH - Accel: X,RH - Accel: Y,RH - Accel: Z,RH - ADC Channel 1,RH - ADC Channel 2,RH - ADC Channel 3,RH - ADC Channel 4,RH - ADC Channel 5,RH - ADC Channel 6,RH - ADC Channel 7, Pelvis - Accel: X, Pelvis - Accel: Y, Pelvis - Accel: Z  \n";

       	ff_result = f_write(&file, buffer1, sizeof(buffer1) - 1, (UINT *) &bytes_written);
       	lIsFirstTime = false;
		free(buffer1);

		if (ff_result != FR_OK)
		{
			NRF_LOG_INFO("Write failed\r\n.");
		}
		else
		{
			NRF_LOG_INFO("%d bytes written.\r\n", bytes_written);
		}
    }

    (void) f_close(&file);
    return;
}

void configure_sd()
{

    static FATFS fs;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...\r\n");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.\r\n");
        SD_Card = false;
        return;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB\r\n", capacity);

    NRF_LOG_INFO("Mounting volume...\r\n");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.\r\n");
        SD_Card = false;
        return;
    }
}

//////////////////////////////////////////// RTC ///////////////////////////////////


void rtc_config(void)
{

    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos);    //  RC oscillator
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;                                         //  Clear it so we can sense when it changes
    NRF_CLOCK->TASKS_LFCLKSTART = 1;                                            //  Start RC oscillator
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0){}                               //  Wait for oscillator to start

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;                                         //  Clean up
//
//  Config RTC1 to generate an interrupt every .0009765625 seconds
//
    NRF_RTC1->TASKS_STOP = 1;                                                   //  Must be stopped to be able to write PRESCALER

    RTC1_Milliseconds = 0;
    RTC1_skip_increment_counter = 0;

    NRF_RTC1->PRESCALER = 31;                                                   //  RTC1 clock is 32768 Hz, divided by 32 == 1024Hz
    //
    // Enable TICK event and TICK interrupt:
    //
    NRF_RTC1->EVTENSET = (RTC_EVTENSET_TICK_Enabled << RTC_EVTENSET_TICK_Pos);
    NRF_RTC1->INTENSET = (RTC_INTENSET_TICK_Enabled << RTC_INTENSET_TICK_Pos);

    NVIC_EnableIRQ(RTC1_IRQn);    // Enable Interrupt for RTC1 in the core

    NRF_RTC1->TASKS_START = 1;
}


void RTC1_IRQHandler(void)
{
    if(NRF_RTC1->EVENTS_TICK != 0)
    {                  								    //  make sure it is a tick event, every 976.5625 us
        NRF_RTC1->EVENTS_TICK = 0;                      //  clear the event

        RTC1_skip_increment_counter++;                  //  special handling of every 42nd event

        if(RTC1_skip_increment_counter < 42)
        {
           ++RTC1_Milliseconds;                         //  check if we are at a multiple of 1000 milliseconds
        }
        else
        {                                               //  special case for every 42nd tick. No increment of millisecond counter
            RTC1_skip_increment_counter = 0;            //  reset event counter. No need to check for incrementing seconds, as 1000
        }                                               //   is not divisible by 42
    }
}

/////////////////////////////////////// I2C /////////////////////////////////////


//ACCEL *********************************************************************

void writeAccel(uint8_t* valToWrite, uint8_t numBytesToWrite)
{
    uint32_t err_code;
    m_tx_done = false;

    err_code = nrf_drv_twi_tx(&m_twi_device, LSM303_ADDR, (uint8_t*)valToWrite, numBytesToWrite, true);
    APP_ERROR_CHECK(err_code);

    while(!m_tx_done);
}

uint8_t* readAccel(uint8_t addrToRead,uint8_t numBytesToRead){

    uint32_t err_code;
    writeAccel(&addrToRead, 1);

    m_rx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi_device, LSM303_ADDR, (uint8_t*)&m_fromI2Cdevice, numBytesToRead);
    APP_ERROR_CHECK(err_code);

    while(!m_rx_done);

    return m_fromI2Cdevice;
}

uint8_t readAccelReg(uint8_t addrToRead){

    uint32_t err_code;
    m_fromI2Cdevice[0] = 0x00;
    writeAccel(&addrToRead, 1);
    m_rx_done = false;
    err_code = nrf_drv_twi_rx(&m_twi_device, LSM303_ADDR, (uint8_t*)&m_fromI2Cdevice, 1);
    APP_ERROR_CHECK(err_code);

    while(!m_rx_done);

    return m_fromI2Cdevice[0];
}

void enableAccel(){

	//
	// CTRL_REG4_A: BDU (0:cont. update, 1: output registers not updated until read), BLE (0: data LSB, 1: data MSB), FS(1:0), HR(1: high-resolution, 0: low res.)
    //
	uint8_t setCtrlReg[2] = {CTRL_REG4_A ,0x38}; // 0011 1000
    writeAccel(setCtrlReg,2);
//    nrf_delay_ms(10);
    //
    // CTRL_REG1_A: Data Rate Selection (3:0), Low-Power Mode Enable (0: normal, 1: low), Zen, Yen, Xen (Z,Y and X enable)
    //
    setCtrlReg[0] = CTRL_REG1_A;
    setCtrlReg[1] = 0x77;        // 0111 0111
    writeAccel(setCtrlReg,2);
//    nrf_delay_ms(10);

    // Magnetometer
    //  CRA_REG_M: TEMP_EN : Temperature Enable:1, DO[2:0]:111 (220 Hz)
    //
    setCtrlReg[0] = CRA_REG_M;
    setCtrlReg[1] = 0x00; // 0001 1100
    writeAccel(setCtrlReg,2);
//    nrf_delay_ms(10);

    //
    // CRB_REG_M: GN(2:0): 111: +- 8 Gauss,
    //
    setCtrlReg[0] = CRB_REG_M;
    setCtrlReg[1] = 0x00;   //    1110 0000
    writeAccel(setCtrlReg,2);
//    nrf_delay_ms(10);

    //
    // MR_REG_M: MD(1:0) -> 00 = continuous conversion mode
    //
    setCtrlReg[0] = MR_REG_M;
    setCtrlReg[1] = 0x00;
    writeAccel(setCtrlReg,2);
//    nrf_delay_ms(10);
}

uint8_t* readAccelXYZ(){

    uint8_t msbassert = (OUT_X_L_A | (1 << 7));
	uint8_t* results = readAccel(msbassert,6);


	if(accelCounter < 200)
	{
		accelVals[accelCounter*3] = ((int16_t)(results[1] << 8) + (int16_t)results[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
		accelVals[accelCounter*3 + 1] = ((int16_t)(results[3] << 8) + (int16_t)results[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
		accelVals[accelCounter*3 + 2] = ((int16_t)(results[5] << 8) + (int16_t)results[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

		accelCounter++;
	}
	else
	{
		accelCounter = 0;

		accelVals[accelCounter*3] = ((int16_t)(results[1] << 8) + (int16_t)results[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
		accelVals[accelCounter*3 + 1] = ((int16_t)(results[3] << 8) + (int16_t)results[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
		accelVals[accelCounter*3 + 2] = ((int16_t)(results[5] << 8) + (int16_t)results[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

		accelCounter++;
	}

	return results;
}

void setClickCnfg (bool enable){

    if(enable){

        //SET INT1
        uint8_t setInteruptPin[2] = {CTRL_REG3_A , 0x80}; //0b11000000
        writeAccel(setInteruptPin,sizeof(setInteruptPin));

        //SET HIGH PASS FILTER
         uint8_t setHPF[2] = {CTRL_REG2_A ,0x00};
         writeAccel(setHPF,sizeof(setHPF));

        //SET XYZ as listening for single click
        uint8_t setXYZListen[2] = {CLICK_CFG_A,0x15}; //0001 0101
        writeAccel(setXYZListen,sizeof(setXYZListen));

        //SET THRESHOLD
        uint8_t setThresh[2] = {CLICK_THS_A,0x00};
        writeAccel(setThresh,sizeof(setThresh));

        //SET INT1
        uint8_t setTimeLimit[2] = {TIME_LIMIT_A,0x3F};
        writeAccel(setTimeLimit,sizeof(setTimeLimit));

    }
    else{

        //SET INT1
        uint8_t setInteruptPinOff[2] = {CTRL_REG3_A,0b10000000};
        writeAccel(setInteruptPinOff,sizeof(setInteruptPinOff));

        //SET XYZ as listening for single click
        uint8_t setXYZListenOff[2] = {CLICK_CFG_A,0b00000000};
        writeAccel(setXYZListenOff,sizeof(setXYZListenOff));

        //SET HIGH PASS FILTER
        uint8_t setHPFOff[2] = {CTRL_REG2_A,0b00000000};
        writeAccel(setHPFOff,sizeof(setHPFOff));

        //SET THRESHOLD
        uint8_t setThreshOff[2] = {CLICK_THS_A,0b00000000};
        writeAccel(setThreshOff,sizeof(setThreshOff));

        uint8_t setEnable[2] = { CLICK_SRC_A ,0b00000000};
        writeAccel(setEnable,sizeof(setEnable));
    }
}

bool checkIntrpt()
{
    uint8_t* interruptSource = readAccel(CLICK_SRC_A,1);
    if((interruptSource[0] >> 7) & 1) return true;
    return false;
}

//GYRO **********************************************************************

void writeGyro(uint8_t* valToWrite, uint8_t numBytesToWrite){
    uint32_t err_code;
    m_tx_done = false;
    err_code = nrf_drv_twi_tx(&m_twi_device, D20_SA0_HIGH_ADDRESS, (uint8_t*)valToWrite, numBytesToWrite, true);
    APP_ERROR_CHECK(err_code);
    while(!m_tx_done);
}

uint8_t* readGyro(uint8_t addrToRead,uint8_t numBytesToRead){
    uint32_t err_code;
    writeGyro(&addrToRead, 1);
    m_rx_done = false;
    err_code = nrf_drv_twi_rx(&m_twi_device, D20_SA0_HIGH_ADDRESS, (uint8_t*)&m_fromI2Cdevice, numBytesToRead);
    APP_ERROR_CHECK(err_code);
    while(!m_rx_done);
    return m_fromI2Cdevice;
}

uint8_t readGyroReg(uint8_t addrToRead){

    uint32_t err_code;
    m_fromI2Cdevice[0] = 0x00;
    writeGyro(&addrToRead, 1);
    m_rx_done = false;
    err_code = nrf_drv_twi_rx(&m_twi_device, D20_SA0_HIGH_ADDRESS, (uint8_t*)&m_fromI2Cdevice, 1);
    APP_ERROR_CHECK(err_code);
    while(!m_rx_done);
    return m_fromI2Cdevice[0];
}

void enableGyro(){

    // 0x00 = 0b00000000
    // Low_ODR = 0 (low speed ODR disabled)
    uint8_t setCtrlReg[2] = {LOW_ODR,0x00};
    writeGyro(setCtrlReg, 2);
    nrf_delay_ms(10);

    // 0x6F = 0b01101111
    // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
    setCtrlReg[0] = CTRL1G;
    setCtrlReg[1] = 0x6F;
    writeGyro(setCtrlReg,2);//CTRL1G, 0x6F);
    nrf_delay_ms(10);

    // 0x00 = 0b00000000
    // FS = 00 (+/- 250 dps full scale)
    setCtrlReg[0] = CTRL4G;
    setCtrlReg[1] = 0x00;
    writeGyro(setCtrlReg,2);//CTRL4G, 0x00);
    nrf_delay_ms(10);
}

int16_t* readGyroXYZ(){

    uint8_t msbassert = 0 ;//(OUT_X_L_G | (1 << 7));

    uint8_t* results = readGyro(msbassert,6);

    uint8_t xla = results[0];
    uint8_t xha = results[1];

    uint8_t yla = results[2];
    uint8_t yha = results[3];

    uint8_t zla = results[4];
    uint8_t zha = results[5];

    int16_t gx = (int16_t)(xha << 8 | xla) >> 4;
    int16_t gy = (int16_t)(yha << 8 | yla) >> 4;
    int16_t gz = (int16_t)(zha << 8 | zla) >> 4;

    gyroVals[0] = gx;
    gyroVals[1] = gy;
    gyroVals[2] = gz;

    return gyroVals;
}


//TWI ************************************************************************
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context){

    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if ((p_event->type == NRF_DRV_TWI_EVT_DONE) &&
                (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX))
            {
                if(m_tx_done != true)
                {
                    m_tx_done  = true;
                    return;
                }
            }
            else  if ((p_event->type == NRF_DRV_TWI_EVT_DONE) &&
                (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX))
            {
                if(m_rx_done != true)
                {
                    m_rx_done  = true;
                    return;
                }
            }
            break;
        default:
            break;
    }
}

void twi_init (){

    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_steval_config = {
       .scl                = NRF_SCL_PIN,
       .sda                = NRF_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    err_code = nrf_drv_twi_init(&m_twi_device, &twi_steval_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi_device);
    nrf_delay_ms(10);

}

/////////////////////////////////////////  RF  /////////////////////////////////////////////

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            (void) nrf_esb_flush_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            // Get the most recent element from the RX FIFO.
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS) ;

            if(lFillBuffer1 == true)
            {
            	if(accelCounter1 < 200){
				accelVals1[accelCounter1*3] = ((int16_t)(rx_payload.data[1] << 8) + (int16_t)rx_payload.data[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
				accelVals1[accelCounter1*3 + 1] = ((int16_t)(rx_payload.data[3] << 8) + (int16_t)rx_payload.data[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
				accelVals1[accelCounter1*3 + 2] = ((int16_t)(rx_payload.data[5] << 8) + (int16_t)rx_payload.data[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

				accelCounter1++;
            	}
            	else
            	{
            		accelCounter1 = 0;

    				accelVals1[accelCounter1*3] = ((int16_t)(rx_payload.data[1] << 8) + (int16_t)rx_payload.data[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
    				accelVals1[accelCounter1*3 + 1] = ((int16_t)(rx_payload.data[3] << 8) + (int16_t)rx_payload.data[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
    				accelVals1[accelCounter1*3 + 2] = ((int16_t)(rx_payload.data[5] << 8) + (int16_t)rx_payload.data[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

    				accelCounter1++;
            	}

            	if(adcCounter1 < 200)
            	{

            		adc_result1[adcCounter1*7]     = ((int16_t)(rx_payload.data[6]  << 8) + (int16_t)rx_payload.data[13]);
            		adc_result1[adcCounter1*7 + 1] = ((int16_t)(rx_payload.data[7]  << 8) + (int16_t)rx_payload.data[14]);
            		adc_result1[adcCounter1*7 + 2] = ((int16_t)(rx_payload.data[8]  << 8) + (int16_t)rx_payload.data[15]);
            		adc_result1[adcCounter1*7 + 3] = ((int16_t)(rx_payload.data[9]  << 8) + (int16_t)rx_payload.data[16]);
            		adc_result1[adcCounter1*7 + 4] = ((int16_t)(rx_payload.data[10] << 8) + (int16_t)rx_payload.data[17]);
            		adc_result1[adcCounter1*7 + 5] = ((int16_t)(rx_payload.data[11] << 8) + (int16_t)rx_payload.data[18]);
            		adc_result1[adcCounter1*7 + 6] = ((int16_t)(rx_payload.data[12] << 8) + (int16_t)rx_payload.data[19]);

				adcCounter1++;
            	}
            	else
            	{
            		adcCounter1 = 0;

    				accelVals1[accelCounter1*3] = ((int16_t)(rx_payload.data[1] << 8) + (int16_t)rx_payload.data[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
    				accelVals1[accelCounter1*3 + 1] = ((int16_t)(rx_payload.data[3] << 8) + (int16_t)rx_payload.data[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
    				accelVals1[accelCounter1*3 + 2] = ((int16_t)(rx_payload.data[5] << 8) + (int16_t)rx_payload.data[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

    				adcCounter1++;
            	}


        		lFillBuffer1 = false;
            }

            if(lFillBuffer2 == true)
            {
            	if(accelCounter2 < 200){
				accelVals2[accelCounter2*3] = ((int16_t)(rx_payload.data[1] << 8) + (int16_t)rx_payload.data[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
				accelVals2[accelCounter2*3 + 1] = ((int16_t)(rx_payload.data[3] << 8) + (int16_t)rx_payload.data[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
				accelVals2[accelCounter2*3 + 2] = ((int16_t)(rx_payload.data[5] << 8) + (int16_t)rx_payload.data[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

				accelCounter2++;
            	}
            	else
            	{
            		accelCounter2 = 0;

    				accelVals2[accelCounter2*3] = ((int16_t)(rx_payload.data[1] << 8) + (int16_t)rx_payload.data[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
    				accelVals2[accelCounter2*3 + 1] = ((int16_t)(rx_payload.data[3] << 8) + (int16_t)rx_payload.data[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
    				accelVals2[accelCounter2*3 + 2] = ((int16_t)(rx_payload.data[5] << 8) + (int16_t)rx_payload.data[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

    				accelCounter2++;
            	}

				if(timeCounter < 200)
				{
					timeinms[timeCounter] = RTC1_Milliseconds;
					timeCounter++;
				}
				else
				{
					timeCounter = 0;
					timeinms[timeCounter] = RTC1_Milliseconds;
					timeCounter++;
				}

            	if(adcCounter2 < 200)
            	{

            		adc_result2[adcCounter2*7]     = ((int16_t)(rx_payload.data[6]  << 8) + (int16_t)rx_payload.data[13]);
            		adc_result2[adcCounter2*7 + 1] = ((int16_t)(rx_payload.data[7]  << 8) + (int16_t)rx_payload.data[14]);
            		adc_result2[adcCounter2*7 + 2] = ((int16_t)(rx_payload.data[8]  << 8) + (int16_t)rx_payload.data[15]);
            		adc_result2[adcCounter2*7 + 3] = ((int16_t)(rx_payload.data[9]  << 8) + (int16_t)rx_payload.data[16]);
            		adc_result2[adcCounter2*7 + 4] = ((int16_t)(rx_payload.data[10] << 8) + (int16_t)rx_payload.data[17]);
            		adc_result2[adcCounter2*7 + 5] = ((int16_t)(rx_payload.data[11] << 8) + (int16_t)rx_payload.data[18]);
            		adc_result2[adcCounter2*7 + 6] = ((int16_t)(rx_payload.data[12] << 8) + (int16_t)rx_payload.data[19]);

				adcCounter2++;
            	}
            	else
            	{
            		adcCounter2 = 0;

    				accelVals2[accelCounter2*3] = ((int16_t)(rx_payload.data[1] << 8) + (int16_t)rx_payload.data[0]) >> 4; //	uint8_t xla = results[0], uint8_t xha = results[1];
    				accelVals2[accelCounter2*3 + 1] = ((int16_t)(rx_payload.data[3] << 8) + (int16_t)rx_payload.data[2]) >> 4; //	uint8_t yla = results[2], uint8_t yha = results[3];
    				accelVals2[accelCounter2*3 + 2] = ((int16_t)(rx_payload.data[5] << 8) + (int16_t)rx_payload.data[4]) >> 4; //	uint8_t zla = results[4], uint8_t zha = results[5];

    				adcCounter2++;
            	}


        		lFillBuffer2 = false;
            }
            break;
    }
}


void clocks_start( void )
{
    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

#ifndef NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;

#else // NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_LEGACY_CONFIG;

#endif // NRF_ESB_LEGACY
    nrf_esb_config.retransmit_count         = 6;
    nrf_esb_config.selective_auto_ack       = false;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.tx_output_power          = NRF_ESB_TX_POWER_4DBM;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    tx_payload.length  = NRF_ESB_MAX_PAYLOAD_LENGTH;
    tx_payload.pipe    = 0;
    tx_payload.data[0] = 0x00;

    return NRF_SUCCESS;
}


uint32_t send_start_codon(bool lboolean)
{
    uint32_t err_code;
    uint32_t channel;

        tx_payload.data[0] = 0xFF;

        tx_payload.noack = false;
        err_code = nrf_esb_write_payload(&tx_payload);
        VERIFY_SUCCESS(err_code);

        if(lboolean)
        {
        nrf_delay_ms(2.5);
	    channel = 0x00000064;
	    err_code = nrf_esb_set_rf_channel(channel);
	    VERIFY_SUCCESS(err_code);
        }

    return NRF_SUCCESS;
}

uint32_t send_gather_data(bool lboolean)
{
    uint32_t err_code;
    uint32_t channel;

        tx_payload.data[0] = 0xEE;

        tx_payload.noack = false;
        err_code = nrf_esb_write_payload(&tx_payload);
        VERIFY_SUCCESS(err_code);

        if(lboolean)
        {
        nrf_delay_ms(2.5);
	    channel = 0x00000000;
	    err_code = nrf_esb_set_rf_channel(channel);
	    VERIFY_SUCCESS(err_code);
        }

    return NRF_SUCCESS;
}
///////////////////////////////////////// MAIN /////////////////////////////////////////////////


int main(void)
 {
 	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    uint32_t err_code;

    // Configure RF
    clocks_start();
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

	// Configure I2C
	twi_init();

	// Configure and enable STEVAL Sensor
	setClickCnfg(true);
	enableAccel();


	// Configure SD Card
	configure_sd();

	if(SD_Card)
	write_to_sd(true);

	// Use RTC1 to time SAADC sampling
	rtc_config();

    uint32_t channel;									/////////// SEND START CODON FOR BOTH HEELS (CHANGE CHANNEL
    channel = 0x00000000;                              ///////////  AFTER FIRST SEND)
    err_code = nrf_esb_set_rf_channel(channel);
    VERIFY_SUCCESS(err_code);

	while (1)
	{
		send_start_codon(true);  // Send Start Codon For LEFT HEEL
		send_start_codon(false); // Send Start Codon For RIGHT HEEL

                ////////////////////////////////// GATHER PELVIS DIGITAL DATA
		readAccelXYZ();
		//////////////////////////////////

		nrf_delay_ms(2.5);       /////////// SEND GATHER DATA PULSE TO BOTH HEELS (2.5ms apart
		lFillBuffer1 = true;
		send_gather_data(true);  // Gather the Data FROM RIGHT HEEL

		lFillBuffer2 = true;
		send_gather_data(false);  // Gather the Data FROM LEFT HEEL
		nrf_delay_ms(2.5);


		if(SD_Card && ((accelCounter1 == 199 && adcCounter1 == 199) && (accelCounter2 == 199 && adcCounter2 == 199) && (accelCounter == 199) && (timeCounter == 199)))  // STORE WHEN BOTH LEFT AND RIGHT HEEL HAVE GATHERED DATA
		{
		  write_to_sd(false);
		}

	}
}

