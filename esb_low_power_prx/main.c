/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.

 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */



// CODE IS MEANT FOR LEFT HEEL

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


//#define NRF_ESB_LEGACY


static nrf_esb_payload_t tx_payload;
static nrf_esb_payload_t rx_payload;


#define SAMPLES_IN_BUFFER 7
float conversion_factor = 206.868687;

//static volatile bool SD_Card = true;
//static volatile bool lIsFirstTime = true;
static volatile bool m_tx_done = false; ///< Indicates if writing operation from accelerometer has ended.
static volatile bool m_rx_done = false; ///< Indicates if reading operation from accelerometer has ended.
static const nrf_drv_twi_t m_twi_device = NRF_DRV_TWI_INSTANCE(1);/* TWI instance. */
static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];

#define NRF_SCL_PIN 14                ///< SCL Line for I2C.
#define NRF_SDA_PIN 11					///< SDA Line for I2C.

#define RESET_MEMORY_TEST_BYTE  (0x0DUL)        /**< Known sequence written to a special register to check if this wake up is from System OFF. */
#define RAM_RETENTION_OFF       (0x00000003UL)  /**< The flag used to turn off RAM retention on nRF52. */
#define BTN_PRESSED     0                       /**< Value of a pressed button. */
#define BTN_RELEASED    1                       /**< Value of a released button. */

volatile uint8_t state = 1;
volatile uint8_t saadc_use_buffer = 0;

//static uint8_t  RTC1_skip_increment_counter;
static uint8_t  m_fromI2Cdevice[20];
static int16_t  gyroVals[3];

static volatile bool lStartCodon = false;

//static int16_t  accelVals[3*500];
//static int      accelCounter = 0;

//static int16_t adc_result[SAMPLES_IN_BUFFER*500];
//static int adcCounter = 0;
//
//static uint32_t timeinms[500];
//static int timeCounter = 0;

//static uint32_t RTC1_Milliseconds;
//static int16_t magVals[3];

static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00);
static nrf_esb_payload_t rx_payload;

/////////////////////////////////////////////////////// SAADC CODE ///////////////////////////////////////////////


void SAADC_IRQHandler(void)
{

	while(NRF_SAADC->EVENTS_RESULTDONE == 0){};

    // Clear events
    NRF_SAADC->EVENTS_END = 0;

    // Alternate between buffers so that we can read and print one buffer while filling the other at the same tim
//    if (saadc_use_buffer == 0)
//    {
//        // Change to second buffer
//        NRF_SAADC->RESULT.PTR  =(uint32_t)&m_buffer_pool[1][0];
//        saadc_use_buffer = 1;
//    }
//    else
//    {
//        // Change to first buffer
//        NRF_SAADC->RESULT.PTR  =(uint32_t)&m_buffer_pool[0][0];
//        saadc_use_buffer = 0;
//    }
}


void saadc_init(void)
{
    NVIC_EnableIRQ(SAADC_IRQn);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;

    // Configure the SAADC channel with AIN0 (P0.02) as positive input, no negative input(single ended).
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0 << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    NRF_SAADC->CH[0].CONFIG = (( SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos ) | // Burst mode is disabled (normal operation)
                              ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos  ) | // Single ended, PSELN will be ignored, negative input to ADC shorted to GND
                              ( SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos  ) | // Acquisition time, the time the ADC uses to sample the input voltage
                              ( SAADC_CH_CONFIG_REFSEL_VDD1_4  << SAADC_CH_CONFIG_REFSEL_Pos ) | // VDD/4 as reference
                              ( SAADC_CH_CONFIG_GAIN_Gain1_6   << SAADC_CH_CONFIG_GAIN_Pos   ) | // Gain control
                              ( SAADC_CH_CONFIG_RESN_Bypass    << SAADC_CH_CONFIG_RESN_Pos   ) | // Bypass resistor ladder
                              ( SAADC_CH_CONFIG_RESP_Bypass    << SAADC_CH_CONFIG_RESP_Pos   )); // Bypass resistor ladder

    // Configure the SAADC channel with AIN1 (P0.0X) as positive input, no negative input(single ended).
    NRF_SAADC->CH[1].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput1 << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[1].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    NRF_SAADC->CH[1].CONFIG = (( SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos ) | // Burst mode is disabled (normal operation)
                              ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos  ) | // Single ended, PSELN will be ignored, negative input to ADC shorted to GND
                              ( SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos  ) | // Acquisition time, the time the ADC uses to sample the input voltage
                              ( SAADC_CH_CONFIG_REFSEL_VDD1_4  << SAADC_CH_CONFIG_REFSEL_Pos ) | // VDD/4 as reference
                              ( SAADC_CH_CONFIG_GAIN_Gain1_6   << SAADC_CH_CONFIG_GAIN_Pos   ) | // Gain control
                              ( SAADC_CH_CONFIG_RESN_Bypass    << SAADC_CH_CONFIG_RESN_Pos   ) | // Bypass resistor ladder
                              ( SAADC_CH_CONFIG_RESP_Bypass    << SAADC_CH_CONFIG_RESP_Pos   )); // Bypass resistor ladder

    // Configure the SAADC channel with AIN2 (P0.0X) as positive input, no negative input(single ended).
    NRF_SAADC->CH[2].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput3 << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[2].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    NRF_SAADC->CH[2].CONFIG = (( SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos ) | // Burst mode is disabled (normal operation)
                              ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos  ) | // Single ended, PSELN will be ignored, negative input to ADC shorted to GND
                              ( SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos  ) | // Acquisition time, the time the ADC uses to sample the input voltage
                              ( SAADC_CH_CONFIG_REFSEL_VDD1_4  << SAADC_CH_CONFIG_REFSEL_Pos ) | // VDD/4 as reference
                              ( SAADC_CH_CONFIG_GAIN_Gain1_6   << SAADC_CH_CONFIG_GAIN_Pos   ) | // Gain control
                              ( SAADC_CH_CONFIG_RESN_Bypass    << SAADC_CH_CONFIG_RESN_Pos   ) | // Bypass resistor ladder
                              ( SAADC_CH_CONFIG_RESP_Bypass    << SAADC_CH_CONFIG_RESP_Pos   )); // Bypass resistor ladder

    // Configure the SAADC channel with AIN3 (P0.0X) as positive input, no negative input(single ended).
    NRF_SAADC->CH[3].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput4 << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[3].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    NRF_SAADC->CH[3].CONFIG = (( SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos ) | // Burst mode is disabled (normal operation)
                              ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos  ) | // Single ended, PSELN will be ignored, negative input to ADC shorted to GND
                              ( SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos  ) | // Acquisition time, the time the ADC uses to sample the input voltage
                              ( SAADC_CH_CONFIG_REFSEL_VDD1_4  << SAADC_CH_CONFIG_REFSEL_Pos ) | // VDD/4 as reference
                              ( SAADC_CH_CONFIG_GAIN_Gain1_6   << SAADC_CH_CONFIG_GAIN_Pos   ) | // Gain control
                              ( SAADC_CH_CONFIG_RESN_Bypass    << SAADC_CH_CONFIG_RESN_Pos   ) | // Bypass resistor ladder
                              ( SAADC_CH_CONFIG_RESP_Bypass    << SAADC_CH_CONFIG_RESP_Pos   )); // Bypass resistor ladder

    // Configure the SAADC channel with AIN4 (P0.0X) as positive input, no negative input(single ended).
    NRF_SAADC->CH[4].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput5 << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[4].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    NRF_SAADC->CH[4].CONFIG = (( SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos ) | // Burst mode is disabled (normal operation)
                              ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos  ) | // Single ended, PSELN will be ignored, negative input to ADC shorted to GND
                              ( SAADC_CH_CONFIG_TACQ_10us        << SAADC_CH_CONFIG_TACQ_Pos  ) | // Acquisition time, the time the ADC uses to sample the input voltage
                              ( SAADC_CH_CONFIG_REFSEL_VDD1_4   << SAADC_CH_CONFIG_REFSEL_Pos ) | // VDD/4 as reference
                              ( SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos   ) | // Gain control
                              ( SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos   ) | // Bypass resistor ladder
                              ( SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos   )); // Bypass resistor ladder

    // Configure the SAADC channel with AIN5 (P0.0X) as positive input, no negative input(single ended).
    NRF_SAADC->CH[5].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput6 << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[5].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    NRF_SAADC->CH[5].CONFIG = (( SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos ) | // Burst mode is disabled (normal operation)
                              ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos  ) | // Single ended, PSELN will be ignored, negative input to ADC shorted to GND
                              ( SAADC_CH_CONFIG_TACQ_10us        << SAADC_CH_CONFIG_TACQ_Pos  ) | // Acquisition time, the time the ADC uses to sample the input voltage
                              ( SAADC_CH_CONFIG_REFSEL_VDD1_4   << SAADC_CH_CONFIG_REFSEL_Pos ) | // VDD/4 as reference
                              ( SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos   ) | // Gain control
                              ( SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos   ) | // Bypass resistor ladder
                              ( SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos   )); // Bypass resistor ladder

    // Configure the SAADC channel with AIN6 (P0.0X) as positive input, no negative input(single ended).
    NRF_SAADC->CH[6].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput7 << SAADC_CH_PSELP_PSELP_Pos;
    NRF_SAADC->CH[6].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    NRF_SAADC->CH[6].CONFIG = (( SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos ) | // Burst mode is disabled (normal operation)
                              ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos  ) | // Single ended, PSELN will be ignored, negative input to ADC shorted to GND
                              ( SAADC_CH_CONFIG_TACQ_10us        << SAADC_CH_CONFIG_TACQ_Pos  ) | // Acquisition time, the time the ADC uses to sample the input voltage
                              ( SAADC_CH_CONFIG_REFSEL_VDD1_4   << SAADC_CH_CONFIG_REFSEL_Pos ) | // VDD/4 as reference
                              ( SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos   ) | // Gain control
                              ( SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos   ) | // Bypass resistor ladder
                              ( SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos   )); // Bypass resistor ladder


    // No automatic sampling, will trigger with TASKS_SAMPLE.
    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

     //RAM Buffer pointer for ADC Result
    NRF_SAADC->RESULT.PTR  =(uint32_t)&m_buffer_pool;

    // Number of Sample Count
    NRF_SAADC->RESULT.MAXCNT = SAMPLES_IN_BUFFER;

    // Enable SAADC END interrupt to do maintenance and printing of values.
        NRF_SAADC->INTENSET = SAADC_INTENSET_END_Enabled << SAADC_INTENSET_END_Pos;

    NVIC_EnableIRQ(SAADC_IRQn);
}

void saadc_configure()
{
  // Enable SAADC. This should be done after the SAADC is configure due to errata 74 SAADC: Started events fires prematurely
  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
}


void saadc_read()
{
	// Start the ADC Start

	NRF_SAADC->TASKS_START = 1;
	while (NRF_SAADC->EVENTS_STARTED == 0);

    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;
    SAADC_IRQHandler();
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
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
//            NRF_LOG_INFO("SUCCESS\r\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
//            NRF_LOG_DEBUG("FAILED\r\n");
            (void) nrf_esb_flush_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:

            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS) ;

            if(rx_payload.data[0] == 0xFF)
            {
            	lStartCodon = true;
            	tx_payload.data[0] = 0x00;
                (void) nrf_esb_write_payload(&tx_payload);      // NRF_LOG_DEBUG("Queue transmitt packet: %02x\r\n", tx_payload.data[0]);
                break;
            }
            else if(rx_payload.data[0] == 0xEE)
            {
                (void) nrf_esb_write_payload(&tx_payload);      // NRF_LOG_DEBUG("Queue transmitt packet: %02x\r\n", tx_payload.data[0]);
                 break;
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
    uint32_t channel_1     = 0x00000000; // LEFT HEEL

#ifndef NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;

#else // NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_LEGACY_CONFIG;

#endif // NRF_ESB_LEGACY
    nrf_esb_config.selective_auto_ack       = 0;
    nrf_esb_config.payload_length           = NRF_ESB_MAX_PAYLOAD_LENGTH;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.tx_output_power          = NRF_ESB_TX_POWER_4DBM;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_rf_channel(channel_1);
    VERIFY_SUCCESS(err_code);

    tx_payload.length = 1;
//    rx_payload.length = NRF_ESB_MAX_PAYLOAD_LENGTH;
    return NRF_SUCCESS;
}



int main(void)
{
    uint32_t err_code = 0;
//    err_code = logging_init();
    APP_ERROR_CHECK(err_code);

    // Configure RF
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);
    clocks_start();

	// Configure I2C
	twi_init();

	// Configure and enable STEVAL Sensor
	setClickCnfg(true);
	enableAccel();

	// Configure SAADC
	saadc_init();

	// Enable SAADC Sampling Task
	saadc_configure();

    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);


    while (true)
    {
    	if(lStartCodon){

		uint8_t* digitalresults;       // NRF_LOG_DEBUG("Receiving packet: %x\r\n", rx_payload.data[0]);

		int8_t MSB[SAMPLES_IN_BUFFER];
		int8_t LSB[SAMPLES_IN_BUFFER];

		saadc_read();
		digitalresults = readAccelXYZ();

	   // Splitting
		for(int i=0 ; i< SAMPLES_IN_BUFFER; i++)
		{
			MSB[i] = m_buffer_pool[0][i] >> 8;
			LSB[i] = m_buffer_pool[0][i] & 0x00FF;
		}

		for(int i=0; i < 6; i++)
		{
			tx_payload.data[i] = digitalresults[i];
		}

		for(int k=0; k < SAMPLES_IN_BUFFER; k++)
		{
			tx_payload.data[k+6] = MSB[k];
		}

		for(int j=0; j < SAMPLES_IN_BUFFER; j++)
		{
			tx_payload.data[j+13] = LSB[j];
		}
			tx_payload.length = 32;
    }
    }
    lStartCodon = false;
}
/*lint -restore */
