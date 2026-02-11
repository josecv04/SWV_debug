//Adding here -jose 
extern "C" {
  #include "ad5940.h"
  #include "SqrWaveVoltammetry.h"
  #include "BleComm.h"
  #include <LibPrintf.h>
}




/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  $Author: nxu2 $
 @brief:   Used to control specific application and process data.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#include "SqrWaveVoltammetry.h"

/**
   User could configure following parameters
**/

#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

/**
 * @brief An example to deal with data read back from AD5940. Here we just print data to UART
 * @note UART has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
 * @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/
static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  static uint32_t index = 0;
  /* Print/stream data.
   * NOTE: pData has already been converted to microamps (uA) inside AppSWVDataProcess().
   * IMPORTANT: We DO NOT infer phase from index parity here. Instead, we request phase
   * metadata from the SWV engine (AppSWV_StreamNextPhaseChar()), which tracks the SWV
   * high/low progression.
   */
  for (uint32_t i = 0; i < DataCount; i++)
  {
    // Optional pacing (UART can be slow). Reduce/remove if you need max throughput.
    //delay(10);

    uint32_t step = AppSWV_StreamNextStepIndex();
    char phase = AppSWV_StreamNextPhaseChar(); // 'F' for high, 'R' for low

    // BLE: send SWV-aware line
    ble_push_swv(index, step, phase, pData[i]);

    // UART: mirror BLE line
    printf("idx:%lu, step:%lu, phase:%c, I_uA:%.9f\n",
           (unsigned long)index,
           (unsigned long)step,
           phase,
           (double)pData[i]);

    index++;
  }
  return 0;
}

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;  
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */
	
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;   /* */
  fifo_cfg.FIFOThresh = 4;            /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIOs */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\n", LFOSCFreq);
 // AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}

/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/
void AD5940RampStructInit(void)
{
  AppSWVCfg_Type *pRampCfg;
  
  AppSWVGetCfg(&pRampCfg);
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 512-0x10;              /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 10000.0;                  /* 10kOhm RCAL */
  pRampCfg->ADCRefVolt = 1.820f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 1023;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;           /* LFOSC frequency */
	pRampCfg->AdcPgaGain = ADCPGA_1P5;
	pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;
  
	/* Step 2:Configure square wave signal parameters */
  pRampCfg->RampStartVolt = 100.0f;  /* OG settings: 200*/  /* Barak settings: 100*/ /* Measurement starts at 0V*/
  pRampCfg->RampPeakVolt = 500.0f;    /* OG settings: 800*/  /* Barak settings: 500*/		 /* Measurement finishes at -0.4V */
  pRampCfg->VzeroStart = 1300.0f;           /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->VzeroPeak = 1300.0f;          /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->Frequency = 400;    /* OG settings: 300*/     /* Barak settings: 400*/        /* Frequency of square wave in Hz */
  pRampCfg->SqrWvAmplitude = 50;    /* OG settings: 25*/  /* Barak settings: 50*/ /* Amplitude of square wave in mV */
  pRampCfg->SqrWvRampIncrement = 2; /* OG settings: 5*/  /* Barak settings: 1*/  /* Increment in mV*/
  pRampCfg->SampleDelay = 0.2f;             /* Time between update DAC and ADC sample. Unit is ms and must be < (1/Frequency)/2 - 0.2*/
  pRampCfg->LPTIARtiaSel = LPTIARTIA_1K;      /* Maximum current decides RTIA value */
	pRampCfg->bRampOneDir = bTRUE;			/* Only measure ramp in one direction */
}

void AD5940_Main(void)
{
  uint32_t temp;

  const uint32_t RESTART_DELAY_MS = 60000; // 1 minute
  bool waiting_for_restart = false;
  uint32_t run_complete_ms = 0;

  AD5940PlatformCfg();
  AD5940RampStructInit();

  AppSWVInit(AppBuff, APPBUFF_SIZE);    /* Initialize SWV application */

  AD5940_Delay10us(100000);             /* Allow sensor to reach equilibrium before starting */
  AppSWVCtrl(APPCTRL_START, 0);

  while(1)
  {
    /* If a sweep finished, wait 60s, then restart */
    if(waiting_for_restart)
    {
      if((uint32_t)(millis() - run_complete_ms) >= RESTART_DELAY_MS)
      {
        AD5940RampStructInit();
        AppSWVInit(AppBuff, APPBUFF_SIZE);
        AppSWVCtrl(APPCTRL_START, 0);

        waiting_for_restart = false;
        printf("[SWV] Restarting sweep after 60s park delay\n");
      }
    }

    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppSWVISR(AppBuff, &temp);
      RampShowResult((float*)AppBuff, temp);

      if(AppSWV_ConsumeRunCompleteFlag())
      {
        waiting_for_restart = true;
        run_complete_ms = millis();
        printf("[SWV] Sweep complete. Parked at ~0 mV. Waiting 60s...\n");
      }
    }
	  
	delay(1);
}



//Insert my code here 
//Initialization 

void setup() {
 ble_init();  // start BLE advertising for amperometric data

  Serial.begin(115200);
  ////pinMode(13, OUTPUT);
  delay(20000);
  printf("start\n");
  //digitalWrite(13, HIGH);
  AD5940_MCUResourceInit(NULL);
  printf("initialized\n");
  AD5940_Main();
  //digitalWrite(13, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}
