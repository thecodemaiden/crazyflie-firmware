
#include "mic.h"
#include "deck.h"
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "worker.h"
#include "nvicconf.h"
#include "log.h"


static bool isInit = false;
static bool isDmaInit = false;

static uint16_t dmaBuffer[BUFFER_SIZE];
//static int headPtr = BUFFER_SIZE-1;
static float windowEnergy = 0;
static uint16_t lastValue = 0;
static uint16_t nInterrupts = 0;
static uint16_t nReads = 0;
static uint16_t adcInterrupts = 0;
static float filteredOut = 0;

static xTimerHandle timer;
static void micReadTimer(xTimerHandle timer);

static DMA_InitTypeDef DMA_InitStructure;
static void micDmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  /* enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  //adcInit()
  // Direct paste from deck api so we can config more
  ADC_DeInit();

  /* Define ADC init structures */
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  /* Populates structures with reset values */
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonStructInit(&ADC_CommonInitStructure);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Populate structure with RESET values. */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  /* Initialise PA3 to analog mode. */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* init ADCs in independent mode, div clock by two */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; /* HCLK = 168MHz, PCLK2 = 84MHz, ADCCLK = 42MHz (when using ADC_Prescaler_Div2) */
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* Init ADC3: 12bit, single-conversion. For Arduino compatibility set 10bit */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  ADC_ITConfig(ADC3, ADC_IT_OVR, ENABLE);

  // interrupt enable -- stream overrun
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable ADC3 */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
  ADC_Cmd(ADC3, ENABLE);

  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC3->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dmaBuffer;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);

  // interrupt enable - transfer complete
  DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // ENABLE DMA
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  DMA_Cmd(DMA2_Stream1, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  isDmaInit = true;
}


void micInit(DeckInfo *info)
{
  if (isInit) return;
  if (!isDmaInit) micDmaInit();

  isInit = true;
  timer = xTimerCreate("micTimer", M2T(50),pdTRUE,NULL,micReadTimer);
 // xTimerStart(timer,100);
ADC_SoftwareStartConv(ADC3);
}

bool micTest()
{
  bool reallyDidInit = isDmaInit;
  reallyDidInit &= (DMA_GetCmdStatus(DMA2_Stream1)==ENABLE); 
  return isDmaInit && reallyDidInit; 
}

/*** periodic analog read + process ***/

void updateMicBuffer()
{
//  if (!(DMA_GetCmdStatus(DMA2_Stream1) == DISABLE)){
//    DMA_Init(DMA2_Stream1, &DMA_InitStructure);
//    DMA_Cmd(DMA2_Stream1, ENABLE);
//  }
//ADC_SoftwareStartConv(ADC3);
//while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET);
//nReads = nReads + 1;
}

static void micReadTimer(xTimerHandle timer)
{
  workerSchedule(updateMicBuffer, NULL);
}


const DeckDriver micDriver = {
  .vid = 0,
  .pid = 0,
  .name = "analogMic",
  .usedGpio = DECK_USING_PA3,
  .init = micInit,
  .test = micTest,
};

DECK_DRIVER(micDriver);

LOG_GROUP_START(mic)
  LOG_ADD(LOG_FLOAT, energy, &windowEnergy)
  LOG_ADD(LOG_FLOAT, filter, &filteredOut)
  LOG_ADD(LOG_UINT16, rawRead, &lastValue)
  LOG_ADD(LOG_UINT16, triggers, &nInterrupts)
  LOG_ADD(LOG_UINT16, nRead, &nReads)
  LOG_ADD(LOG_UINT16, adc, &adcInterrupts)
LOG_GROUP_STOP(mic)

/** 
 * Interrupt handling etc
 **/
void __attribute__((used)) ADC_IRQHandler(void)
{
  FlagStatus isOverrun = ADC_GetFlagStatus(ADC3, ADC_FLAG_OVR);
  adcInterrupts += 1;
  if (isOverrun) {
    nInterrupts = 0xdead;
    // clear the flag
    ADC_ClearFlag(ADC3, ADC_FLAG_OVR);
    ADC_ClearITPendingBit(ADC3, ADC_IT_OVR);
  }
}
#define ADC_OFFSET (2047.0f)
void __attribute__((used)) DMA2_Stream1_IRQHandler(void)
{
 // portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  //DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, DISABLE);
  // Clear stream flags
  DMA_ClearITPendingBit(DMA2_Stream1, DMA_FLAG_TCIF1);
  DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);

  // PROCESS THE DATA QUICKLY!!
  windowEnergy = 0;
  nInterrupts = nInterrupts+1;
  for (int ii=0; ii<BUFFER_SIZE; ii++) {
    lastValue = dmaBuffer[ii];
    windowEnergy = windowEnergy + (float)lastValue-ADC_OFFSET;
    filteredOut = 0.8f*filteredOut + 0.2f*(lastValue - ADC_OFFSET);
  }
 // DMA_MemoryTargetConfig(DMA2_Stream1, dmaBuffer);
  //DMA_SetCurrDataCounter(DMA2_Stream1, BUFFER_SIZE);
  //DMA_Cmd(DMA2_Stream1, DISABLE);

// TODO: use semaphore to access the buffer data?
//  xSemaphoreGiveFromISR(txComplete, &xHigherPriorityTaskWoken);

//  if (xHigherPriorityTaskWoken)
//  {
 //   portYIELD();
//  }
}
