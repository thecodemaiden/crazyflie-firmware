
#include "mic.h"
#include "deck.h"
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "worker.h"
#include "nvicconf.h"
#include "log.h"

#include "arm_math.h"

#define SAMPLE_RATE (8000)
#define ADC_SAMPLE_PERIOD ((84000000L/SAMPLE_RATE)-1)


static bool isInit = false;
static bool isTimerInit = false;
static bool isAdcInit = false;
static bool isDmaInit = false;
static bool isGpioInit = false;
static bool isFftInit = false;

static uint16_t dmaBufferBottom[BUFFER_SIZE];
static uint16_t dmaBufferTop[BUFFER_SIZE];
//TODO: when we premultiply with chirp it will be twice as long
// and we will need another buffer of the same size for the chirp coeffs
static float32_t fftWindow[BUFFER_SIZE];
static float32_t fftOut[BUFFER_SIZE];

static int8_t readyBuffer = -1; // changes to 0 or 1 as buffers are filled by DMA
static float windowEnergy = 0;
static uint16_t lastValue = 0;
static uint16_t nInterrupts = 0;
static uint16_t nReads = 0;
static uint16_t adcInterrupts = 0;
static float filteredOut = 0;
static float maxFreq = 0;
static float32_t maxFreqVal = 0;

static xTimerHandle timer;
static void micReadTimer(xTimerHandle timer);

static NVIC_InitTypeDef NVIC_InitStructure;
static arm_rfft_fast_instance_f32 fftParams;

static void micGpioInit(void)
{
  /* Populate structure with RESET values. */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  /* Initialise PA3 to analog mode. */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  isGpioInit = true;
}

static void micAdcInit(void)
{
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
  /* init ADCs in independent mode, div clock by two */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; /* HCLK = 168MHz, PCLK2 = 84MHz, ADCCLK = 42MHz (when using ADC_Prescaler_Div2) */
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* Init ADC3: 12bit, single-conversion. For Arduino compatibility set 10bit */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  // trigger using TIM3_TRG0
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
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

  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  isAdcInit = true;
}

static void micDmaInit(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC3->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dmaBufferBottom;
  //DMA_InitStructure.DMA_Memory1BaseAddr = (uint32_t)dmaBufferTop;
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

  // set up double buffering
  DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)dmaBufferTop, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);

  // interrupt enable - transfer complete
  DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // ENABLE DMA
  DMA_Cmd(DMA2_Stream1, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);

  isDmaInit = true;
}

static void micSamplerInit(void)
{
  /* Prepare timer 3 to generate our samples - start with 8kHz */
  TIM_TimeBaseInitTypeDef TIM_BaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  TIM_BaseStructure.TIM_Period = ADC_SAMPLE_PERIOD;
  TIM_BaseStructure.TIM_Prescaler = 0;
  TIM_BaseStructure.TIM_ClockDivision = 0;
  TIM_BaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_BaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_BaseStructure);
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  TIM_Cmd(TIM3, ENABLE);
  
  isTimerInit = true;
}

static void micFftInit(void)
{
  isFftInit = (arm_rfft_fast_init_f32(&fftParams, BUFFER_SIZE) == ARM_MATH_SUCCESS);
}

void micInit(DeckInfo *info)
{
  if (isInit) return;
  if (!isGpioInit) micGpioInit();
  if (!isAdcInit) micAdcInit();
  if (!isDmaInit) micDmaInit();
  if (!isTimerInit) micSamplerInit();
  if (!isFftInit) micFftInit();

  isInit = true;
  timer = xTimerCreate("micTimer", M2T(10),pdTRUE,NULL,micReadTimer);
  xTimerStart(timer,100);
}

bool micTest()
{
  bool reallyDidInit = isInit;
  reallyDidInit &= (DMA_GetCmdStatus(DMA2_Stream1)==ENABLE); 
  return isDmaInit && reallyDidInit; 
}

// process data windows if available
static void processMicBuffer()
{
#define ADC_OFFSET (2047.0f)
  uint16_t *dmaBuffer = NULL;
  if (readyBuffer < 0) return;

  if (readyBuffer == 0) {
    dmaBuffer = dmaBufferTop;
  }else{
    dmaBuffer = dmaBufferBottom;
  }
  // PROCESS THE DATA QUICKLY!!
  windowEnergy = 0;
  nInterrupts = nInterrupts+1;
  for (int ii=0; ii<BUFFER_SIZE; ii++) {
    lastValue = dmaBuffer[ii];
    windowEnergy = windowEnergy + (float)lastValue-ADC_OFFSET;
    filteredOut = 0.8f*filteredOut + 0.2f*(lastValue - ADC_OFFSET);
    fftWindow[ii] = (float32_t)lastValue - ADC_OFFSET;
  }
  nReads = nReads+1;
  readyBuffer = -1;
  // do the FFT here
  uint32_t maxBin = 0;
  arm_rfft_fast_f32(&fftParams, fftWindow, fftOut, 0);
  // change the complex values back to real values
  arm_cmplx_mag_f32(fftOut, fftWindow, BUFFER_SIZE/2);
  // get the max frequency, ignore bin 0
  arm_max_f32(fftWindow+1, BUFFER_SIZE/2-1, &maxFreqVal, &maxBin);
  // calculate the max frequency from the bin
  maxFreq = (float)(maxBin+1)*((float)SAMPLE_RATE/BUFFER_SIZE);
}

static void micReadTimer(xTimerHandle timer)
{
  workerSchedule(processMicBuffer, NULL);
}


const DeckDriver micDriver = {
  .vid = 0,
  .pid = 0,
  .name = "analogMic",
  .usedGpio = DECK_USING_PA3,
  .usedPeriph = DECK_USING_TIMER3,
  .init = micInit,
  .test = micTest,
};

DECK_DRIVER(micDriver);

LOG_GROUP_START(mic)
  LOG_ADD(LOG_FLOAT, energy, &windowEnergy)
  LOG_ADD(LOG_FLOAT, filter, &filteredOut)
  LOG_ADD(LOG_UINT16, rawRead, &lastValue)
  LOG_ADD(LOG_UINT16, triggers, &nInterrupts)
  //LOG_ADD(LOG_UINT16, nRead, &nReads)
 // LOG_ADD(LOG_UINT16, adc, &adcInterrupts)
  LOG_ADD(LOG_FLOAT, maxFreq, &maxFreq) 
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
void __attribute__((used)) DMA2_Stream1_IRQHandler(void)
{
  // Check which buffer we should use to process the data
  uint32_t busyBuffer = DMA_GetCurrentMemoryTarget(DMA2_Stream1);
  // Clear stream flags
  readyBuffer = 1-(busyBuffer&1);
  DMA_ClearITPendingBit(DMA2_Stream1, DMA_FLAG_TCIF1);
  DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);

}
