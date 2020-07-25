
#include "mic.h"
#include "deck.h"
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "worker.h"
#include "nvicconf.h"
#include "log.h"

#include "arm_math.h"
#include "arm_const_structs.h"
#include "motor_sound.h"

static bool isInit = false;
static bool isTimerInit = false;
static bool isAdcInit = false;
static bool isDmaInit = false;
static bool isGpioInit = false;

static uint16_t dmaBufferBottom[BUFFER_SIZE];
static uint16_t dmaBufferTop[BUFFER_SIZE];
//TODO: when we premultiply with chirp it will be twice as long
// and we will need another buffer of the same size for the chirp coeffs
static float32_t fftWindow[BUFFER_SIZE];
static float32_t fftChirped[2*BUFFER_SIZE];
static float32_t chirpTemplate[2*BUFFER_SIZE]; // complex chirp

static uint16_t chirpLen = 0;
static uint16_t chirpSlope = 2000;

static int8_t readyBuffer = -1; // changes to 0 or 1 as buffers are filled by DMA
static float windowEnergy = 0;
static uint16_t lastValue = 0;
static uint16_t nInterrupts = 0;
static uint16_t nReads = 0;
static uint16_t adcInterrupts = 0;
static float maxFreqTop = 0;
static float maxFreqBottom = 0;
static float32_t maxFreqVal = 0;
static float dfTop = 0;
static float dfBottom = 0;

static xTimerHandle timer;
static void micReadTimer(xTimerHandle timer);

static NVIC_InitTypeDef NVIC_InitStructure;
const static arm_cfft_instance_f32*  fftParams = &arm_cfft_sR_f32_len1024;

#define E_PI ((float32_t)3.14159265)
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

void micInit(DeckInfo *info)
{
  if (isInit) return;
  if (!isGpioInit) micGpioInit();
  if (!isAdcInit) micAdcInit();
  if (!isDmaInit) micDmaInit();
  if (!isTimerInit) micSamplerInit();

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

static void prepareChirpMultiplier()
{
  // this is using a lot of RAM....
  static float32_t lChpBuffer[BUFFER_SIZE];
  static float32_t kBuffer[BUFFER_SIZE];
  int ii;
  float32_t chpFactor;
  // prepare the vector -N/2 .. N/2
  float32_t d = (float32_t)(BUFFER_SIZE)/(BUFFER_SIZE-1);
  for (ii=0; ii<BUFFER_SIZE; ii++) {
    kBuffer[ii] = -(BUFFER_SIZE>>1) + ii*d;
  }

  fs = (float32_t)(SAMPLE_RATE)
  // complex angle = 2*pi*s*k^2/(2*Fs^2)
  arm_mult_f32(kBuffer, kBuffer, lChpBuffer, BUFFER_SIZE);
  arm_scale_f32(lChpBuffer, ((float32_t)360.0*chirpSlope)/(2*fs*fs))

  // make the real and imaginary parts of the complex sinusoid
  for (ii=0; ii<BUFFER_SIZE; ii++) {
    chirpTemplate[2*ii + 0] = arm_cos_f32(lChpBuffer[ii]);
    chirpTemplate[2*ii + 1] = arm_sin_f32(lChpBuffer[ii]);
  }

}

static void updateChirpParams()
{
  const MotorSoundParameters *p = currentMotorParams();

  bool slopeChanged = (p->chirpSlope != chirpSlope);
  chirpLen = p->chirpLen;
  chirpSlope = p->chirpSlope;

  if (slopeChanged) prepareChirpMultiplier();
}



static void convertDataToFloat(uint16_t *dmaBuffer)
{
  // PROCESS THE DATA QUICKLY!!
  windowEnergy = 0;
  nInterrupts = nInterrupts+1;
  for (int ii=0; ii<BUFFER_SIZE; ii++) {
    lastValue = dmaBuffer[ii];
    windowEnergy = windowEnergy + (float)lastValue-ADC_OFFSET;
    fftWindow[ii] = (float32_t)lastValue - ADC_OFFSET;
  }
}

// process data windows if available
static void processMicBuffer()
{

  //static arm_rfft_fast_instance_f32 realParams;
  //arm_rfft_fast_init_f32(&realParams, BUFFER_SIZE);
  uint16_t *dmaBuffer = NULL;
  if (readyBuffer < 0) return;

  if (readyBuffer == 0) {
    dmaBuffer = dmaBufferTop;
  }else{
    dmaBuffer = dmaBufferBottom;
  }
  updateChirpParams();
  readyBuffer = -1;

  // copy the ADC data as floating point
  convertDataToFloat(dmaBuffer);
  nReads = nReads+1;

  // multiply by the complex chirp
  arm_cmplx_mult_real_f32(chirpTemplate, fftWindow, fftChirped, BUFFER_SIZE);
  // take the fft of the chirped audio
  arm_cfft_f32(fftParams, fftChirped, 0, 0);
  //arm_rfft_fast_f32(&realParams, fftWindow, fftChirped, 0);
  // get the (real) magnitude of the complex fft
  arm_cmplx_mag_f32(fftChirped, fftWindow, BUFFER_SIZE);

  uint32_t maxBin = 0;
  arm_max_f32(fftWindow+START_BIN, BUFFER_SIZE/2 - START_BIN, &maxFreqVal, &maxBin);
  
  float lastTop = maxFreqTop;
  maxFreqTop = (float)(maxBin+START_BIN)*((float)SAMPLE_RATE/BUFFER_SIZE);
  dfTop = maxFreqTop - lastTop;

  float lastBottom = maxFreqBottom;
  arm_max_f32(fftWindow+BUFFER_SIZE/2, BUFFER_SIZE/2 - START_BIN, &maxFreqVal, &maxBin);
  maxFreqBottom =  (float)(maxBin) *((float)SAMPLE_RATE/BUFFER_SIZE) - (float)(SAMPLE_RATE/2);
  dfBottom = maxFreqBottom - lastBottom;
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
  LOG_ADD(LOG_UINT16, rawRead, &lastValue)
  LOG_ADD(LOG_UINT16, triggers, &nInterrupts)
  LOG_ADD(LOG_FLOAT, topFreq, &maxFreqTop) 
  LOG_ADD(LOG_FLOAT, bottomFreq, &maxFreqBottom) 
  LOG_ADD(LOG_FLOAT, dBottom, &dfBottom) 
  LOG_ADD(LOG_FLOAT, dTop, &dfTop) 
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
