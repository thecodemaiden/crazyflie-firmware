
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
static bool isFftInit = false;

static uint16_t dmaBufferBottom[BUFFER_SIZE];
static uint16_t dmaBufferTop[BUFFER_SIZE];
//TODO: when we premultiply with chirp it will be twice as long
// and we will need another buffer of the same size for the chirp coeffs
static float32_t fftWindow[BUFFER_SIZE];
static float32_t fftReal[BUFFER_SIZE];
static float32_t fftChirped[2*BUFFER_SIZE];
static float32_t chirpTemplate[2*BUFFER_SIZE]; // complex chirp
static uint16_t chirpLen = 750;
static uint16_t chirpSlope = 2000;

static int8_t readyBuffer = -1; // changes to 0 or 1 as buffers are filled by DMA
static float windowEnergy = 0;
static uint16_t lastValue = 0;
static uint16_t nReads = 0;
static uint16_t nIdle = 0;
static volatile uint16_t adcInterrupts = 0;
static float maxFreqTop = 0;
static float maxFreqBottom = 0;
static float32_t maxFreqVal = 0;
static float dfTop = 0;
static float dfBottom = 0;
static float maxTopVal = 0;
static float maxBottomVal = 0;
static arm_rfft_fast_instance_f32 realParams;
static float maxFreqReal = 0;
static uint32_t taskTime=0;
static uint8_t bitPointer=0;
static uint8_t msgPointer=0;
#define MSG_BUFFER_SIZE 4
uint16_t inOneCount = 0;
uint16_t inZeroCount = 0;
static float freqDelta = 0;
static uint16_t maxSymbolCount = 0;
static uint16_t minSymbolCount = 0;
static uint8_t msgBuffer[MSG_BUFFER_SIZE] = {0};

static xTimerHandle timer;
static void micReadTimer(xTimerHandle timer);
static void prepareChirpMultiplier();

static NVIC_InitTypeDef NVIC_InitStructure;
const static arm_cfft_instance_f32*  fftParams;

#define E_PI ((float32_t)3.14159265)

#define BIN_TO_FREQ(x) ((float)(x)*SAMPLE_RATE/BUFFER_SIZE)

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

static void fftInit()
{
  uint16_t length = BUFFER_SIZE;
  switch (length) {
    case 16:
      fftParams = &arm_cfft_sR_f32_len16;
      break;
    case 32:
      fftParams = &arm_cfft_sR_f32_len32;
      break;
    case 64:
      fftParams = &arm_cfft_sR_f32_len64;
      break;
    case 128:
      fftParams = &arm_cfft_sR_f32_len128;
      break;
    case 256:
      fftParams = &arm_cfft_sR_f32_len256;
      break;
    case 512:
      fftParams = &arm_cfft_sR_f32_len512;
      break;
    case 1024:
      fftParams = &arm_cfft_sR_f32_len1024;
      break;
    case 2048:
      fftParams = &arm_cfft_sR_f32_len2048;
      break;
    case 4096:
      fftParams = &arm_cfft_sR_f32_len4096;
      break;
  }
  isFftInit = (arm_rfft_fast_init_f32(&realParams, BUFFER_SIZE)) == ARM_MATH_SUCCESS;

}

void micInit(DeckInfo *info)
{
  if (isInit) return;
  if (!isGpioInit) micGpioInit();
  if (!isAdcInit) micAdcInit();
  if (!isDmaInit) micDmaInit();
  if (!isTimerInit) micSamplerInit();
  if (!isFftInit) fftInit();

  prepareChirpMultiplier();
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
 int ii;
  for (ii=0; ii<BUFFER_SIZE; ii++) {
      float32_t ts = ((float32_t)ii)/SAMPLE_RATE;
      float32_t phase = E_PI*chirpSlope*ts*ts;
      chirpTemplate[2*ii] = arm_cos_f32(phase);
      chirpTemplate[2*ii+1] = arm_sin_f32(phase);
  }
  // expected change in frequency during a window
  freqDelta = ((float32_t)BUFFER_SIZE)*chirpSlope/SAMPLE_RATE;
  maxSymbolCount = (uint16_t)((float32_t)chirpLen/1000.0f*SAMPLE_RATE/BUFFER_SIZE);
  minSymbolCount = (uint16_t)(0.6f*maxSymbolCount);
}

//
//static void updateChirpParams()
//{
//  const MotorSoundParameters *p = currentMotorParams();
//
//  bool slopeChanged = (p->chirpSlope != chirpSlope);
//  chirpLen = p->chirpLen;
//  chirpSlope = p->chirpSlope;
//
//  if (slopeChanged) prepareChirpMultiplier();
//}
//


static void convertDataToFloat(uint16_t *dmaBuffer)
{
  // PROCESS THE DATA QUICKLY!!
  windowEnergy = 0;
  nReads = nReads+1;
  for (int ii=0; ii<BUFFER_SIZE; ii++) {
    lastValue = dmaBuffer[ii];
    windowEnergy = windowEnergy + (float)lastValue-ADC_OFFSET;
    fftWindow[ii] = ((float32_t)lastValue - ADC_OFFSET)/ADC_OFFSET;
  }
}

static void insertMsgBit(uint8_t bit)
{
  uint8_t shiftedBit = bit << bitPointer;
  uint8_t maskedMsg = msgBuffer[msgPointer] & ~(1 << bitPointer);
  msgBuffer[msgPointer] = maskedMsg | shiftedBit;
  bitPointer++;
  if (bitPointer >= 8) {
    bitPointer = 0;
    msgPointer++;
  }
  if (msgPointer >= MSG_BUFFER_SIZE) {
    msgPointer = 0;
  }
}

static void decodeChirps()
{
  static float pThreshold = 5.5;
  bool inOne = false;
  bool inZero = false;
  float freqError = 0.5f*freqDelta;

  if ((dfTop < 0) && (maxTopVal > pThreshold) &&
      (dfTop+freqDelta > -freqError) && (dfTop+freqDelta < freqError)){
    inOne = true;
  }
  if ((dfBottom < 0) && (maxBottomVal > pThreshold) &&
      (dfBottom+freqDelta > -freqError) && (dfBottom+freqDelta < freqError)){
    inZero = true;
  }
  if (inOne && !inZero) inOneCount++;
  if (inZero && !inOne) inZeroCount++;

  if (inOneCount >= maxSymbolCount) {
    inOneCount = 0;
    inZeroCount = 0;
    insertMsgBit(1);
    return;
  }

  if (inZeroCount >= maxSymbolCount) {
    inOneCount = 0;
    inZeroCount = 0;
    insertMsgBit(0);
    return;
  }

  if (inOneCount > 0 && inZero) {
    if (inOneCount >= minSymbolCount) insertMsgBit(1);
    inOneCount = 0;
    return;
  }

  if (inZeroCount > 0 && inOne) {
    if (inZeroCount >= minSymbolCount) insertMsgBit(0);
    inZeroCount = 0;
    return;
  }

}

// process data windows if available
static void processMicBuffer()
{

  uint16_t *dmaBuffer = NULL;
  if (readyBuffer < 0) {
    nIdle += 1;
    return;
  }
  if (readyBuffer == 0) {
    dmaBuffer = dmaBufferTop;
  }else{
    dmaBuffer = dmaBufferBottom;
  }
  readyBuffer = -1;

  uint32_t taskStart = xTaskGetTickCount();
  static float32_t tempBuffer[2*BUFFER_SIZE];
  uint32_t maxBin = 0;
  // copy the ADC data as floating point
  convertDataToFloat(dmaBuffer);

  arm_cmplx_mult_real_f32(chirpTemplate, fftWindow, tempBuffer, BUFFER_SIZE);
  // find the fft max bin without chirping
  arm_rfft_fast_f32(&realParams, fftWindow, fftReal, 0);
  arm_cmplx_mag_f32(fftReal, fftChirped, BUFFER_SIZE);
  arm_max_f32(fftChirped+START_BIN, BUFFER_SIZE/2-START_BIN, &maxFreqVal, &maxBin);
  maxFreqReal = BIN_TO_FREQ(maxBin+START_BIN);

  // find the fft max bin with chirping
  // TODO: for multiple transmissions, just do this max and min in each bin
  arm_cfft_f32(fftParams, tempBuffer, 0, 1);
  arm_cmplx_mag_f32(tempBuffer, fftChirped, BUFFER_SIZE);
  arm_max_f32(fftChirped+START_BIN, BUFFER_SIZE/2 - START_BIN, &maxTopVal, &maxBin);
  float meanValTop = 0;
  arm_mean_f32(fftChirped+START_BIN, BUFFER_SIZE/2 - START_BIN, &meanValTop);
  maxTopVal = maxTopVal/meanValTop;
  float lastTop = maxFreqTop;
  maxFreqTop = BIN_TO_FREQ(maxBin+START_BIN);
  dfTop = maxFreqTop - lastTop;

  arm_max_f32(fftChirped+BUFFER_SIZE/2, BUFFER_SIZE/2 - START_BIN, &maxBottomVal, &maxBin);
  float meanValBottom = 0;
  arm_mean_f32(fftChirped+BUFFER_SIZE/2, BUFFER_SIZE/2 - START_BIN, &meanValBottom);
  maxBottomVal = maxBottomVal/meanValBottom;
  float lastBottom = maxFreqBottom;
  maxFreqBottom =  BIN_TO_FREQ(maxBin) - SAMPLE_RATE/2.0f;
  dfBottom = maxFreqBottom - lastBottom;

  decodeChirps();
  uint32_t taskEnd = xTaskGetTickCount();
  taskTime = taskEnd - taskStart;
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

LOG_GROUP_START(sndDebug)
  LOG_ADD(LOG_UINT16, bufReads, &nReads)
  LOG_ADD(LOG_UINT16, triggers, &adcInterrupts)
  LOG_ADD(LOG_UINT16, idleCount, &nIdle)
  LOG_ADD(LOG_UINT32, taskTime, &taskTime)
LOG_GROUP_STOP(sndDebug)

LOG_GROUP_START(sndMsg)
  LOG_ADD(LOG_UINT8, msg1, msgBuffer)
  LOG_ADD(LOG_UINT8, msg2, msgBuffer+1)
  LOG_ADD(LOG_UINT8, msg3, msgBuffer+2)
  LOG_ADD(LOG_UINT8, msg4, msgBuffer+3)
  LOG_ADD(LOG_FLOAT, df, &freqDelta)
LOG_GROUP_STOP(sndMsg)

LOG_GROUP_START(mic)
  LOG_ADD(LOG_FLOAT, energy, &windowEnergy)
  LOG_ADD(LOG_UINT16, rawRead, &lastValue)
  LOG_ADD(LOG_FLOAT, topFreq, &maxFreqTop) 
  LOG_ADD(LOG_FLOAT, rawFreq, &maxFreqReal) 
  LOG_ADD(LOG_FLOAT, bottomFreq, &maxFreqBottom) 
  LOG_ADD(LOG_FLOAT, topVal, &maxTopVal) 
  LOG_ADD(LOG_FLOAT, bottomVal, &maxBottomVal) 
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
    adcInterrupts = 0xdead;
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
