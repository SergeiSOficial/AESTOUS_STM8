/*!
 * \file app.c
 * \author Sergei Savkin (geos2@mail.ru)
 * \author Andrey Kosinec
 * \brief User application for AESTOUS project
 * \version 0.1
 * \date 27-11-2020
 *
 * \copyright AESTOUS 2020
 *
 */
//#include "main.h"
#include "stm8s.h"

//LEDS!
#define LED_GPIO_PIN (GPIO_PIN_4)
#define LED_GPIO_PORT (GPIOD)

//Application ADC

#define ADC_GPIO_PINS34 (GPIO_PIN_3 | GPIO_PIN_4)
#define ADC_GPIO_PORT34 (GPIOD)
#define ADC_GPIO_PINS2 (GPIO_PIN_4)
#define ADC_GPIO_PORT2 (GPIOC)

#define ADC_CHANNELS ADC1_CHANNEL_4//(ADC1_CHANNEL_2 | ADC1_CHANNEL_3 | ADC1_CHANNEL_4)

static uint16_t CCR1_Val = 8000;

static uint16_t PWM_Val = 0;

float adc_battarey = 0;
float adc_button = 0;
float adc_charge_cap = 0;

/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void adc_config(void);
void timers_config(void);
void pwm_config(void);

void AppInit(void)
{
    adc_config();
    //timers_config();
    //pwm_config();
}

//void gpio_config(void)
//{
//    GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
//}

void adc_config(void)
{
    GPIO_Init(ADC_GPIO_PORT2, (GPIO_Pin_TypeDef)(ADC_GPIO_PINS2), GPIO_MODE_IN_FL_NO_IT);
    GPIO_Init(ADC_GPIO_PORT34, (GPIO_Pin_TypeDef)(ADC_GPIO_PINS34), GPIO_MODE_IN_FL_NO_IT);

    ADC1_DeInit();
    ADC1_ScanModeCmd(ENABLE);
    //ADC1_DataBufferCmd(ENABLE);
    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
    ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
              (ADC1_Channel_TypeDef)(ADC_CHANNELS),
              ADC1_PRESSEL_FCPU_D18,
              ADC1_EXTTRIG_TIM, DISABLE,
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_ALL, DISABLE);

    /* Enable EOC interrupt */
    ADC1_Cmd(ENABLE);
}

void timers_config(void)
{
    /* Time base configuration */
    TIM2_TimeBaseInit(TIM2_PRESCALER_1, 16000 - 1);

    /* PWM1 Mode configuration: Channel1 */
    TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, CCR1_Val, TIM2_OCPOLARITY_HIGH);
    TIM2_OC1PreloadConfig(ENABLE);

    TIM2_ARRPreloadConfig(ENABLE);

    /* TIM2 enable counter */
    TIM2_Cmd(ENABLE);
}

void pwm_config(void)
{
    TIM1_DeInit();

    /* Time Base configuration */
    /*
  TIM1_Period = 4095
  TIM1_Prescaler = 0
  TIM1_CounterMode = TIM1_COUNTERMODE_UP
  TIM1_RepetitionCounter = 0
  */

    TIM1_TimeBaseInit(0, TIM1_COUNTERMODE_UP, 4095, 0);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */

    /*
  TIM1_OCMode = TIM1_OCMODE_PWM2
  TIM1_OutputState = TIM1_OUTPUTSTATE_ENABLE
  TIM1_OutputNState = TIM1_OUTPUTNSTATE_ENABLE
  TIM1_Pulse = CCR1_Val
  TIM1_OCPolarity = TIM1_OCPOLARITY_LOW
  TIM1_OCNPolarity = TIM1_OCNPOLARITY_HIGH
  TIM1_OCIdleState = TIM1_OCIDLESTATE_SET
  TIM1_OCNIdleState = TIM1_OCIDLESTATE_RESET

  */
    TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                 PWM_Val, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);

    /* TIM1 counter enable */
    TIM1_Cmd(ENABLE);

    /* TIM1 Main Output Enable */
    TIM1_CtrlPWMOutputs(ENABLE);
}


void AppADCHandler(void)
{
    adc_battarey = ADC1_GetBufferValue(ADC1_CHANNEL_2) / 204.8;
    adc_button = ADC1_GetBufferValue(ADC1_CHANNEL_3) / 204.8;
    adc_charge_cap = ADC1_GetBufferValue(ADC1_CHANNEL_4) / 204.8 * 40;
}