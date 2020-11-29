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

uint32_t time = 0;

float adc_battarey = 0;
float adc_button = 0;
float adc_charge_cap = 0;

float testval = 0;
uint32_t testval2 = 0;

uint16_t mesure = 0; //ïåðåìåííàÿ äëÿ ñ÷åò ïðîïóñêà ïðåðûâàíèé adc äëÿ ïàóçû ïðè èçìåðåíèè â ìîìåíò çàðÿäêè

float target_voltage = 50;
float readed_voltage = 0;

uint8_t work_status = 1; // 0 - íè÷åãî 1 - ñòàáèëèçèðóåì 2 - çàðÿæàåì
uint8_t solution_to_inject = 0;
uint8_t percents = 0; //Â ýòîé ïåðåìåííîé õðàíèì íà ñêîëüêî ïðîóåíòîâ çàðÿäèëè

uint32_t work_time = 30000; //Âðåìÿ ðàáîòû ïîñëå ñòàáèëèçàöèè íàïðÿæåíèÿ

uint16_t RGB_red = 0;
uint16_t RGB_green = 1000;
uint16_t RGB_blue = 0;

uint16_t LED_green_PWM = 1000;

uint8_t bite_status = 0;

#define GPIO_A_PORT (GPIOA)
#define GPIO_B_PORT (GPIOB)
#define GPIO_C_PORT (GPIOC)
#define GPIO_D_PORT (GPIOD)

//LEDS!
#define LED_Y_1 (GPIO_PIN_2) //Not inverted  LED   (port A)
#define LED_Y_2 (GPIO_PIN_3) //Inverted      LED   (port C)
#define RGB_1 (GPIO_PIN_4)   //Inverted      LED   (port C)
#define RGB_2 (GPIO_PIN_5)   //Inverted      LED   (port C)
#define RGB_3 (GPIO_PIN_6)   //Inverted      LED   (port C)

//Application GPIO
#define Soft_Run (GPIO_PIN_1)         //                  (port A)
#define HV_charge_enable (GPIO_PIN_7) //          (port C)
#define Fire (GPIO_PIN_3)             //                      (port D)

//Application ADC
#define ADC_Batterey (GPIO_PIN_5) //              (port D)
#define ADC_Button (GPIO_PIN_6)   //              (port D)
#define ADC_HV (GPIO_PIN_2)       //              (port D)                   //ïðè òåêóùåé êîíôèãóðàöèè äåëèòåëåé 50 -> 1.25

/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void adc_config(void);
void gpio_config(void);
void timers_config(void);
void pwm_config(void);


void application(void)
{

    gpio_config();
    adc_config();
    timers_config();
    pwm_config();
}

void gpio_config(void)

{
    // GPIO_DeInit;
    // GPIO_Init(GPIO_C_PORT, LED_Y_2, GPIO_MODE_OUT_PP_LOW_FAST);
    // GPIO_Init(GPIO_A_PORT, LED_Y_1 | Soft_Run, GPIO_MODE_OUT_PP_LOW_FAST);
    // GPIO_Init(GPIO_C_PORT, HV_charge_enable, GPIO_MODE_OUT_PP_LOW_FAST);
    // GPIO_Init(GPIO_D_PORT, Fire, GPIO_MODE_OUT_PP_LOW_FAST);

    // GPIO_WriteHigh(GPIO_C_PORT, HV_charge_enable);
}

void adc_config(void)
{
    GPIO_Init(GPIO_D_PORT, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);

    ADC1_DeInit();
    ADC1_ScanModeCmd(ENABLE);
    ADC1_DataBufferCmd(ENABLE);
    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
              ADC1_CHANNEL_3 | ADC1_CHANNEL_5 | ADC1_CHANNEL_6,
              ADC1_PRESSEL_FCPU_D10,
              ADC1_EXTTRIG_TIM, ENABLE,
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_ALL, DISABLE);

    enableInterrupts();
    ADC1_Cmd(ENABLE);
    ADC1_StartConversion();
}

void timers_config(void)
{

    TIM1_DeInit();
    TIM1_TimeBaseInit(16, TIM1_COUNTERMODE_UP, 1000 - 1, 0);
    TIM1_SelectOutputTrigger(TIM1_TRGOSOURCE_UPDATE);
    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    TIM1_Cmd(ENABLE);

    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM2_PRESCALER_16, 1000 - 1);
    TIM2_Cmd(ENABLE);

    TIM4_DeInit();
    TIM4_TimeBaseInit(TIM4_PRESCALER_16, 10 - 1);
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
}

void pwm_config(void)
{

    TIM1_CtrlPWMOutputs(ENABLE);

    TIM1_OC1Init(TIM1_OCMODE_PWM1,
                 TIM1_OUTPUTSTATE_ENABLE,
                 TIM1_OUTPUTNSTATE_DISABLE,
                 RGB_red,
                 TIM1_OCPOLARITY_LOW,
                 TIM1_OCNPOLARITY_LOW,
                 TIM1_OCIDLESTATE_RESET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC1PreloadConfig(ENABLE);

    TIM1_OC4Init(TIM1_OCMODE_PWM1,
                 TIM1_OUTPUTSTATE_ENABLE,
                 RGB_blue,
                 TIM1_OCPOLARITY_LOW,
                 TIM1_OCIDLESTATE_RESET);
    TIM1_OC4PreloadConfig(ENABLE);

    TIM1_ARRPreloadConfig(ENABLE);


    TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, RGB_green, TIM2_OCPOLARITY_LOW);
    TIM2_OC1PreloadConfig(ENABLE);
    TIM2_ARRPreloadConfig(ENABLE);
}
