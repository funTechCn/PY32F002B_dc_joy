#include "main.h"
#include "py32f002bxx_ll_Start_Kit.h"
#include "py32f002b_ll_adc.h"
#include "py32f002b_ll_gpio.h"
#include "py32f002b_ll_bus.h"
#include "py32f002b_ll_utils.h"
#include <stdio.h>

#define DEAD_ZONE 20
#define PWM_PERIOD_MS 20   // 软件PWM周期20ms
#define PWM_SCALE_NG 0.5f   // 软件PWM周期20ms

__IO uint16_t AdcValue;
uint16_t mid_point = 0;
uint16_t max_range_pos = 0;
uint16_t max_range_neg = 0;

/* Private function prototypes */
static void APP_SystemClockConfig(void);
//static void APP_AdcEnable(void);
static void APP_AdcCalibrate(void);
static void APP_AdcConfig(void);
static uint32_t APP_AdcConvert(uint32_t channel);
static uint16_t APP_AdcReadMidPoint(void);
static void APP_MotorControl_SoftPWM(uint16_t adc_value);

int main(void)
{
    /* 时钟初始化 */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    APP_SystemClockConfig();

    /* LED初始化 */
    BSP_LED_Init(LED_GREEN);

    /* 按键初始化 */
    BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);

    /* USART初始化（PA4/PA3） */
    BSP_USART_Config();

    /* ADC初始化 */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
    APP_AdcConfig();
    APP_AdcCalibrate();

    /* PB0/PB1 初始化为推挽输出 */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);

    /* 上电多次采样ADC PA7 计算中点+虚位 */
    mid_point = APP_AdcReadMidPoint();
    printf("Mid point: %u\r\n", mid_point);

    while(1)
    {
        /* 读取ADC PA7 */
        AdcValue = APP_AdcConvert(LL_ADC_CHANNEL_4);
        printf("PA7 ADC: %u\r\n",(unsigned int)AdcValue);

        /* 软件PWM控制电机 */
        APP_MotorControl_SoftPWM(AdcValue);
    }
}

/* ADC读取 */
static uint32_t APP_AdcConvert(uint32_t channel)
{
    uint16_t AdcConvertValue = 0;
    if(LL_ADC_IsEnabled(ADC1))
        LL_ADC_Disable(ADC1);

    LL_ADC_REG_SetSequencerChannels(ADC1, channel);
    LL_ADC_Enable(ADC1);
    LL_mDelay(1);

    LL_ADC_REG_StartConversion(ADC1);
    while(LL_ADC_IsActiveFlag_EOC(ADC1)==0);
    LL_ADC_ClearFlag_EOC(ADC1);
    AdcConvertValue = LL_ADC_REG_ReadConversionData12(ADC1);

    LL_ADC_Disable(ADC1);
    LL_ADC_REG_SetSequencerChRem(ADC1, channel);

    return (uint32_t)AdcConvertValue;
}

/* ADC配置 */
static void APP_AdcConfig(void)
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);

    LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV4);
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE );
}

/* ADC校准 */
static void APP_AdcCalibrate(void)
{
    if(LL_ADC_IsEnabled(ADC1)==0)
    {
        LL_ADC_StartCalibration(ADC1);
        while(LL_ADC_IsCalibrationOnGoing(ADC1));
        LL_mDelay(1);
    }
}


/* 多次采样计算中点+虚位 */
static uint16_t APP_AdcReadMidPoint(void)
{
    uint32_t sum = 0;
    const uint8_t samples = 10;
    for(uint8_t i=0;i<samples;i++)
    {
        sum += APP_AdcConvert(LL_ADC_CHANNEL_4);
        LL_mDelay(5);
    }

		mid_point = (sum / samples);   // 平均 + 虚位
    if (mid_point > 4095) mid_point = 4095;

    // 一次性计算最大偏移范围
    max_range_pos = (4095 > mid_point + DEAD_ZONE) ? (4095 - mid_point - DEAD_ZONE) : 1;
    max_range_neg = (mid_point > DEAD_ZONE) ? (mid_point - DEAD_ZONE) : 1;
		
    return mid_point;
}

int16_t offset = 0;
uint32_t duty_ms = 0;

/* 软件PWM控制电机 */
#define PWM_MIN_DUTY  1
#define PWM_MAX_DUTY  (PWM_PERIOD_MS - 1)

/* 软件PWM控制函数（改进版） */
static void APP_MotorControl_SoftPWM(uint16_t adc_value)
{
    int16_t offset = adc_value - mid_point;
    uint32_t duty_ms;

    if (offset > DEAD_ZONE) // 正转
    {
        duty_ms = ((uint32_t)(offset - DEAD_ZONE) * PWM_PERIOD_MS) / max_range_pos;
        if (duty_ms > PWM_PERIOD_MS - 1) duty_ms = PWM_PERIOD_MS - 1;

        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
        LL_mDelay(duty_ms);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
        LL_mDelay(PWM_PERIOD_MS - duty_ms);
    }
    else if (offset < -DEAD_ZONE) // 反转
    {
        uint16_t abs_offset = -offset;
        duty_ms = ((uint32_t)(abs_offset - DEAD_ZONE) * PWM_PERIOD_MS * PWM_SCALE_NG) / max_range_neg;
        if (duty_ms > PWM_PERIOD_MS - 1) duty_ms = PWM_PERIOD_MS - 1;

        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
        LL_mDelay(duty_ms);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
        LL_mDelay(PWM_PERIOD_MS - duty_ms);
    }
    else
    {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
        LL_mDelay(PWM_PERIOD_MS);
    }
}



/* 系统时钟初始化 */
static void APP_SystemClockConfig(void)
{
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady()!=1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
    while(LL_RCC_GetSysClkSource()!=LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_Init1msTick(24000000);
    LL_SetSystemCoreClock(24000000);
}

/* 错误处理 */
void APP_ErrorHandler(void)
{
    while(1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while(1);
}
#endif
