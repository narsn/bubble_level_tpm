#define RED_LED_ON xAxisParams.uDutyCyclePercent = 0
#define RED_LED_OFF xAxisParams.uDutyCyclePercent = 100
#define RED_LED_BY_ANGLE xAxisParams.uDutyCyclePercent = 100 - absX
#define BLUE_LED_ON yAxisParams.uDutyCyclePercent = 0
#define BLUE_LED_OFF yAxisParams.uDutyCyclePercent = 100
#define BLUE_LED_BY_ANGLE  yAxisParams.uDutyCyclePercent = 100 - absY
#define GREEN_LED_ON LED1_ON
#define GREEN_LED_OFF LED1_OFF
