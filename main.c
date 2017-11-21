///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
#include <string.h>
// SDK Included Files
#include "board.h"
#include "fsl_tpm_driver.h"
#include "fsl_debug_console.h"
#include "accel.h"
#include "main.h"
#include "gpio_pins.h"

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

volatile bool isButtonPress = FALSE;
int alert = FALSE;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// Method to handle SW pressing
void BOARD_SW_IRQ_HANDLER(void)
{
	GPIO_DRV_ClearPinIntFlag(BOARD_SW_GPIO);
	isButtonPress = TRUE;
}

// Read the data from the sensor
void getAccelData(accel_sensor_data_t *accelData, int *x, int *y)
{
	*x = (int)((int16_t)((accelData->data.accelXMSB << 8) | accelData->data.accelXLSB) * 0.011);
	*y = (int)((int16_t)((accelData->data.accelYMSB << 8) | accelData->data.accelYLSB) * 0.011);
}

// show the message in normal behavior
void showNormalBehavior(int *absX, int *absY, int *x, int *y)
{
	PRINTF("Roulis %d degre %s Tanguage %d degre %s\r\n",
		   *absY,
		   *y < 0 ? "D" : "G",
		   *absX,
		   *x < 0 ? "H" : "B")
}

int main(void)
{
	tpm_general_config_t driverInfo;
	accel_dev_t accDev;
	accel_dev_interface_t accDevice;
	accel_sensor_data_t accelData;
	accel_i2c_interface_t i2cInterface;
	tpm_pwm_param_t yAxisParams;
	tpm_pwm_param_t xAxisParams;
	int x, y, absX, absY;

	// Message variables
	int taguage, valeur, dir;
	char **directions = {{"haut", "bas"}, {"droite", "gauche"}};

	// Define gpio input pin config structure SW.
	gpio_input_pin_user_config_t inputPin[] = {{
		.pinName = BOARD_SW_GPIO,
		.config.isPullEnable = true,
#if FSL_FEATURE_PORT_HAS_PULL_SELECTION
		.config.pullSelect = kPortPullUp,
#endif
#if FSL_FEATURE_PORT_HAS_PASSIVE_FILTER
		.config.isPassiveFilterEnabled = false,
#endif
#if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
		.config.isDigitalFilterEnabled = false,
#endif
		.config.interrupt = kPortIntFallingEdge,
		},
		{
			.pinName = GPIO_PINS_OUT_OF_RANGE,
		}};

	xAxisParams.mode = kTpmEdgeAlignedPWM;
	xAxisParams.edgeMode = kTpmHighTrue;
	xAxisParams.uFrequencyHZ = 100000u;
	xAxisParams.uDutyCyclePercent = 0u;

	yAxisParams.mode = kTpmEdgeAlignedPWM;
	yAxisParams.edgeMode = kTpmHighTrue;
	yAxisParams.uFrequencyHZ = 100000u;
	yAxisParams.uDutyCyclePercent = 0u;

	// Register callback func for I2C
	i2cInterface.i2c_init = I2C_DRV_MasterInit;
	i2cInterface.i2c_read = I2C_DRV_MasterReceiveDataBlocking;
	i2cInterface.i2c_write = I2C_DRV_MasterSendDataBlocking;

	accDev.i2c = &i2cInterface;
	accDev.accel = &accDevice;

	accDev.slave.baudRate_kbps = BOARD_ACCEL_BAUDRATE;
	accDev.slave.address = BOARD_ACCEL_ADDR;
	accDev.bus = BOARD_ACCEL_I2C_INSTANCE;

	hardware_init();

	// Accel device driver utilizes the OSA, so initialize it.
	OSA_Init();

	accel_init(&accDev);

	// Prepare memory for initialization.
	memset(&driverInfo, 0, sizeof(driverInfo));

	// Init TPM.
	TPM_DRV_Init(BOARD_BUBBLE_TPM_INSTANCE, &driverInfo);

	// Set clock for TPM.
	TPM_DRV_SetClock(BOARD_BUBBLE_TPM_INSTANCE, kTpmClockSourceModuleClk,
					 kTpmDividedBy2);

	LED1_EN;
	GPIO_DRV_InputPinInit(inputPin);

	alert = FALSE;
	// Main loop.  Get sensor data and update duty cycle for the TPM timer.
	while (TRUE)
	{
		OSA_TimeDelay(5);
		READ_SENSOR_DATA;

		getAccelData(&accelData, &x, &y);
		absX = abs(x);
		absY = abs(y);

		if ((absX <= 2 && absY <= 2) && !alert)
		{
			// Plane has no angle
			alert = FALSE;
			RED_LED_ON;
			BLUE_LED_ON;
			GREEN_LED_ON;
		}
		else if ((absX >= 45 || absY >= 45) || alert)
		{
			// Plane has > 45 deg in any way
			alert = TRUE;
			GREEN_LED_ON;
			RED_LED_OFF;
			BLUE_LED_OFF;
		}
		else if (!alert)
		{
			// normal behavior
			alert = FALSE;
			GREEN_LED_OFF;
			RED_LED_BY_ANGLE;
			BLUE_LED_BY_ANGLE;
		}

		TPM_DRV_PwmStart(BOARD_BUBBLE_TPM_INSTANCE, &xAxisParams, BOARD_TPM_X_CHANNEL);
		TPM_DRV_PwmStart(BOARD_BUBBLE_TPM_INSTANCE, &yAxisParams, BOARD_TPM_Y_CHANNEL);

		if (absX < 45 && absY < 45)
		{
			showNormalBehavior(&absX, &absY, &x, &y);
		}
		else
		{
			taguage = absX >= 45;
			valeur = taguage ? x : y;
			dir = valeur < 0 ? 0 : 1;

			PRINTF("Alerte %s : plus de 45, %s %s a %d degres\r\n",
				   taguage ? "taguage" : "roulis",
				   taguage ? "nez" : "inclinaison",
				   directions[taguage][dir],
				   abs(valeur));
		}

		if (isButtonPress)
		{
			isButtonPress = FALSE;
			alert = FALSE;
		}
	}
}
