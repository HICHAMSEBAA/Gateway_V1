/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MY_NRF24.h"
#include "RGB.h"
#include "stdio.h"
#include <stdbool.h> // Include for boolean type
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// commands
#define LED_ON "LED_ON"    // Command to turn LED on
#define LED_OFF "LED_OFF"  // Command to turn LED off
#define POWER_1 "P_1"      // Power level 1
#define POWER_2 "P_2"      // Power level 2
#define POWER_3 "P_3"      // Power level 3
// ---------------------------------------------------------//

// -----------------------------------------------------------------------//

char time[10];             // Array to store time
char date[10];             // Array to store date

#define TX_MODE 1          // Transmission mode
#define RX_MODE 0          // Reception mode
#define DATARATE RF24_250KBPS  // Data rate setting for RF24 module

// Define the transmission pipe address
uint64_t Addr = 0x11223344AA;  // Address for RF24 communication

// Define an array to store transmission data with an initial value "SEND"
uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;  // Variables for humidity and temperature
float tCelsius, tFahrenheit, RH;          // Temperature and humidity values
uint32_t Value = 0;                        // Placeholder for values
uint8_t R, G, B;                           // Variables for RGB LED control
uint8_t payload[32];                       // Payload for transmission

// Define an array to store acknowledgment payload
char AckPayload[32];                       // Acknowledgment payload buffer

// Define an array to store received data
char myRxData[32];                          // Buffer for received data

// Define an acknowledgment payload with an initial value "Ack by Master !!"
char myAckPayload[32] = "Ack by Node 0";   // Acknowledgment payload

int count = 0;  // Counter variable

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to configure NRF24 module for transmit mode without acknowledgment
// This function configures the NRF24 module for either transmit or receive mode without acknowledgment, 
// depending on the value of the parameter 'transmit_mode'.

void nrf24_config_mode(bool transmit_mode) {
	// Print information about entering transmit mode without acknowledgment
	//printf("________________________Tx Mode________________________ \n\r");

	if (transmit_mode) {
		// Stop listening for incoming data
		NRF24_stopListening();

		// Set writing pipe address to TxpipeAddrs
		NRF24_openWritingPipe(Addr);
	} else {
		// Start listening for incoming data
		NRF24_startListening();
		// Open reading pipe with address RxpipeAddrs
		NRF24_openReadingPipe(1, Addr);
	}

	// Enable auto acknowledgment
	NRF24_setAutoAck(true);

	// Set channel to 52
	NRF24_setChannel(52);

	// Set payload size to 32 bytes
	NRF24_setPayloadSize(32);

	// Enable dynamic payloads
	NRF24_enableDynamicPayloads();

	// Enable acknowledgment payloads
	NRF24_enableAckPayload();
}

// Function to simulate payload charge
// This function populates the provided payload array with simulated sensor data.
// It returns true if the payload is successfully populated, otherwise false.

bool Payload_charge_simulate(uint8_t *payload, uint8_t len) {

	uint8_t test_bit = 0; // Variable to track which sensor data is populated in the payload

	// Check for valid payload pointer and length
	if (payload == NULL || len < 11) {
		return false; // Indicate error (invalid payload or insufficient size)
	}

	// Directly access payload elements without unnecessary array indexing
	if (dht22_readings()) {
		// Copy sensor data directly
		payload[1] = RH1; // Relative humidity MSB
		payload[2] = RH2; // Relative humidity LSB
		payload[3] = TC1; // Temperature Celsius MSB
		payload[4] = TC2; // Temperature Celsius LSB
		test_bit = test_bit + 1; // Set corresponding test bit
	}

	if (HAL_ADC_Start(&hadc1) == HAL_OK) {
		HAL_ADC_PollForConversion(&hadc1, 100); // Poll ADC for conversion
		Value = HAL_ADC_GetValue(&hadc1); // Get ADC value
		payload[5] = Value; // Store ADC value in payload
		test_bit = test_bit + 2; // Set corresponding test bit
	}

	if (getRGB(&R, &G, &B) != 0) {
		payload[6] = R; // Red value
		payload[7] = G; // Green value
		payload[8] = B; // Blue value
		test_bit = test_bit + 3; // Set corresponding test bit
	}

	payload[0] = test_bit; // Set the test bit in the first byte of payload

	return true; // Indicate successful payload population
}


// Function to send data
// This function configures the NRF24 module for transmit mode, attempts to send data, and waits for acknowledgment.
// It returns true if the data is successfully transmitted and acknowledged, otherwise false.

bool Send_Data(void) {
	nrf24_config_mode(TX_MODE); // Configure NRF24 module for transmit mode

	// Variable to track the status of data transmission
	bool send_stat = false;

	// Loop indefinitely until acknowledgment is sent or a timeout occurs
	int coun = 0; // Counter for attempts
	while (coun < 5) { // Try for maximum 5 times
		printf("Waiting for acknowledgement... (Attempt %d)\n", coun); // Display attempt number
		HAL_Delay(10); // Wait for a short period

		// Attempt to write data to NRF24 module
		if (NRF24_write(payload, 32)) { // If data is successfully written
			// Read acknowledgment payload
			NRF24_read(AckPayload, 32); // Read acknowledgment payload
			if (strlen(AckPayload) != 0) { // If acknowledgment payload is received
				printf("Master acknowledgement : %s \r\n", AckPayload); // Display acknowledgment
				send_stat = true; // Set send status to true
				break; // Exit loop
			}
		}

		// If acknowledgment transmission fails, wait for a short period before retrying
		coun++; // Increment attempt counter
	}

	// Return the status of data transmission
	return send_stat; // Return send status
}


// Function to analyze the payload received
// This function analyzes the received payload and performs corresponding actions based on the command.
// It returns 1 to indicate that payload analysis is complete.

uint8_t AnalyseThePayload(uint8_t *rxPayload) {
	// Check the first byte of the payload for an error flag (assuming 0 indicates error)
	if (strcmp(rxPayload, LED_ON) == 0) { // If the payload is LED_ON command
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET); // Turn on LED
	} else if (strcmp(rxPayload, LED_OFF) == 0) { // If the payload is LED_OFF command
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET); // Turn off LED
	} else if (strcmp(rxPayload, POWER_1) == 0) { // If the payload is POWER_1 command
		NRF24_setPALevel(RF24_PA_m18dB); // Set power level to -18dBm
	} else if (strcmp(rxPayload, POWER_2) == 0) { // If the payload is POWER_2 command
		NRF24_setPALevel(RF24_PA_m6dB); // Set power level to -6dBm
	} else if (strcmp(rxPayload, POWER_3) == 0) { // If the payload is POWER_3 command
		NRF24_setPALevel(RF24_PA_0dB); // Set power level to 0dBm
	} else { // If the command is not recognized
		printf("No Command from the master \n"); // Print message indicating no recognized command
	}
	return 1; // Return 1 to indicate payload analysis complete
}


// Function to receive data
// This function configures the NRF24 module for receive mode and waits for incoming data.
// It returns true if data is successfully received, otherwise false.

bool Receive_Data(void) {
	nrf24_config_mode(RX_MODE); // Configure NRF24 module for receive mode

	// Variable to track the status of data reception
	bool receive_stat = false;

	while (1) { // Loop indefinitely
		// Check if there is data available to read
		if (NRF24_available()) { // If data is available
			// Read data from NRF24 module
			NRF24_read(myRxData, 32); // Read data

			// Send acknowledgment payload to Node 1
			NRF24_writeAckPayload(1, myAckPayload, 32); // Send acknowledgment payload

			// Print the received data
			count++; // Increment counter for received data

			// Set receive_stat to true to indicate successful data reception
			receive_stat = true; // Set receive status to true
			break; // Exit loop
		}
	}

	// Return the status of data reception
	return receive_stat; // Return receive status
}


// Function to setup the system
// This function initializes the NRF24 module, configures it for receive mode, and prints current radio settings.

void setup(void) {
	// Initialize NRF24 module
	NRF24_Init(); // Initialize NRF24 module

	nrf24_config_mode(RX_MODE); // Configure NRF24 module for receive mode

	// Print information about entering receive mode with acknowledgment
	printf("________________________Engaging communication channels...________________________ \n\r");

	// Print current radio settings
	printRadioSettings(); // Print current radio settings
}


// Function to continuously execute main program logic
// This function repeatedly simulates payload charging, receives data, sends acknowledgment, analyzes payload, and introduces delay.

void loop(void) {
	Payload_charge_simulate(payload, 32); // Simulate payload charging

	// Check if data is received successfully
	if (Receive_Data()) { // If data is received
		// Print received payload data
		printf("Payload : %d | %d | %d | %d | %d | %d | %d | %d | %d | \n\r", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7], payload[8]);
		// Check if acknowledgment is sent successfully
		Send_Data(); // Send acknowledgment
	}

	// Analyze the received payload
	AnalyseThePayload(myRxData); // Analyze received payload

	HAL_Delay(200); // Introduce delay
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim6);
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2) {
		set_time();
	}
	/*
	 * Description: This block of code initializes the NRF24 module, enters receive mode,
	 *              and prints current radio settings.
	 */
	setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		loop();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
