/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Font.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX 1400
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spi_byte;
char gsm_rx_buffer[MAX];
char gsm_rx_buffer2[MAX];

uint8_t gsm_tx_buffer[512];
char significant_data_buffer[MAX];
volatile char gsm_incoming_byte;
volatile uint16_t gsm_index = 0;
volatile uint8_t prayer_times_are_parsed = 0;
volatile uint8_t comma_count = 0;

volatile uint8_t flag_OK_Status = 0;
volatile uint8_t flag_HTTP_Status_200 = 0;
volatile uint8_t flag_HTTP_Get_New_Data = 0;
volatile uint8_t flag_HTTP_Unread_Data = 0;




typedef struct
{
	char imsak[10];
	char gunes[10];
	char ogle[10];
	char ikindi[10];
	char aksam[10];
	char yatsi[10];

}prayer_times_t;


prayer_times_t prayer_times;

void parse_significant_data(char *target, char *source)
{
	while(comma_count <= 8)
	{
		if(*source == ',')
		{
			comma_count ++;
		}

		*target = *source;
		source ++;
		target ++;
	}
	*target = '\0';
}


void parse_prayer_times()
{

	char *token = strtok(significant_data_buffer, ",");
	//puts(token);
	sscanf(token, "%*[^:]:\"%[^\"]", prayer_times.imsak);

	token = strtok(NULL, ",");
	sscanf(token, "%*[^:]:\"%[^\"]", prayer_times.gunes);
	//puts(token);

	token = strtok(NULL, ",");
	sscanf(token, "%*[^:]:\"%[^\"]", prayer_times.ogle);
	//puts(token);

	token = strtok(NULL, ",");
	sscanf(token, "%*[^:]:\"%[^\"]", prayer_times.ikindi);
	//puts(token);

	token = strtok(NULL, ",");
	sscanf(token, "%*[^:]:\"%[^\"]", prayer_times.aksam);
	//puts(token);

	token = strtok(NULL, ",");	// PASS - Repeated data

	token = strtok(NULL, ",");
	sscanf(token, "%*[^:]:\"%[^\"]", prayer_times.yatsi);
	//puts(token);
}


uint8_t gsm_end_of_data(unsigned char data) // detect "end of line" from uart
{
	volatile static uint8_t cr_flag = 0;
	volatile static uint8_t data_step = 0;

	data_step++;

	if(data == '\r')
	{
		cr_flag = 1;
		data_step = 1;
	}

	if( (data_step == 2) && (cr_flag = 1) && (data == '\n')) //CRLF came and there is nothing between them
	{
		data_step = 0;
		cr_flag = 0;
		return 1;
	}


	else
	{
		return 0;
	}
}

void sim800_init()
{
	HAL_Delay(1000);

	uint8_t attempts_count = 0;
	uint8_t dont_continue = 0;




	flag_OK_Status = 0;
	if (dont_continue != 1) 
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "ATE1");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) 
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;


	flag_OK_Status = 0;
	if (dont_continue != 1)
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+CMGF=1");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10)
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;

	flag_OK_Status = 0;
	if (dont_continue != 1)
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+CNMI=1,2,0,0,0");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) 
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;


	/*
	snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+CLIP=1");
	HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
	 */

	HAL_Delay(300);

	flag_OK_Status = 0;
	if (dont_continue != 1) 
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+SAPBR=3,1,\"Contype\", \"GPRS\"");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) 
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;


	flag_OK_Status = 0;
	if (dont_continue != 1) {
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+SAPBR=3,1,\"APN\",\"internet\"");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) 
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;



	HAL_Delay(300);

	snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+SAPBR=1,1");
	HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
	flag_OK_Status = 0;


	HAL_Delay(300);

	flag_OK_Status = 0;
	if (dont_continue != 1) 
	{
		snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+SAPBR=2,1"); //Show Ip
		HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);

		do {
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 50) 
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;

	if (dont_continue == 1)
	{
		// Some error messages
	}


}

void prayer_time_request()
{

	uint8_t attempts_count = 0;
	uint8_t dont_continue = 0;

	snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+HTTPTERM\r\n");
	HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
	HAL_Delay(100);


	flag_OK_Status = 0;
	if (dont_continue != 1) 
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+HTTPINIT");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) 
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;

	HAL_Delay(300);


	flag_OK_Status = 0;
	if (dont_continue != 1) 
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+HTTPPARA=\"CID\",1");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) 
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;


	HAL_Delay(300);

	flag_OK_Status = 0;
	if (dont_continue != 1)
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+HTTPPARA=\"URL\",\"http://api.aladhan.com/v1/timingsByCity?city=Kahramanmaras&country=Turkey&method=13&school=1\"");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10)
			{
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;


	flag_OK_Status = 0;
	if (dont_continue != 1)
	{
		flag_HTTP_Status_200 = 0;
		snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s", "AT+HTTPACTION=0\r\n");
		HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
		HAL_Delay(1000);
		flag_OK_Status = 0;

		do {
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) {
				dont_continue = 1;
				break;
			}
		} while (flag_HTTP_Status_200 == 0);
	}
	attempts_count = 0;

	flag_OK_Status = 0;
	if (dont_continue != 1)
	{
		HAL_Delay(1000);
		snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s", "AT+HTTPREAD\r\n");
		HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
		do {
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 100) {
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;



	flag_OK_Status = 0;
	if (dont_continue != 1)
	{
		do {
			snprintf((char *)gsm_tx_buffer, sizeof(gsm_tx_buffer), "%s\r\n", "AT+HTTPTERM");
			HAL_UART_Transmit(&huart3, gsm_tx_buffer, strlen((const char *)gsm_tx_buffer), HAL_MAX_DELAY);
			HAL_Delay(1000);
			attempts_count++;
			if (attempts_count >= 10) {
				dont_continue = 1;
				break;
			}
		} while (flag_OK_Status == 0);
	}
	attempts_count = 0;



	if (dont_continue == 1)
	{
		// HTTP Request Error
	}


}

char incoming_byte;
char mystring[15] = "Fajr";

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)	//GSM
	{
		incoming_byte = gsm_incoming_byte;

		if(gsm_index == MAX-1)
		{
			gsm_index = 0;
		}


		if(gsm_end_of_data(incoming_byte))
		{


			if (flag_HTTP_Get_New_Data == 1)
			{
				char *p = strstr((char *)gsm_rx_buffer, mystring);
				if(p) // Detect the keyword
				{
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
					parse_significant_data(significant_data_buffer, p);
					parse_prayer_times();
					gsm_index = 0;
					prayer_times_are_parsed = 1;
					memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
				}
				flag_HTTP_Get_New_Data = 0;
				flag_HTTP_Unread_Data = 1;
			}


			if (strstr((char *)gsm_rx_buffer, "OK"))
			{
				flag_OK_Status = 1;
				gsm_index = 0;
				memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
			}

			if (strstr((char *)gsm_rx_buffer, "HTTPACTION: 0,200")) {
				flag_HTTP_Status_200 = 1;
				gsm_index = 0;
				memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
			}

			if (strstr((char *)gsm_rx_buffer, "+HTTPREAD:")) {
				flag_HTTP_Get_New_Data = 1;
				gsm_index = 0;
				memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
			}

			if(strstr((char *)gsm_rx_buffer, "kapali")) // There is a new message
			{
				gsm_index = 0;
				memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

			}

			gsm_index = 0;

		}

		else // save uart byte to the buffer
		{
			gsm_rx_buffer[gsm_index] = incoming_byte;
			gsm_index ++;
		}

	}


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
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_USART3_UART_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Receive_DMA(&huart3, &gsm_incoming_byte, 1); // UART GSM Receive enable

	sim800_init();
	prayer_time_request();


	char lcd_buffer[15];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // CE High
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // RST High


	void lcd_command(uint8_t data)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // CE Low
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // DC Low
		HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // CE High
	}

	void lcd_clear()
	{
		int i;
		uint8_t data = 0x00;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // CE Low
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // DC High

		for (i = 0; i < 504; i++)
		{
			HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // DC Low
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // CE High
	}

	void lcd_reset()
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // RST Low
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // RST High
	}

	void lcd_write(char *data)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // DC High
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // CE Low
		uint8_t data_lenght = strlen(data);
		for(uint8_t char_order = 0; char_order < data_lenght; char_order ++)
		{
			for(uint8_t pixel_column = 0; pixel_column < 5; pixel_column ++)
			{
				HAL_SPI_Transmit(&hspi1, (uint8_t *)&(ASCII[data[char_order]-0x20][pixel_column]), 1, HAL_MAX_DELAY);
			}
			spi_byte = 0x00;
			HAL_SPI_Transmit(&hspi1, &spi_byte, 1, HAL_MAX_DELAY);


		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // CE High
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // DC Low
	}

	void lcd_set_colomn_row(char x, char y)
	{
		lcd_command((x |= (1 << 7)));
		lcd_command((y |= (1 << 6)));
	}

	void lcd_init()
	{
		lcd_reset();
		HAL_Delay(100);
		lcd_command(0x21);  /* command set in addition mode */
		//100001
		lcd_command(0xC0);  /* set the voltage by sending C0 means VOP = 5V */
		//11000000
		lcd_command(0x07);  /* set the temp. coefficient to 3 */
		//111
		lcd_command(0x13);  /* set value of Voltage Bias System */
		//10011    bias n4
		lcd_command(0x20);  /* command set in basic mode */
		//100000
		lcd_command(0x0C);  /* display result in normal mode */
		//1100
	}

	void lcd_write_int(uint16_t number)
	{
		sprintf(lcd_buffer, "%d", number);
		lcd_write(lcd_buffer);
	}

	void lcd_write_float(float number)
	{
		sprintf(lcd_buffer, "%.2f", number);
		lcd_write(lcd_buffer);
	}

	lcd_init();
	lcd_clear();
	lcd_set_colomn_row(0, 0);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if(prayer_times_are_parsed == 1)
		{
			prayer_times_are_parsed = 0;
			lcd_clear();
			lcd_set_colomn_row(0, 0);
			lcd_write("Imsak ->");
			lcd_set_colomn_row(50, 0);
			lcd_write(prayer_times.imsak);

			lcd_set_colomn_row(0, 1);
			lcd_write("Gunes ->");
			lcd_set_colomn_row(50, 1);

			lcd_write(prayer_times.gunes);

			lcd_set_colomn_row(0, 2);
			lcd_write("Ogle ->");
			lcd_set_colomn_row(50, 2);

			lcd_write(prayer_times.ogle);

			lcd_set_colomn_row(0, 3);
			lcd_write("Ikindi->");
			lcd_set_colomn_row(50, 3);

			lcd_write(prayer_times.ikindi);

			lcd_set_colomn_row(0, 4);
			lcd_write("Aksam ->");
			lcd_set_colomn_row(50, 4);

			lcd_write(prayer_times.aksam);

			lcd_set_colomn_row(0, 5);
			lcd_write("Yatsi ->");
			lcd_set_colomn_row(50, 5);
			lcd_write(prayer_times.yatsi);
		}
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_Delay(1000);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pins : LCD_RST_Pin LCD_CE_Pin LCD_DC_Pin */
	GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CE_Pin|LCD_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
	while (1)
	{
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
