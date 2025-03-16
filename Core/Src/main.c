/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "AxelFlow.h"
#include "AxelFlow_Debug.h"
#include "AxelFlow_Serial.h"
#include "cJSON.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
// Global arrays for values (similar to dir[] and pwm[] in reference)
int dof[3] = {0, 0, 0};      // Stores DOF values (dof_4, dof_5, dof_6)
int angles[4] = {0, 0, 0, 0}; // Stores angles (angle_dof_4, angle_dof_5, angle_dof_6, angle_gripper)
int gripper = 0;             // Gripper value
int hold = 0;                // Hold value
char debugMsg[1024];

void Parse_JSON(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);

    if (!root) {
        char errorMsg[] = "Invalid JSON\n";
        CDC_Transmit_FS((uint8_t*)errorMsg, strlen(errorMsg));
        return;
    }

    // Define JSON keys
    const char *dof_keys[] = {"dof_4", "dof_5", "dof_6"};
    const char *angle_keys[] = {"angle_dof_4", "angle_dof_5", "angle_dof_6", "angle_gripper"};

    // Extract DOF values
    for (int i = 0; i < 3; i++) {
        cJSON *item = cJSON_GetObjectItem(root, dof_keys[i]);
        if (cJSON_IsNumber(item)) {
            dof[i] = item->valueint;
        }
    }

    // Extract Angle values
    for (int i = 0; i < 4; i++) {
        cJSON *item = cJSON_GetObjectItem(root, angle_keys[i]);
        if (cJSON_IsNumber(item)) {
            angles[i] = item->valueint;
        }
    }

    // Extract Gripper value
    cJSON *gripper_item = cJSON_GetObjectItem(root, "gripper");
    if (cJSON_IsNumber(gripper_item)) {
        gripper = gripper_item->valueint;
    }

    // Extract Hold value
    cJSON *hold_item = cJSON_GetObjectItem(root, "hold");
    if (cJSON_IsNumber(hold_item)) {
        hold = hold_item->valueint;
    }

    cJSON_Delete(root);

    char response[512];
    snprintf(response, sizeof(response),
        "DOFs: %d, %d, %d, %d angle : %d, %d, %d, %d\n",
        dof[0], dof[1], dof[2], angles[0],
        angles[1], angles[2], angles[3], gripper);
    CDC_Transmit_FS((uint8_t*)response, strlen(response));
}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char USB_RX_Buffer[1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	AxelFlow_debug_init(&huart2);
	UART_HandleTypeDef servo1_UART_Handle = AxelFlow_UART_Init(USART1,1000000); // Make sure that interrupt is selected.
	Servo servo1 = AxelFlow_servo_init(0x04, &servo1_UART_Handle, false);
	Servo servo2 = AxelFlow_servo_init(0x03, &servo1_UART_Handle, false);
	Servo servo3 = AxelFlow_servo_init(0x01, &servo1_UART_Handle, false);
	Servo servo4 = AxelFlow_servo_init(0x05, &servo1_UART_Handle, false);
	Servo servo5 = AxelFlow_servo_init(0x02, &servo1_UART_Handle, true);
	Servo servo6 = AxelFlow_servo_init(0x09, &servo1_UART_Handle, false);
	Servo servo7 = AxelFlow_servo_init(0x08, &servo1_UART_Handle, false);
	Status_Packet set_status;
	//	setSpeedinRPM(50, servo5);
	//	changeServoID(2, servo3);
	//	Status_Packet set_status=setMaxTorque(100, servo5);
	//	Status_Packet set_status = setCCWLimit(220, servo5);
	//	print_status(set_status, 1);
	//	HAL_Delay(2000);
	//	set_status = setCWLimit(10, servo5);
	//	print_status(set_status, 0);
	//	setWheelMode(servo1);
//	 changeBaudRate(1000000, servo3);

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		printf("Hello");
		char ch[40];
		int x;
		SyncWrite_Packet write_packet;
		Parse_JSON(USB_RX_Buffer);
		HAL_Delay(1);
		snprintf(debugMsg, sizeof(debugMsg), "Received JSON: %s\n", USB_RX_Buffer);
		CDC_Transmit_FS((uint8_t*)debugMsg, strlen(debugMsg));


		setSpeed(20, servo4);
		HAL_Delay(2000);
//		int Servo[2]= {servo7.id,3};
//		int pos[2]={100,0};
//		int speed[2]={20,200};
//		write_packet.ID= Servo;
//		write_packet.pos=pos;
//		write_packet.speed=speed;
//		Sync_write( write_packet,USART1);
//        confirm_set(120, servo6);
		setPosition(120, servo4);
//		setSpeed(20, servo6);
		HAL_Delay(1000);
//        confirm_set(180, servo6);
//        move_arm(servo4,angles[0], servo6,angles[1], servo2, angles[2], servo1,angles[3], 0 ); //FOR ARM
        //move_arm(servo4,angles[0], servo6,angles[1], servo3, angles[2], servo1,angles[3], 0 ); //FOR TESTING
		setPosition(180, servo4);
//		setSpeed(50, servo3);
		HAL_Delay(2000);
		//		status = setPunch(30.0, s'ervo1);
		//		print_status(status, 0);
		//		HAL_Delay(100);
		//		char ch[40];
		//		sprintf(ch, "%f\n\r", getPresentLoad(servo1));
		//		AxelFlow_debug_println(ch);
		//		setPosition(60, servo2);
//				set_status = setPosition(100, servo2);
		//		print_status(set_status, 0);
		//
//				HAL_Delay(2000);
//                setPosition(80, servo6);
		//		print_status(set_status, 1);
//				setPosition(180, servo3);
//		        HAL_Delay(4000);
//        move_arm(servo4,120, servo6,100, servo2, 30, servo1,180, 1 );
		//		setPosition(0, servo3);
		//		set_status = setPosition(90, servo6);
		//		x = getPositionAngle(servo1);
		//		sprintf(ch, "%f\n\r", x);
		//		AxelFlow_debug_println(ch);
		//		print_status(set_status, 0);
//				HAL_Delay(2000);
//				x =scanID(servo3);
		//	    x = getPositionAngle(servo5);
//				sprintf(ch, "%f\n\r", x);
//				AxelFlow_debug_println(ch);
		//	    sprintf(ch, "%f\n\r", x);
		//	    AxelFlow_debug_println(ch);
		//		grip(130, servo1);
		//		setPosition(130, servo6);
		//		x = getPositionAngle(servo1);
		//		sprintf(ch, "%f\n\r", x);
		//		AxelFlow_debug_println(ch);
//				setSpeed(20, servo4);
//				HAL_Delay(2000);
//			    setPosition(300, servo4);
//			    HAL_Delay(2000);
//		        confirm_set(10, servo2);
//		        HAL_Delay(2000);
//		        confirm_set(40, servo2);
//		        HAL_Delay(2000);
//				setPosition(0, servo4);
//			    setPosition(180, servo6);
		//		print_status(set_status, 0);
//				HAL_Delay(4000);
//				set_status = setPosition(120, servo2);
//				HAL_Delay(2000);
		//		print_status(set_status, 0);
		//		HAL_Delay(2000);
		//		setSpeed(10, servo2);
//				grip(137, servo1);

//				HAL_Delay(2000);
		//		setSpeed(30, servo5);
//			    x = getPositionAngle(servo6);
//				char ch1[20] = "hello";
//				sprintf(ch, "%u\n", x);
//				HAL_UART_Transmit(&huart2, ch, strlen(ch),HAL_MAX_DELAY);
//				AxelFlow_debug_println(ch1);
//				HAL_Delay(4000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
