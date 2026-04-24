/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include "vl53l0x_api.h" // time of flight sensor library

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// positions structure - specifies x and y for block positions
typedef struct {
    float x;
    float z;
} Position;

// holds identity of two blocks that need to be swapped
typedef struct {
    int pos1;
    int pos2;
} SwapStep;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Define algorithms
#define BUBBLE_SORT 0
#define INSERTION_SORT 1
#define QUICK_SORT 2

// PIN DEFINITIONS //
// Motors
// x direction motor
#define MOTORX_DIR_PORT GPIOA
#define MOTORX_DIR_PIN  GPIO_PIN_8
#define MOTORX_STEP_PORT GPIOA
#define MOTORX_STEP_PIN GPIO_PIN_9

// z direction motors
#define MOTORZ1_DIR_PORT GPIOC
#define MOTORZ1_DIR_PIN  GPIO_PIN_7
#define MOTORZ1_STEP_PORT GPIOB
#define MOTORZ1_STEP_PIN GPIO_PIN_6

#define MOTORZ2_DIR_PORT GPIOB
#define MOTORZ2_DIR_PIN  GPIO_PIN_10
#define MOTORZ2_STEP_PORT GPIOB
#define MOTORZ2_STEP_PIN GPIO_PIN_4

// Buttons
#define BUBBLE_BUTTON SW_3_Pin
#define INSERTION_BUTTON SW_2_Pin

// LEDs
#define BUBBLE_LED_PORT GPIOC
#define BUBBLE_LED_PIN LED_1_Pin
#define INSERTION_LED_PORT LED_3_GPIO_Port
#define INSERTION_LED_PIN LED_3_Pin

// motor-related
#define STEPS_PER_CM_X 66.67f // experimental value
#define STEPS_PER_CM_Z (66.67f * 4.3f)  // experimental value

// direction inputs for moving motors
#define DIR_RIGHT  1
#define DIR_LEFT 0
#define DIR_DOWN 1
#define DIR_UP 0

// motors to move (defined as ints to use a switch statement in moveStepper)
#define MOTOR_X 1
#define MOTOR_Z 2

// gripper instruction (defines as ints to use a switch statement in gripperControl)
#define OPEN_GRIPPER 0
#define CLOSE_GRIPPER 1

// Z heights (cm)
#define Z_MIN_HEIGHT 10.0f // min height - used for clamping so that gripper never goes lower
#define Z_SAFE_HEIGHT 13.0f   // used for z homing
#define Z_CARRY_HEIGHT 20.0f // height to carry the blocks at to clear the other blocks
#define Z_PICK_HEIGHT 10.5f    // height to grab blocks at (we drop blocks at .5 cm higher so the taller blocks don't push against the bed)
#define Z_MEASURE_BLOCKS_HEIGHT 12.0f // height for measured block heights with time of flight sensor (much higher and it becomes innacurate, lower and it hits the tallest block)

// x spacing (between blocks)
#define POSITION_SPACING_CM 3.5f  // horizontal spacing between blocks

// Buffer space position
#define BUFFER_POS_INDEX 5 // would have to change if we use more that 5 blocks

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile bool xLimitTriggered = false; // set to true in the interrupt function that's triggered by the limit switch

Position currentPosition = {0.0f, Z_MIN_HEIGHT}; // updated to keep track of where the gripper is (we use for global coordinates)

SwapStep swapList[36]; // list of swaps to run

int blocks_heights[5]; // array of block order (defined based on time of flight sensor readings and used for sorting algorithms)

// position of the block spaces
Position positions[6] = {
    {0.0f, Z_CARRY_HEIGHT},
    {POSITION_SPACING_CM, Z_CARRY_HEIGHT},
    {2 * POSITION_SPACING_CM, Z_CARRY_HEIGHT},
    {3 * POSITION_SPACING_CM, Z_CARRY_HEIGHT},
    {4 * POSITION_SPACING_CM, Z_CARRY_HEIGHT},
    {5 * POSITION_SPACING_CM, Z_CARRY_HEIGHT}
};

// time of flight sensor variables
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c;
VL53L0X_DEV Dev = &vl53l0x_c;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// initializing all the functions
// Low-level motor control
bool moveStepper(float dist_cm, int dir, int motorMovement);

// Gripper / servo
void gripperControl(int gripInstruction);
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle);

//startup functions
void homeZ(float targetDistance); // using time of flight sensor
void homeX(); // using limit switch interrupt
float getAverageDistanceMm(int samples); // get measured value from time of flight sensor
void measureBlocks(); // defines block_heights array from time of flight sensor readings

// movement control
bool moveGripperTo(Position target); // moving gripper to target position
bool moveGripper_over(Position target); // moving gripper to position that

// High-level
void pickBlock(Position pos);
void placeBlock(Position pos);
void swapBlocks_withBuffer(int posA, int posB);
void swapBlocks_moveIntoPosition(int fromPos, int toPos); // SWAP FUNC 2.0

// bubble sort & insertion sort functions
void executeSwapList(int numSwaps, int algorithm);
int sortingAlgorithm(int algorithm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// set up the lidar sensor
void LidarInit() {
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	VL53L0X_WaitDeviceBooted( Dev );
	VL53L0X_DataInit( Dev );
	VL53L0X_StaticInit( Dev );
	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Start the PWM signal generation
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  Dev->I2cHandle = &hi2c1;
  Dev->I2cDevAddr = 0x52;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(20);
  LidarInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  gripperControl(OPEN_GRIPPER);

  homeX();
  homeZ(Z_SAFE_HEIGHT);

  bool runSort = false;
  int algorithm = 2; //

  while (1)
  {
	  // Selected sorting algorithm using buttons (LED associated with button pressed lights up)
	    GPIO_PinState bubble    = HAL_GPIO_ReadPin(GPIOB, BUBBLE_BUTTON);
	    GPIO_PinState insertion = HAL_GPIO_ReadPin(GPIOB, INSERTION_BUTTON);


	    if (bubble == GPIO_PIN_RESET)
	    {
	        HAL_GPIO_WritePin(BUBBLE_LED_PORT, BUBBLE_LED_PIN, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(INSERTION_LED_PORT, INSERTION_LED_PIN, GPIO_PIN_RESET);

	        algorithm = 0;
	        runSort = true;
	    }

	    if (insertion == GPIO_PIN_RESET)
	    {
	        HAL_GPIO_WritePin(BUBBLE_LED_PORT, BUBBLE_LED_PIN, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(INSERTION_LED_PORT, INSERTION_LED_PIN, GPIO_PIN_SET);

	        algorithm = 1;
	        runSort = true;
	    }

	    // after selection of sorting algorithms, enters this if statement that calculates the sorting algorithm and executes it
	    if (runSort)
	    {
		  runSort = false; // prevents code from running multiple times
		  measureBlocks(); // Measure the blocks
		  int numSwaps = sortingAlgorithm(algorithm); // record required swaps to do the selected sortng algorithm

		  executeSwapList(numSwaps, algorithm); // runs the sorting algorithm

	    }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8400-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_1_Pin */
  GPIO_InitStruct.Pin = SW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT_SWITCH_Pin */
  GPIO_InitStruct.Pin = LIMIT_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIMIT_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_2_Pin SW_3_Pin */
  GPIO_InitStruct.Pin = SW_2_Pin|SW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 LED_1_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_7|LED_1_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// LOW LEVEL MOTOR CONTROL //
bool moveStepper(float dist_cm, int dir, int motorMovement)
{
	// dist_cm is how far the motor should move the end effector
	// dir is the direction the motor will move
	// motorMovement is which motor(s) will be moved (1 for X movement, 2 for Z movement)

    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
    GPIO_TypeDef* STEP_PORT;
    uint16_t STEP_PIN;

    GPIO_TypeDef* DIR_PORT_2 = NULL;
    uint16_t DIR_PIN_2 = 0;
    GPIO_TypeDef* STEP_PORT_2 = NULL;
    uint16_t STEP_PIN_2 = 0;

    bool moveSecondMotor = false;
    float steps_per_cm;

    switch(motorMovement)
    {
        case MOTOR_X:
            DIR_PORT  = MOTORX_DIR_PORT;
            DIR_PIN   = MOTORX_DIR_PIN;
            STEP_PORT = MOTORX_STEP_PORT;
            STEP_PIN  = MOTORX_STEP_PIN;

            steps_per_cm = STEPS_PER_CM_X;

            break;

        case MOTOR_Z:
            DIR_PORT  = MOTORZ1_DIR_PORT;
            DIR_PIN   = MOTORZ1_DIR_PIN;
            STEP_PORT = MOTORZ1_STEP_PORT;
            STEP_PIN  = MOTORZ1_STEP_PIN;

            DIR_PORT_2  = MOTORZ2_DIR_PORT;
            DIR_PIN_2   = MOTORZ2_DIR_PIN;
            STEP_PORT_2 = MOTORZ2_STEP_PORT;
            STEP_PIN_2  = MOTORZ2_STEP_PIN;

            steps_per_cm = STEPS_PER_CM_Z;

            moveSecondMotor = true;
            break;

        default:
            return false;
    }

    int steps = (int)(dist_cm * steps_per_cm);

    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, (dir ? GPIO_PIN_SET : GPIO_PIN_RESET));

    if (moveSecondMotor) // true for moving in Z direction
    {
    	HAL_GPIO_WritePin(DIR_PORT_2, DIR_PIN_2, (dir ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }
    HAL_Delay(1);

	for(int i = 0; i < steps; i++)
	{
		// making manual PWM
		// BOTH HIGH
		HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
		if (moveSecondMotor)
			HAL_GPIO_WritePin(STEP_PORT_2, STEP_PIN_2, GPIO_PIN_SET);

		HAL_Delay(1);

		// BOTH LOW
		HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
		if (moveSecondMotor)
			HAL_GPIO_WritePin(STEP_PORT_2, STEP_PIN_2, GPIO_PIN_RESET);

		HAL_Delay(1);
	}


    return true;
}


// GRIPPER & SERVO CONTROL //
void gripperControl(int gripInstruction) {
	switch(gripInstruction)
	{
	case OPEN_GRIPPER:
		// instruction to open gripper
		  for (uint8_t angle = 180; angle > 0; angle -= 10) // open claw
		  {
			  Set_Servo_Angle(&htim2, TIM_CHANNEL_1, angle);
			  HAL_Delay(100);
		  }
		break;

	case CLOSE_GRIPPER:
		// instruction to close gripper
		  for (uint8_t angle = 0; angle <= 180; angle += 10) // close claw
		  {
			  Set_Servo_Angle(&htim2, TIM_CHANNEL_1, angle);
			  HAL_Delay(100);
		  }
		break;

	default:
		break;
	}
}

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle)
{
    // Map angle (0-180) to pulse width (210-1050 counts)
    // 210 for 0.5ms (0 degrees) and 1050 for 2.5ms (180 degrees) - from servo datasheet
    uint32_t pulse_length = 210 + (angle * (1050 - 210) / 180);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
}


// STARTUP FUNCTIONS //
void homeZ(float targetDistance){ // move to open position, measure distance to bed and adjust z until within tolerance of reuested value
	//gripperControl(OPEN_GRIPPER);
	moveGripperTo((Position){positions[5].x, currentPosition.z});
	while(1){
		float dist = getAverageDistanceMm(5);
		float distCentimeter = dist/10;
		// tolerance (important to avoid oscillation)
		        if (distCentimeter > targetDistance + 2)
		        {
		        	moveStepper(2.0f, DIR_DOWN, MOTOR_Z);
		            //moveGripperTo((Position){currentPosition.x, currentPosition.z - 0.1f});
		        }
		        else if (distCentimeter < targetDistance - 2)
		        {
		        	moveStepper(2.0f, DIR_UP, MOTOR_Z);
		        	//moveGripperTo((Position){currentPosition.x, currentPosition.z + 0.1f});
		        }
		        else
		        {
		            currentPosition.z = distCentimeter; // Set measured height as current position to have heights referenced to a measured value from the bed
		            moveGripperTo((Position){0.0f, currentPosition.z});
		            break;
		        }
		        HAL_Delay(10);  // give time between readings
	}
}

void homeX()
{
    xLimitTriggered = false;

    // Move left continuously until switch is hit
    HAL_GPIO_WritePin(MOTORX_DIR_PORT, MOTORX_DIR_PIN, GPIO_PIN_RESET); // DIR_LEFT

    while (!xLimitTriggered) // set in an interrupt
    {
        // Single step pulse (manual stepping instead of moveStepper)
        HAL_GPIO_WritePin(MOTORX_STEP_PORT, MOTORX_STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(MOTORX_STEP_PORT, MOTORX_STEP_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    // Stop motion
    HAL_GPIO_WritePin(MOTORX_STEP_PORT, MOTORX_STEP_PIN, GPIO_PIN_RESET); // Stop X motor
    HAL_GPIO_WritePin(MOTORZ1_STEP_PORT, MOTORZ1_STEP_PIN, GPIO_PIN_RESET); // Stop Z1 motor
    HAL_GPIO_WritePin(MOTORZ2_STEP_PORT, MOTORZ2_STEP_PIN, GPIO_PIN_RESET); // Stop Z2 motor

    HAL_Delay(200); // debounce / settle

    // Move right 5 cm away from switch
    moveStepper(3.0f, DIR_RIGHT, MOTOR_X);

    // Set this as origin
    currentPosition.x = 0.0f;
}

float getAverageDistanceMm(int samples)
{
	// uses average of a few readings from the time of flight sensor to get a good distance reading
    float sum = 0.0f;
    int valid = 0;

    for (int i = 0; i < samples; i++)
    {
        if (VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData) == VL53L0X_ERROR_NONE)
        {
            // check measurement quality
            if (RangingData.RangeStatus == 0)  // 0 = valid
            {
                sum += RangingData.RangeMilliMeter;
                valid++;
            }
        }

        HAL_Delay(10); // small delay between samples
    }

    // avoid divide by zero
    if (valid == 0)
        return -1.0f;

    return sum / valid;
}

void measureBlocks(){
	// measure and record all block heights
	float measured[5];
	for (int i=0; i<5; i++){
		moveGripperTo((Position){positions[i].x - 2, Z_MEASURE_BLOCKS_HEIGHT});
		HAL_Delay(500);
		float measurement = getAverageDistanceMm(5);
		HAL_Delay(500);
		measured[i] = measurement;
	}

	//move gripper to starting location
	moveGripperTo((Position){0.0, Z_CARRY_HEIGHT});
    // convert to relative ranks (1–5)
    for (int i = 0; i < 5; i++)
    {
        int rank = 5;

        for (int j = 0; j < 5; j++)
        {
            if (i != j && measured[j] < measured[i])
            {
                rank--;
            }
        }

        blocks_heights[i] = rank;
    }
}


// POSITIONING / MOTION CONTROL
bool moveGripperTo(Position target)
{
    // Clamp Z to move no higher than Z_Carry_height and no lower than z_min_height to avoid hitting bed or getting to end of lead screws
    if (target.z < Z_MIN_HEIGHT)
        target.z = Z_MIN_HEIGHT;

    if (target.z > Z_CARRY_HEIGHT)
        target.z = Z_CARRY_HEIGHT;

    // move up
    if ((target.z - currentPosition.z) > 0)
        moveStepper((target.z - currentPosition.z), DIR_UP, MOTOR_Z);
    else if ((target.z - currentPosition.z) < 0)
        moveStepper(-(target.z - currentPosition.z), DIR_DOWN, MOTOR_Z);

    // move over
    if ((target.x - currentPosition.x) > 0)
        moveStepper((target.x - currentPosition.x), DIR_RIGHT, MOTOR_X);
    else if ((target.x - currentPosition.x) < 0)
        moveStepper(-(target.x - currentPosition.x), DIR_LEFT, MOTOR_X);

    currentPosition = target;
    return true;
}

bool moveGripper_over(Position target)
{
    // move up to safe height
    moveGripperTo((Position){currentPosition.x, Z_CARRY_HEIGHT});

    // move horizontally
    moveGripperTo((Position){target.x, Z_CARRY_HEIGHT});

    // move down to goal
    moveGripperTo(target);

    return true;
}


// HIGH LEVEL //
void pickBlock(Position pos)
{
	// picks up block in position pos
    moveGripper_over((Position){pos.x, Z_PICK_HEIGHT});
    gripperControl(CLOSE_GRIPPER);
    HAL_Delay(500);

    moveGripperTo((Position){pos.x, Z_CARRY_HEIGHT});
}

void placeBlock(Position pos)
{
	//drops block in position pos
    moveGripper_over((Position){pos.x, Z_PICK_HEIGHT + 0.5});
    gripperControl(OPEN_GRIPPER);
    HAL_Delay(500);

    moveGripperTo((Position){pos.x, Z_CARRY_HEIGHT});
}

void swapBlocks_withBuffer(int posA, int posB)
{
	// uses buffer space to swap two blocks
    Position A = positions[posA];
    Position B = positions[posB];
    Position buffer = positions[BUFFER_POS_INDEX];

    // 1. Pick A → move to buffer
    pickBlock(A);
    placeBlock(buffer);

    // 2. Pick B → move to A
    pickBlock(B);
    placeBlock(A);

    // 3. Pick buffer → move to B
    pickBlock(buffer);
    placeBlock(B);
}

void swapBlocks_moveIntoPosition(int fromPos, int toPos)
{
	// moves multiple blocks to insert
    Position buffer = positions[BUFFER_POS_INDEX];

    if (fromPos == toPos)
        return;

    // Put the moving block into the buffer first
    pickBlock(positions[fromPos]);
    placeBlock(buffer);

    if (fromPos > toPos)
    {
        // Example: 4 -> 1
        // Shift 3->4, 2->3, 1->2
        for (int k = fromPos - 1; k >= toPos; k--)
        {
            pickBlock(positions[k]);
            placeBlock(positions[k + 1]);
        }
    }
    else
    {
        // Example: 1 -> 4
        // Shift 2->1, 3->2, 4->3
        for (int k = fromPos + 1; k <= toPos; k++)
        {
            pickBlock(positions[k]);
            placeBlock(positions[k - 1]);
        }
    }

    // Put the buffered block into its final location
    pickBlock(buffer);
    placeBlock(positions[toPos]);
}


// SORTING FUNCTIONS //
int sortingAlgorithm(int algorithm)
{
	int numSwaps = 0;
	if (algorithm == BUBBLE_SORT)
	{
		for (int i = 0; i < 4; i++) // i < number of blocks - 1
		{
			for (int j = 0; j < 4; j++) // j < number of blocks - 1
			{
				if (blocks_heights[j] > blocks_heights[j+1]) // Swap at this location needs to happen
				{
					// Updating swap array
					swapList[numSwaps].pos1 = j;
					swapList[numSwaps].pos2 = j+1;
					numSwaps++;

					// Updating heights array
					int temp = blocks_heights[j];
					blocks_heights[j] = blocks_heights[j+1];
					blocks_heights[j+1] = temp;
				}

			}
		}
	}
	else if (algorithm == INSERTION_SORT)
	{
		for (int i = 1; i < 5; i++)
		{
			int key = blocks_heights[i];
			int j = i;

			// Find where this block should go
			while (j > 0 && key < blocks_heights[j - 1])
			{
				blocks_heights[j] = blocks_heights[j - 1];
				j--;
			}

			// Put the key in its final sorted position
			blocks_heights[j] = key;

			// If it actually moved, record it as one shift move
			if (j != i)
			{
				swapList[numSwaps].pos1 = i;   // from
				swapList[numSwaps].pos2 = j;   // to
				numSwaps++;
			}
		}
	}

	return numSwaps;
}

void executeSwapList(int numSwaps, int algorithm)
{
    if (algorithm == INSERTION_SORT)
    {
        for (int i = 0; i < numSwaps; i++)
        {
            swapBlocks_moveIntoPosition(swapList[i].pos1, swapList[i].pos2);
        }
    }
    else
    {
        for (int i = 0; i < numSwaps; i++)
        {
            swapBlocks_withBuffer(swapList[i].pos1, swapList[i].pos2);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// limit switch bumper interrupt (x direction)
    if(GPIO_Pin == GPIO_PIN_1)  // limit switch pin
    {
        xLimitTriggered = true;
    }
}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

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
#ifdef USE_FULL_ASSERT
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
