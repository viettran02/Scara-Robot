/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmath"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Configure X-axis parameters */
#define STEP_PinX 	GPIO_PIN_1
#define DIR_PinX 		GPIO_PIN_4

/* Configure Y-axis parameters */
#define STEP_PinY 	GPIO_PIN_2
#define DIR_PinY		GPIO_PIN_5

/* Configure Z-axis parameters */
#define STEP_PinZ 	GPIO_PIN_3
#define DIR_PinZ 		GPIO_PIN_6

/* Configure HC_SR04 parameters */
#define TrigPin			GPIO_PIN_9
#define EchoPin			GPIO_PIN_10

/* Configure other parameters */
#define pi 3.1415926535


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* Variable mode control */ 
uint8_t Manualdata;
uint8_t USB_Rx_Buffer[64];
float Xdata, Ydata, Zdata;
float CurDataX, CurDataY, CurDataZ;
double thetaX, thetaY;
char Modedata;
char Manualdata1;

/* Variable of Measure distance Z-axis*/
uint32_t pMillis;
uint32_t val1 = 0;
uint32_t val2 = 0;
int16_t distance  = 0;

/* Variable of Shelf. */
uint16_t LengthWH = 159;
uint8_t disX = 0;
uint8_t disY = 250;

/* Variable of current posittion */
double temp1, temp2, pz_temp;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Delay_us(uint16_t timedelay);
void moveXY(long nStepX, int PInX,int dirX, long nStepY, int PInY,int dirY);
void backhome(double t1,double t2);
void Resethome(double theta1, double theta2);
void PositionControl(float Xvalue, float Yvalue, int16_t Zvalue);
void PerformSteps(int32_t steps, uint8_t direction, int stepPin, int dirPin);
void CurrentPosittion(float valueX, float valueY, int16_t valueZ);
void ImportProduct(uint8_t ProductID);
void ModeControl(uint8_t dataMode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ----------------------CODE MOVE SCARA ARM ROBOT BEGINE--------------------*/

/* Function delay us --BEGIN-- */
void microDelay (uint32_t delay){
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}
/* Funotion delay us --END--*/

/* Function measure distance for Z-axis --BEGIN-- */
void measureDistance(){
	//Create pwm 10us
	HAL_GPIO_WritePin(GPIOA, TrigPin, GPIO_PIN_SET);
	microDelay(10);
	HAL_GPIO_WritePin(GPIOA, TrigPin, GPIO_PIN_RESET);
	
	
	pMillis = HAL_GetTick();
	while (!(HAL_GPIO_ReadPin (GPIOA, EchoPin)) && pMillis + 10 >  HAL_GetTick());
	val1 = __HAL_TIM_GET_COUNTER (&htim1);

	pMillis = HAL_GetTick();
	while ((HAL_GPIO_ReadPin (GPIOA, EchoPin)) && pMillis + 50 > HAL_GetTick());
	val2 = __HAL_TIM_GET_COUNTER (&htim1);
	
	distance = (int)((val2-val1)*0.034/2*10);
	
	char distance_str[1];
	sprintf(distance_str, "%d\n", distance);
	CDC_Transmit_FS((uint8_t*) distance_str, strlen(distance_str));
	HAL_Delay(1000);
}
/* Function measure distance for Z-axis --BEGIN-- */

/* Function Reset system --BEGIN-- */
void ResetSystem(void){
	 HAL_NVIC_SystemReset();
}
/* Function Reset system --END-- */

/* Move the robot arm on the xy plane-- BEGIN -- */
void moveXY(long nStepX, int stepPInX,int dirX, long nStepY, int stepPInY,int dirY){
  double nStepMax = nStepX;
  double nStepMin = nStepY;
  int stepPInMax = stepPInX;
  int stepPInMin = stepPInY;

  double current_axis_min = 0;
  long steps_axis_min = 0;
  double ratio_max_min = 0;

  if (nStepY > nStepX) {
    nStepMax = nStepY;
    nStepMin = nStepX;
    stepPInMax = stepPInY;
    stepPInMin = stepPInX;
  }

  ratio_max_min = nStepMax / nStepMin;
  if( dirX==1)
  {
    HAL_GPIO_WritePin(GPIOA,DIR_PinX, GPIO_PIN_SET);		// X-axis :CW.
  }
  else
  {
		HAL_GPIO_WritePin(GPIOA,DIR_PinX, GPIO_PIN_RESET);	 			// X-axis : CCW.
  }	

 if( dirY==1)
  {
		HAL_GPIO_WritePin(GPIOA,DIR_PinY, GPIO_PIN_SET);				// Y-axis : CW.
  }
  else
  {
		HAL_GPIO_WritePin(GPIOA,DIR_PinY, GPIO_PIN_RESET);				// Y-axis : CCW.
  }

  for (int i = 1 ; i <= nStepMax ; i = i + 1) {
    current_axis_min = i / ratio_max_min;

    if (current_axis_min - steps_axis_min >= 1) {
			HAL_GPIO_WritePin(GPIOA,stepPInMin, GPIO_PIN_SET);
			
      steps_axis_min++;
    }
    HAL_GPIO_WritePin(GPIOA,stepPInMax, GPIO_PIN_SET);
		microDelay(50);
    HAL_GPIO_WritePin(GPIOA,stepPInMax, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA,stepPInMin, GPIO_PIN_RESET);
    microDelay(50);
  }
}
/* Move the robot arm on the xy plane --END-- */

/* Back home position from another positton --BEGIN-- */
void backhome(double theta1,double theta2){ 
      double dp2X=0,dp2Y=0;
      int dir2X=0,dir2Y=0;
			dp2X = (fabs(theta1) * (180 / pi)) / (1.8 / 16); 
			dp2Y = (fabs(theta2) * (180 / pi)) / (1.8 / 16);
      if (theta1<0)
      {
           dir2X=1;
      }
      else
      {
          dir2X=0;
      }
      
       if (theta2<0)
      {
           dir2Y=1;
      }
      else
      {
          dir2Y=0;
      }

    dp2X = (int)(54*dp2X);
		dp2Y = (int)(16*dp2Y);
		moveXY(dp2X,STEP_PinX,dir2X,dp2Y,STEP_PinY,dir2Y);
	}
/* Back home position from another positton --BEGIN-- */

/* Function perform Step --BEGIN-- */
void PerformSteps(int32_t steps, uint8_t direction, int stepPin, int dirPin){
		if(direction == 0)
		{
			HAL_GPIO_WritePin(GPIOA, dirPin, GPIO_PIN_RESET); // Set direction CW.
		}
		else if (direction == 1)
		{
			HAL_GPIO_WritePin(GPIOA, dirPin, GPIO_PIN_SET);		// Set direction CW.
		}
    for (int32_t i = 0; i <= steps; i++)
    {
        HAL_GPIO_WritePin(GPIOA, stepPin, GPIO_PIN_SET);
        microDelay(150);
        HAL_GPIO_WritePin(GPIOA, stepPin, GPIO_PIN_RESET);
        microDelay(150);
    }
}
/* Function perform Step --END-- */

/* Move robot up or down --BEGIN--*/
void UpDown(int16_t distance, uint8_t direction){
	PerformSteps((int)(distance*320),direction , STEP_PinZ, DIR_PinZ);		
}
/* Move robot up or down --END--*/

/* Function Reset home --BEGIN-- */ 
void ResetHome(double theta1,double theta2, int8_t distance){ 
      double dp2X=0,dp2Y=0;
      int dir2X=0,dir2Y=0;
			dp2X = (fabs(theta1) * (180 / pi)) / (1.8 / 16); 
			dp2Y = (fabs(theta2) * (180 / pi)) / (1.8 / 16);
      if (theta1<0)
      {
           dir2X=1;
      }
      else
      {
          dir2X=0;
      }
      
       if (theta2<0)
      {
           dir2Y=1;
      }
      else
      {
          dir2Y=0;
      }

    dp2X = (int)(54*dp2X);
		dp2Y = (int)(16*dp2Y);
		moveXY(dp2X,STEP_PinX,dir2X,dp2Y,STEP_PinY,dir2Y);
			
		temp1 = 0;
		temp2 = 0;
			
		if(distance < 0){
			UpDown(-distance,1);
		}
		else{
			UpDown(distance,0);
		}
}	
/* Function Reset home --END-- */ 

/* Function Current posittion --BEGIN-- */
void CurrentPosittion(float valueX, float valueY, int16_t valueZ){
	uint16_t l1 = 332;
	uint8_t l2 = 185;
	double px, py, c1, s1, c2,s2;
	 
	px = valueX;
	py = valueY;
	pz_temp = valueZ;
	
	c2=((px*px+py*py-l1*l1-l2*l2))/(2*l1*l2);
  s2=sqrt(fabs(1-c2*c2));
	
	c1 = (px*l1+px*l2*c2 + py*l2*s2);
	s1 = (py*l1+py*l2*c2 - px*l2*s2);
	
  temp1 = atan(s1/c1);
	temp2 = atan(s2/c2);
	
	if (c2<0)
  {
    temp2  = pi + temp2;
  }
  if (c1<0)
  {
    temp1 = pi + temp1;
  }
}
/* Function Current posittion --END-- */

/* Move to XY position --BEGIN--*/
void PositionControl(float Xvalue, float Yvalue, int16_t Zvalue){
	int dirX=0,dirY=0;
  float l1= 332;
  float l2= 185;
  double px=0,py=0, pz=0, c1=0,s1=0,c2=0,s2=0;
  double t1=0,t2=0,dpX=0,dpY=0;
	
	px = Xvalue;
  py = Yvalue;
	pz = Zvalue;
	
  c2=((px*px+py*py-l1*l1-l2*l2))/(2*l1*l2);
  s2=sqrt(fabs(1-c2*c2));

	c1 = (px*l1+px*l2*c2 + py*l2*s2);
	s1 = (py*l1+py*l2*c2 - px*l2*s2);
	
  t1=atan(s1/c1);
	t2=atan(s2/c2);
	
	if (c2<0)
  {
    t2 = pi+t2;
  }
  if (c1<0)
  {
    t1 = pi+t1;
  }
	
	CurrentPosittion(CurDataX, CurDataY, CurDataZ);
	
	dpX=(fabs(t1-temp1)*(180/pi))/(1.8/16);
  dpY=(fabs(t2-temp2)*(180/pi))/(1.8/16);
  if ((t1-temp1)>0)
  {
    dirX=1;
  }
  else
	{
    dirX=0;
  }  
  if ((t2-temp2)>0)
  {
    dirY=1;
  }
  else
  {
    dirY=0;
  }
	dpX = (int)(54*dpX);
	dpY = (int)(16*dpY);
  moveXY(dpX,STEP_PinX,dirX, dpY, STEP_PinY,dirY);
	
	if(pz >= pz_temp){
		UpDown(pz-pz_temp, 1);
	}	
	else{
		UpDown(pz_temp - pz,0);
	}
}
/* Move to XY position --END--*/

/* Move to Product Posittion --BEGIN--*/
void PosProduct(float Xvalue, float Yvalue){
	int dirX=0,dirY=0;
  float l1= 332;
  float l2= 185;
  double px=0,py=0,c1=0,s1=0,c2=0,s2=0;
  double t1=0,t2=0,dpX=0,dpY=0;
	
	px = Xvalue;
  py = Yvalue;

  c2=((px*px+py*py-l1*l1-l2*l2))/(2*l1*l2);
  s2=sqrt(fabs(1-c2*c2));

	c1 = (px*l1+px*l2*c2 + py*l2*s2);
	s1 = (py*l1+py*l2*c2 - px*l2*s2);
	
  t1=atan(s1/c1);
	t2=atan(s2/c2);
	
	if (c2<0)
  {
    t2 = pi+t2;
  }
  if (c1<0)
  {
    t1 = pi+t1;
  }
	
	CurrentPosittion(CurDataX, CurDataY, CurDataZ);
	
	if(pz_temp < 0) UpDown(-pz_temp,1); 
	
	dpX=(fabs(t1-temp1)*(180/pi))/(1.8/16);
  dpY=(fabs(t2-temp2)*(180/pi))/(1.8/16);
  if ((t1-temp1)>0)
  {
    dirX=1;
  }
  else
	{
    dirX=0;
  }  
  if ((t2-temp2)>0)
  {
    dirY=1;
  }
  else
  {
    dirY=0;
  }
	dpX = (int)(54*dpX);
	dpY = (int)(16*dpY);
  moveXY(dpX,STEP_PinX,dirX, dpY, STEP_PinY,dirY);
	UpDown(20,0);
	UpDown(20,1);
	backhome(t1,t2);
}
/* Move to Product Posittion --END--*/

/* Function Manual Control --BEGIN-- */
void ManualControl(uint8_t dataManual){
	switch (dataManual){
		case 'Q':
			PerformSteps(2400, 0, STEP_PinX, DIR_PinX);		      // Move X-rotation 5degree/event - direc: CW.
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			break;
    case 'W':
			PerformSteps(2400, 1, STEP_PinX, DIR_PinX); 		    // Move X-rotation 5degree/event - direc: CCW.
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			break;
		case 'E':
			PerformSteps(711, 0, STEP_PinY, DIR_PinY); 					// Move Y-rotation 5degree/event - direc: CW.
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			break;
    case 'R':
			PerformSteps(711, 1, STEP_PinY, DIR_PinY); 					// Move Y-rotation 5degree/event - direc: CCW.
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			break;
    case 'T':
			PerformSteps(640, 1, STEP_PinZ, DIR_PinZ);					// Move Z-motion 2mm/event - direc: up
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      break;
    case 'Y':
			PerformSteps(640, 0, STEP_PinZ, DIR_PinZ);					// Move Z-motion 2mm/event - direc: down
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			break;
    default: 
      // Handle unrecognized command
      break;
			}
}
/* Function Manual Control --END-- */

/* Function Import Products --BEGIN-- */
void ImportProduct(uint8_t ProductID){

	float CenterX, CenterY;
	
	if (ProductID % 3 != 0){
			CenterX = (ProductID%3) * ( LengthWH /3) - (LengthWH/6) + disX;
			CenterY = ((int)(ProductID/3) + 1)* (LengthWH /3) - (LengthWH/6) + disY;
	}
	else{
			CenterX = LengthWH - (LengthWH/6) + disX;
			CenterY = (int)(ProductID/3)* (LengthWH /3) - (LengthWH/6) + disY;
	}
	PosProduct(CenterX, CenterY);
}
/* Function Import Products --END-- */

/* Function Selected mode --BEGIN-- */
void ModeControl(uint8_t dataMode){
	switch (dataMode){
		case 'M':
			ManualControl(Manualdata);
			break;
    case 'A':
			PositionControl(Xdata, Ydata, Zdata);
			break;
		case 'V':
			ResetHome(CurDataX, CurDataY, CurDataZ);
			break;
		case 'I':
			ImportProduct(Xdata);
			break;
		case 'L':
			ResetSystem();
		default:
			break;
	}
}
/* Function Selected mode--END-- */
 
/* ----------------------CODE MOVE SCARA ARM ROBOT END----------------------------------------*/

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
	
	// Turn on the clock for port A
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//measureDistance();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, StepX_Pin|StepY_Pin|StepZ_Pin|DirX_Pin
                          |DirY_Pin|DirZ_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : StepX_Pin StepY_Pin StepZ_Pin DirX_Pin
                           DirY_Pin DirZ_Pin */
  GPIO_InitStruct.Pin = StepX_Pin|StepY_Pin|StepZ_Pin|DirX_Pin
                          |DirY_Pin|DirZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
