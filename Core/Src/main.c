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
#define ADDRES_PIN_1 GPIOB, GPIO_PIN_11
#define ADDRES_PIN_2 GPIOB, GPIO_PIN_10
#define ADDRES_PIN_3 GPIOB, GPIO_PIN_2
#define ADDRES_PIN_4 GPIOB, GPIO_PIN_1
#define ADDRES_PIN_5 GPIOB, GPIO_PIN_0
#define ADDRES_PIN_6 GPIOA, GPIO_PIN_7
#define ADDRES_PIN_7 GPIOA, GPIO_PIN_6
#define ADDRES_PIN_8 GPIOA, GPIO_PIN_5
#define ADDRES_PIN_9 GPIOA, GPIO_PIN_4


#define STEP_MODE_1_1		0
#define STEP_MODE_1_2		1
#define STEP_MODE_1_4		2
#define STEP_MODE_1_8		3
#define STEP_MODE_1_16	4
#define STEP_MODE_1_32	5


#define ZERO_FOUND 		 				0
#define START_SEARHING_ZERO		1
#define SEARHING_ZERO  				2


//Протокол управления 0 - STEP_DIR 1 -  CW_CCW
#define PROTOCOL 0

#include <math.h>

#define MOVE 1
#define STOP 0

//Количество импульсов на один оборот мотора
int Steps_Rev = 3200; 
//Максимальное кол-во оборотов при значении 255 на первом канале DMX
int64_t Max_Rev = 255;



//Множитель для получения необходимых кол-ва импульсов от 0 до 255 значений канала позиции
float multiplier = 0.0f;

float error = 0;
float max_error = 0;

int64_t setPosition = 0;
float set_speed = 0.0f;
float a = 1.0f/5000;

char search_zero = ZERO_FOUND;
char state = STOP;

//Тормозной путь при текущей скорости и ускорении
float deseleration_distance = 0;

//Точка в которой остановимся если начнем сейчас тормозить с заданным ускорением
int64_t stop_position = 0;

//Текущая скорость 
double actualSpeed = 0;

//Текущее ускорение
float actualAcceleration = 0;

//Прошлое значение позиции, дробная часть
float oldActualPosition = 0;
//Прошлое значение позиции, целая часть, полное кол-во сделанных шагов
int64_t full_oldActualPosition = 0;

int64_t full_steps_actual_position = 0;
float actualPosition = 0;


//Переменные для работы DMX
#define COUNT_CHANNELS 7


uint8_t data[COUNT_CHANNELS] = {0};
uint8_t data_old[COUNT_CHANNELS] = {0};

//Номер текущего принимаемого канала, при отлове ошибки передачи, считаем что передача кадра завершена, будем обнулять DMX_Current_Channel для того что-бы заново копить каналы в буфер
unsigned int DMX_Current_Channel = 0;

//С какого канала начинаем принимать данные, по сути адрес DMX устройства
uint16_t DMX_Start_Channel = 0;

char DMX_Break = 0;

uint8_t byte_receive[1] = {0};

_Bool swich_arrdess[9] = {0};
	
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//Выбор режима микрошага для драйвера drv8825
void set_step_mode( int8_t type_mode)
{
	switch(type_mode) {
		
		case STEP_MODE_1_1:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	//M0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //M1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//M2
		break;
		
		case STEP_MODE_1_2:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);	//M0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //M1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//M2
		break;
		
		case STEP_MODE_1_4:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	//M0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //M1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//M2
		break;
		
		case STEP_MODE_1_8:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);	//M0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //M1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//M2
		break;
		
		case STEP_MODE_1_16:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	//M0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //M1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);	//M2
		break;
		
		case STEP_MODE_1_32:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	//M0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //M1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);	//M2
		break;
	}
}


//Функция ожидания прихода в заданную точку
void Waiting_End_Move()
{
	HAL_Delay(10);
	while(state == MOVE) HAL_Delay(1);
}


//Обработчик прерывания датчика нуля
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13 && search_zero == SEARHING_ZERO)
  {
    actualPosition = 0;
		full_steps_actual_position = 0;
		setPosition = 0;
		search_zero = ZERO_FOUND;
		
  }
  else 
    {
      __NOP();
  
    }
}


//Прерывание, срабатывает когда получен байт по UART
	void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart) {

  if(huart == &huart1) 
		{
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			
			//Для того, что бы начать принимать только после получения сигнала BREAK, иначе после включения, рискуем принимать в середине передачи.
			if (DMX_Break == 1){
				
				if ((DMX_Current_Channel >= DMX_Start_Channel) && DMX_Current_Channel < (DMX_Start_Channel + COUNT_CHANNELS)){
					data[(DMX_Current_Channel - DMX_Start_Channel)] = byte_receive[0];
				}
				DMX_Current_Channel++;
							
			}
			//Ожидаем байта данных следующего канала
			HAL_UART_Receive_IT(&huart1, byte_receive, 1);
			
			
		}
}



void HAL_UART_ErrorCallback ( UART_HandleTypeDef *huart)
{
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		
		if (huart == &huart1)
			{

				__HAL_UART_CLEAR_FEFLAG(huart);
				
				DMX_Break = 1;
				
				if(DMX_Current_Channel > 513 )
				{
//					if(data[1] > 100 ) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//					else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					//Если на одном из каналов произошло изменение, мигаем светодиодом
					if(
						data_old[0] != data[0] ||
						data_old[1] != data[1] ||
						data_old[2] != data[2] ||
						data_old[3] != data[3] ||
						data_old[4] != data[4] ||
						data_old[5] != data[5] ||
						data_old[6] != data[6]
						)
					{
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					}

					else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					
					if(search_zero == ZERO_FOUND)
					{
						
							setPosition = data[0]*255;
							setPosition += data[1];
							setPosition = setPosition*multiplier;
							set_speed = (data[2]/255.0)*0.7;
							a = 1.0/((((data[3]*data[3])/65025.0)*2000000)+1500);
							
						//a = 1.0/((data[3]*10000)+1500);
					}
	//				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 65535);
	//				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 64535);
	//				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 65535);
					
					if(data_old[4] != data[4]) TIM2->CCR2 = data[4];
					if(data_old[5] != data[5]) TIM2->CCR1 = data[5];
					if(data_old[6] != data[6]) TIM2->CCR4 = data[6];
					
					memcpy(data_old , data, COUNT_CHANNELS);
				}
				DMX_Current_Channel = 0;

				
				
				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	
	
			}

}

// функция которая срабатывает по таймеру 1, она расчитывает траекторию и говорит мотору когда нужно сделать шаг.

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  
{

	
//			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//			else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	
	

			error = 0;
			//Тормозной путь при текущей скорости и ускорении
			deseleration_distance = ( (fabs(actualSpeed) - a) * fabs(actualSpeed) ) / (2.0f * a) ;
			
			//Точка в которой остановимся если начнем сейчас тормозить с заданным ускорением
			stop_position = full_steps_actual_position;

			//Если протокол управления драйвером 0 - STEP/DIR
			#if PROTOCOL == 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			#endif
	
			//Если протокол управления драйвером 1 - CW/CCW
			#if PROTOCOL == 1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			#endif
	
			
			//Расчитываем точу остановки, если начнем сейчас тормозить
			if (actualSpeed < 0) stop_position = full_steps_actual_position + (actualPosition - deseleration_distance);				
			if (actualSpeed > 0) stop_position = full_steps_actual_position + (actualPosition + deseleration_distance);
			
			
			if(fabs(stop_position - setPosition) < 0.000001f) stop_position = setPosition;
	
			oldActualPosition = actualPosition;
			full_oldActualPosition = full_steps_actual_position;

			if (setPosition != full_steps_actual_position && actualSpeed != 0) 
			{
				state = MOVE;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			}
			
				error = fabs(setPosition - stop_position);
				
				//Если мы сейчас начнём тормозить и проедем нужную точку
				if (
					(actualSpeed > 0 && stop_position >= setPosition && error < 4)|| 
					(actualSpeed < 0 && stop_position <= setPosition && error < 4))
				{

					
				
					if(max_error < error) max_error = error;
					
					//if(error > 4 ) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

					if( actualSpeed > 0 ) actualSpeed -= a;
					if( actualSpeed < 0 ) actualSpeed += a;

					actualPosition += actualSpeed;
					if(actualPosition >= 1) 
					{
						actualPosition -= 1; 
						full_steps_actual_position += 1;
					}
					if(actualPosition < 0) 
					{
						actualPosition += 1; 
						full_steps_actual_position -= 1;
					}
					

					if ((full_steps_actual_position >= setPosition && full_oldActualPosition < setPosition)||(full_oldActualPosition <= setPosition && full_oldActualPosition > setPosition))
					//if (fabs(actualPosition - setPosition ) <= 0.002)
					{
						state = STOP;
						full_steps_actual_position = setPosition;
						actualPosition = 0;
						actualSpeed = 0;
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
						
					}

				}

				else
				{
					if (full_steps_actual_position < setPosition)
					{
						if (actualSpeed > set_speed)
						{
							actualSpeed -= a;
							if (actualSpeed < set_speed) actualSpeed = set_speed;

						}
						else if (actualSpeed < set_speed)
						{
							actualSpeed += a;
							if (actualSpeed > set_speed) actualSpeed = set_speed;
						}
					}
					else if (full_steps_actual_position > setPosition)
					{
						if (actualSpeed > -set_speed) 
						{ 
							actualSpeed -= a;
							if (actualSpeed < -set_speed) actualSpeed = -set_speed;
						
						}
						else if (actualSpeed < -set_speed)
						{
							actualSpeed += a;
							if (actualSpeed > -set_speed) actualSpeed = -set_speed;
						}
					}
					actualPosition += actualSpeed;
					if(actualPosition >= 1) 
					{
						actualPosition -= 1; 
						full_steps_actual_position += 1;
					}
					if(actualPosition < 0) 
					{
						actualPosition += 1; 
						full_steps_actual_position -= 1;
					}

				}
				//STEP_DIR
				#if PROTOCOL == 0
				
				//Если актуальная позиция увеличилась на целое число по сравнению с прошлой позицией, устанавливаем прямое направление на ноге DIR контроллера ШД.
				if(full_steps_actual_position > full_oldActualPosition)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
					
				}
				//А если уменьшилась, то устанавливаем обратное направление на ноге DIR контроллера ШД
				if(full_steps_actual_position < full_oldActualPosition)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
					
				}
				
				
				//Если прошлое значение актуальной позиции изменилось на целое число, шлём импульс на перемещение
				if(full_steps_actual_position != full_oldActualPosition)
				{	
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
				}
				
				#endif
				
				//CW_CCW
				#if PROTOCOL == 1
				
				//Если актуальная позиция увеличилась на целое число по сравнению с прошлой позицией, даем импульс на 15 ногу.
				if(full_steps_actual_position > full_oldActualPosition)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					
				}
				//А если уменьшилась, даем импульс на 14 ногу.
				if(full_steps_actual_position < full_oldActualPosition)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
					
				}
					
				#endif

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	
	swich_arrdess[0] = !HAL_GPIO_ReadPin(ADDRES_PIN_1);
	swich_arrdess[1] = !HAL_GPIO_ReadPin(ADDRES_PIN_2);
	swich_arrdess[2] = !HAL_GPIO_ReadPin(ADDRES_PIN_3);
	swich_arrdess[3] = !HAL_GPIO_ReadPin(ADDRES_PIN_4);
	swich_arrdess[4] = !HAL_GPIO_ReadPin(ADDRES_PIN_5);
	swich_arrdess[5] = !HAL_GPIO_ReadPin(ADDRES_PIN_6);
	swich_arrdess[6] = !HAL_GPIO_ReadPin(ADDRES_PIN_7);
	swich_arrdess[7] = !HAL_GPIO_ReadPin(ADDRES_PIN_8);
	swich_arrdess[8] = !HAL_GPIO_ReadPin(ADDRES_PIN_9);


	
	for (int i = 0; i <= 8; i++) 
	{
		
        if (swich_arrdess[i]) {
						
            DMX_Start_Channel |= 1 << i;
        }

				
  }
	if(DMX_Start_Channel == 0)
		{ 
			while(1)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_Delay(1000);
				
			}
		}
	
	DMX_Start_Channel = DMX_Start_Channel;
	
	HAL_TIM_Base_Start_IT(&htim1);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	TIM2->CCR1 = 150;
	HAL_Delay(200);
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 150;
	HAL_Delay(200);
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR4 = 150;
	HAL_Delay(200);
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR4 = 0;
	
	
	//set_step_mode(STEP_MODE_1_8);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);	//M0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //M1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //M1
	
	
	HAL_UART_Receive_IT(&huart1, byte_receive, 1);


	multiplier = ((float)Steps_Rev * (float)Max_Rev)/65025.0f;
	
	set_speed = 0.01;
	a = 1.0/1000;

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if(search_zero == START_SEARHING_ZERO)
		{
			//Если датчик уже перекрыт, то отъезжаем немного назад и потом движемся до срабатывания прерывания по датчику нуля
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 1)
			{
				setPosition = -50;
				Waiting_End_Move();
				search_zero = SEARHING_ZERO;
				setPosition = 1000000;
			}
			//Если датчик ещё не перекрыт, то сразу двигаемся до срабатывания прерывания по датчику нуля
			else
			{
				search_zero = SEARHING_ZERO;
				setPosition = 1000000;
			}

		}
		
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		HAL_Delay(1000);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim1.Init.Prescaler = 580;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
