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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_simulation.h"
#include "bsp_dwt.h"
#include <stdbool.h>
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
typedef struct 
{
    float err;
    float err_sum;
    float err_difference;
    float last_err;
    float Kp;
    float Ki;
    float Kd;
    float output_filter;    
    float lvbo;  //低通滤波参数
} pid_t;

pid_t pid_velocity, pid_angle; 
uint32_t DWT_CNT;
float dt;
float t;
motorObject_t Motor;
float VelocityRef=0, AngleRef=0, Input=0;
float Kp_v=0.1, Ki_v=0.01, Kd_v=0.02;  //速度闭环
float Kp_p=0.5, Ki_p=0.0, Kd_p=0.1;    //角度闭环

// 控制模式定义
#define VELOCITY_CONTROL 0    // 速度控制模式
#define ANGLE_CONTROL 1       // 角度控制模式

// 速度控制模式下的响应类型
#define VELOCITY_STEP 0       // 速度阶跃
#define VELOCITY_FREQUENCY 1  // 速度频率响应

// 角度控制模式下的响应类型
#define ANGLE_STEP 0         // 角度阶跃
#define ANGLE_FREQUENCY 1    // 角度频率响应
#define ANGLE_DISTURBANCE 2  // 角度干扰响应

uint8_t control_mode = VELOCITY_CONTROL;  // 控制模式选择
uint8_t velocity_response_mode = VELOCITY_STEP;  // 速度响应模式
uint8_t angle_response_mode = ANGLE_STEP;       // 角度响应模式

float target_velocity = 2.0;      // 目标速度值
float target_angle = 3.14159;     // 目标角度值
float current_reference = 0.0;    

float sine_amplitude = 1.57;      // 正弦信号幅值(π/2弧度)
float sine_frequency = 0.5;       // 正弦信号频率
float sine_offset = 0.0;          // 正弦信号偏置

float disturbance_start_time = 5.0;    // 干扰开始
float disturbance_magnitude = 2.0;      // 干扰幅值
bool disturbance_applied = false;       // 干扰标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* USER CODE BEGIN 2 */
	void PID_Init(pid_t *pid, float Kp, float Ki, float Kd);
	float PID_Calculate(pid_t *pid, float target, float current);
    
    PID_Init(&pid_velocity, Kp_v, Ki_v, Kd_v);
    PID_Init(&pid_angle, Kp_p, Ki_p, Kd_p);
    Motor_Object_Init(&Motor);
    DWT_Init(72);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    dt = DWT_GetDeltaT(&DWT_CNT);//利用 DWT 定时器获得仿真周期
    t += dt;

    if (control_mode == VELOCITY_CONTROL) 
    {
        // 速度控制模式
        switch(velocity_response_mode)
        {
            case VELOCITY_STEP:
                current_reference = target_velocity;
          break;
          
            case VELOCITY_FREQUENCY:
                current_reference = sine_offset + sine_amplitude * sinf(2.0 * 3.14159 * sine_frequency * t);
                break;
        }
        
        // 速度闭环控制
        Input = PID_Calculate(&pid_velocity, current_reference, Motor.Velocity);
          } 
    else if (control_mode == ANGLE_CONTROL) 
    {
        // 角度控制模式
        switch(angle_response_mode)
            {
            case ANGLE_STEP:
                current_reference = target_angle;
                break;
                
            case ANGLE_FREQUENCY:
                current_reference = sine_offset + sine_amplitude * sinf(2.0 * 3.14159 * sine_frequency * t);
                break;

            case ANGLE_DISTURBANCE:
                current_reference = target_angle;
                if (t >= disturbance_start_time && !disturbance_applied) 
                {
                    Motor.Velocity += disturbance_magnitude;
                    disturbance_applied = true;
                }
          break;
    }

        // 角度闭环控制（串级PID）
        VelocityRef = PID_Calculate(&pid_angle, current_reference, Motor.Angle);
        Input = PID_Calculate(&pid_velocity, VelocityRef, Motor.Velocity);
    }

    Motor_Simulation(&Motor, Input, dt);
    Motor.Velocity = Get_Motor_Velocity(&Motor);
    Motor.Angle += Motor.Velocity * dt;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void PID_Init(pid_t *pid, float Kp, float Ki, float Kd)//pid初始化
{
    pid->err = 0;
    pid->err_sum = 0;
    pid->err_difference = 0;
    pid->last_err = 0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_filter = 0;
    pid->lvbo = 0.1;  // 滤波系数
}

float PID_Calculate(pid_t *pid, float target, float current)
{
    pid->err = target - current;
    pid->err_sum += pid->err;
    pid->err_difference = pid->err - pid->last_err;
    pid->last_err = pid->err;
    
    if(pid->err_sum > 1000) pid->err_sum = 1000;
    if(pid->err_sum < -1000) pid->err_sum = -1000;
    
    float output = (pid->Kp * pid->err) + (pid->Ki * pid->err_sum) + (pid->Kd * pid->err_difference);
    
    pid->output_filter = pid->lvbo * output + (1 - pid->lvbo) * pid->output_filter;
    
    return pid->output_filter;
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
