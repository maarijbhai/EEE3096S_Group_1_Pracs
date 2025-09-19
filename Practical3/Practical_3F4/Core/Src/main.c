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
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "core_cm4.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ITER 100
#define SCALE 1000000

// for bit shifting in the fixed point arithmetic function
#define Q   16			// scale factor
#define ONE (1 << Q)	// division by scale factor

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)
uint64_t start_time = 0;
uint64_t end_time = 0;
int imageDimensions[5] = {128, 160, 192, 224, 256};
uint64_t checksum = 0;
uint64_t execution_time = 0;
int initial_height = 128;
int initial_width = 128;
volatile double time_sec;

volatile uint64_t cycles;
volatile double throughput;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
static inline void cycle_counter_init(void);
static inline uint32_t cycle_counter_get(void);
static inline uint64_t cycle_diff(uint32_t start, uint32_t end);

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
	uint32_t program_start = HAL_GetTick();

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

  uint32_t start, end;


   cycle_counter_init();
   start_time = HAL_GetTick();
   start = DWT->CYCCNT;



   uint32_t pixels = initial_height*initial_width;
   //getting the start cycle to get the cycle count
   uint32_t start_cycle = cycle_counter_get();


    //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
   //checksum = calculate_mandelbrot_fixed_point_arithmetic(initial_width, initial_height, MAX_ITER);
    checksum = calculate_mandelbrot_double(initial_width, initial_height, MAX_ITER);

    //TODO: Record the end time
   end_time = HAL_GetTick();
   end = DWT->CYCCNT;
   uint32_t cycles = end - start;
   time_sec = (double)cycles / SystemCoreClock;





   //getting the end cycle to get the cycle count
   uint32_t end_cycle = cycle_counter_get();

    cycles = cycle_diff(start_cycle,end_cycle);

    //TODO: Calculate the execution time
   execution_time = end_time - start_time;

   throughput = (double)pixels / ((double)execution_time / 1000.0);


    //TODO: Turn on LED 1 to signify the end of the operation
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    //TODO: Hold the LEDs on for a 1s delay
   HAL_Delay(1000);

    //TODO: Turn off the LEDs
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //TODO: Visual indicator: Turn on LED0 to signal processing start
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);


	  //TODO: Benchmark and Profile Performance


	  //TODO: Visual indicator: Turn on LED1 to signal processing start
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);


	  //TODO: Keep the LEDs ON for 2s
	  HAL_Delay(1000);

	  //TODO: Turn OFF LEDs
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  }
  /* USER CODE END 3 */

  uint32_t program_end = HAL_GetTick();
  uint32_t total_runtime_ms = program_end - program_start;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations)
{
    uint64_t mandelbrot_sum = 0;

    // This function utilizes bit shifting to decreases the computational costs of division on decimals

    // For the initial loop, x and y are zero so instead of computing it and lastly subtracting the step,
    // this code sets the initial starting point to the constant
    const int32_t x_left = -( (5 * ONE) / 2 ); // -2.5 as x starting point
    const int32_t y_top  = -ONE;               // -1.0 as y starting point

    // Instead of recomputing x*... and y*..., we just add the multiplication each loop as it would amount to the same thing
    // This way addition uses less computational power than multiplication
    const int32_t x_step = (int32_t)(((int64_t)7 * ONE) / (2 * width));			// The 2 in the denominator makes it 7/2 = 3.5
    const int32_t y_step = (int32_t)(((int64_t)2 * ONE) / height);				// Bit shift the top before dividing

    const int32_t FOUR = 4 * ONE;				// for condition in while loop

    int32_t y0 = y_top;				// starting constant
    for (int y = 0; y < height; ++y) {
        int32_t x0 = x_left;			// starting constant

        for (int x = 0; x < width; ++x) {
            int32_t xi = 0, yi = 0;
            int iteration = 0;

            while (iteration < max_iterations) {
                int32_t xi2 = (int32_t)(((int64_t)xi * xi + (1 << (Q - 1))) >> Q);
                int32_t yi2 = (int32_t)(((int64_t)yi * yi + (1 << (Q - 1))) >> Q);

                // after calculating square values for this loop check if condition is still met
                if ((int64_t)xi2 + yi2 > FOUR) break;

                int32_t temp = xi2 - yi2 + x0;

                // Q is 2^16, so Q-1 = 2^15 to account for 2*xi*yi
                yi = (int32_t)(((int64_t)xi * yi + (1 << (Q - 2))) >> (Q - 1)) + y0;
                xi = temp;           // following mandelbrot.py

                ++iteration;
            }

            mandelbrot_sum += iteration;
            x0 += x_step; // add step instead of multiplying new x value
        }
        y0 += y_step;     // add step instead of multiplying new y value
    }

    return mandelbrot_sum;
}


//TODO: Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations)
{
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation
    checksum = 0;
    for (int y = 0; y < height; y++)
    {
    	for (int x = 0; x < width ; x++)
    	{
    		double x_0 = ((double)x/width)*3.5 - 2.5;
    		double y_0 = (double)y/height * 2.0 - 1.0;

    		double x_i = 0;
    		double y_i = 0;
    		int iteration = 0;


    		while (iteration < max_iterations)
    		{

    			double x_i_sq = x_i*x_i;
    			double y_i_sq = y_i*y_i;

    			if(x_i_sq + y_i_sq > 4.0)
    			{
    				break;
    			}
    			double temp  = x_i_sq - y_i_sq;
    			y_i = 2.0*x_i*y_i + y_0;

    			x_i = temp + x_0;

    			iteration = iteration +1;

    		}

    		mandelbrot_sum = mandelbrot_sum + iteration;

    	}
    }
    //checksum = mandelbrot_sum;
    return mandelbrot_sum;
}



static inline void cycle_counter_init(void)
 {
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  DWT->CYCCNT = 0;
	  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
 }



 static inline uint32_t cycle_counter_get(void)
 {
     return DWT->CYCCNT;
 }

 static inline uint64_t cycle_diff(uint32_t start, uint32_t end)
 {
     return (end >= start) ? (uint64_t)(end - start)
                           : (uint64_t)(0x100000000ULL - start + end);
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
