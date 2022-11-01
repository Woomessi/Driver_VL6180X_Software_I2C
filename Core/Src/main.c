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
#include "vl6180x_api.h" //官方API
#include "bsp_i2c.h" //软件I2C
#include <string.h>
#include <stdlib.h>
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
#define DEVICE_NUMBER 2 //传感器数量
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//VL6180xDev_t theVL6180xDev = 0x52; //传感器设备地址（已将原7位地址左移1位，得到8位地址）

/* 用于printf的条件编译 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar()
#else
#define PUTCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

GETCHAR_PROTOTYPE
{
  uint8_t ch;
  HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//I2C底层通讯函数选择
//#define i2c_bus (&hi2c1)
//#define def_i2c_time_out 100
#if VL6180x_SINGLE_DEVICE_DRIVER
/* VL6180X底层驱动函数：写 */
int VL6180x_I2CWrite(VL6180xDev_t addr, uint8_t *buff, uint8_t len)
{
  int status, i;
  I2C_Start();//开启I2C总线
  I2C_SendByte(addr | 0);//发送8位设备地址及LSB的写方向位，实现写功能
  if (I2C_WaitAck())//如果从机没有应答
  {
    I2C_Stop();//关闭I2C总线
    return 1;//返回报错标志
  }
  for (i = 0; i < len; i++)
  {
    I2C_SendByte(buff[i]);//发送buff中的各字节
    if (i == len - 1)//在发送最后一个字节的数据时
    {
      if (I2C_WaitAck())//等待从机应答
      {
        I2C_Stop();//关闭I2C总线
        status = 1;//返回报错标志
      }
    }
    else
    {
      status = I2C_WaitAck();//返回报错标志
    }
  }
  I2C_Stop();//关闭I2C总线
  return status;
}

/* VL6180X底层驱动函数：读 */
int VL6180x_I2CRead(VL6180xDev_t addr, uint8_t *buff, uint8_t len)
{
  int status, i;
  I2C_Start();//开启I2C总线
  I2C_SendByte(addr | 1);//发送8位设备地址及LSB的读方向位，实现读功能
  I2C_WaitAck();//等待从机应答
  for (i = 0; i < len; i++)
  {
    if (i == len - 1)//在读取最后一个字节的数据时
    {
      buff[i] = I2C_ReadByte(0);//读取数据，将其保存于buff[i]，并向从机发送非应答信号，终止数据读取
    }
    else
    {
      buff[i] = I2C_ReadByte(1);//读取数据，将其保存于buff[i]，并向从机发送应答信号，继续读取数据
    }
  }
  I2C_Stop();//关闭I2C总线
  return status = 0;
}
#else
int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t *buff, uint8_t len)
{
  int status, i;
  I2C_Start();//开启I2C总线
  I2C_SendByte(dev->i2c_dev_addr | 0);//发送8位设备地址及LSB的写方向位，实现写功能
  if (I2C_WaitAck())//如果从机没有应答
  {
    I2C_Stop();//关闭I2C总线
    return 1;//返回报错标志
  }
  for (i = 0; i < len; i++)
  {
    I2C_SendByte(buff[i]);//发送buff中的各字节
    if (i == len - 1)//在发送最后一个字节的数据时
    {
      if (I2C_WaitAck())//等待从机应答
      {
        I2C_Stop();//关闭I2C总线
        status = 1;//返回报错标志
      }
    }
    else
    {
      status = I2C_WaitAck();//返回报错标志
    }
  }
  I2C_Stop();//关闭I2C总线
  return status;
}

/* VL6180X底层驱动函数：读 */
int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len)
{
  int status, i;
  I2C_Start();//开启I2C总线
  I2C_SendByte(dev->i2c_dev_addr | 1);//发送8位设备地址及LSB的读方向位，实现读功能
  I2C_WaitAck();//等待从机应答
  for (i = 0; i < len; i++)
  {
    if (i == len - 1)//在读取最后一个字节的数据时
    {
      buff[i] = I2C_ReadByte(0);//读取数据，将其保存于buff[i]，并向从机发送非应答信号，终止数据读取
    }
    else
    {
      buff[i] = I2C_ReadByte(1);//读取数据，将其保存于buff[i]，并向从机发送应答信号，继续读取数据
    }
  }
  I2C_Stop();//关闭I2C总线
  return status = 0;
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct MyDev_t Devs[DEVICE_NUMBER];
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
  /* USER CODE BEGIN 2 */
  //初始化使能端口
  GPIO_InitTypeDef gpioinit;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpioinit.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  gpioinit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpioinit);

//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
//  uint8_t status;
//  uint8_t *data = 0x200027d9;//初始化指针，为其分配一个空闲的地址
//  status = VL6180x_RdByte(theVL6180xDev, IDENTIFICATION_MODEL_ID, data);//读取设备id，测试I2C驱动是否正常

  SysTick_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, 0);//复位所有的传感器

  int i, id, FinalI2cAddr, status;
  uint16_t GPIO_index[DEVICE_NUMBER] = {GPIO_PIN_12, GPIO_PIN_13};//保存要使能的GPIO端口
  //逐一更新各传感器的地址
  for (i = 0; i <= DEVICE_NUMBER - 1; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_index[i], 1);//使能当前传感器
    HAL_Delay(2);
    Devs[i].i2c_dev_addr = 0x52;//刚刚使能的传感器，访问地址仍为默认地址0x52
    FinalI2cAddr = 0x52 + ((i + 1) * 2);//获取修改后的地址
    status = VL6180x_SetI2CAddress(&Devs[i], FinalI2cAddr); //将修改后的地址写入传感器相关寄存器
    Devs[i].i2c_dev_addr = FinalI2cAddr;//记录修改后的地址
    status = VL6180x_RdByte(&Devs[i], IDENTIFICATION_MODEL_ID, &id);//测试I2C读值是否正常
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    Sample_SimpleRanging();//测距函数
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

void Sample_SimpleRanging(void)
{
  VL6180x_RangeData_t Range[DEVICE_NUMBER];

  int i;
  for (i = 0; i <= DEVICE_NUMBER - 1; i++)
  {
    VL6180x_InitData(&Devs[i]);
    VL6180x_Prepare(&Devs[i]);
    ///* 调整测量范围
    VL6180x_SetGroupParamHold(&Devs[i], 1);
    VL6180x_RangeGetThresholds(&Devs[i], NULL, NULL);
    VL6180x_UpscaleSetScaling(&Devs[i], 3);
    VL6180x_RangeSetThresholds(&Devs[i], 0, 600, 0);
    VL6180x_SetGroupParamHold(&Devs[i], 0);
    //*/
    VL6180x_RangePollMeasurement(&Devs[i], &Range[i]);
    if (Range[i].errorStatus == 0)
    {
      printf("range %d: %ld mm\r\n", i + 1, Range[i].range_mm);
      HAL_Delay(250);
    }
    else
    {
      printf("%s\r\n", "error");
      HAL_Delay(250);
    }
  }
}

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCL_Pin | SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL_Pin | SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
