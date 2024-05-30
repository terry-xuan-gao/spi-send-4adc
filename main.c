/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//用户头文件
#include "sys.h"
#include "delay.h"
#include "24l01.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//私有类型定义

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//私有定义

#define a ox1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//私有宏

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//私有变量
u8 tmp_buf[20]= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19};
u8 rv_data[32];
u8 mode=0;//模式选择 0发送模式 1接收模式
u8 status_tx=0;
u8 status_rx=0;
uint16_t ADC_Value[120];
uint16_t adc_aver[10];
uint16_t sumref7=0,sumref15=0;
uint16_t count1=0,count2=0;
	
uint32_t count_send_fail=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//私有函数原型

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

    uint8_t temp[1]= {0x11};
    HAL_UART_Transmit(&huart1,temp,1,2);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//初始化nrf
    NRF24L01_Init();  //初始化NRF24L01
    if(NRF24L01_Check())//函数返回值为1说明检测24L01错误	！！！
    {
        printf("\n\r nrf测试失败 nrf未连接\n\r");
    }
		
//初始化adc相关
    HAL_ADCEx_Calibration_Start(&hadc1);//校准
    HAL_Delay(10);
    if(HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK) 
		printf("\r\n******** 定时器打开失败 ********\r\n\r\n");
    //开启dma传输，传输60个值 每10个取平均值，共6个有效数据
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 120);

    if(NRF24L01_Check()==0)
    {
        printf("\n\r nrf测试成功 nrf连接\n\r");
			  SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_16);
        //Rx接收模式
        if(mode == 1)
        {
            printf("\n\r 接收模式\n\r");
            for(int i=0; i<34; i++)
            {
                NRF24L01_RX_Mode();//将射频模块配置为接收模式
                status_rx  = NRF24L01_RxPacket(tmp_buf);//一次传输32个字节。接收的数据保存在tmp_buf数组中。返回值0为接收成功
                HAL_Delay (1000);
            }
        }
		//发送模式
        if(mode == 0)
        {
            printf("\n\r 发送模式\n\r");
            NRF24L01_TX_Mode();//将射频模块配置为发射模式
        }
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//DMA传输完成中断回调函数   没有开启dma中断  耗时
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//对adc转换得到的值取平均 由于乘法运算时间太长，这里只把对应的10个值累加。
	
//	for(int i=0,l=0,k=0;i<16;i++)
//	{
//				adc_aver[i]=ADC_Value[k+l]+ADC_Value[k+4+l]+ADC_Value[k+8+l]+ADC_Value[k+12+l]+ADC_Value[k+16+l]
//		+ADC_Value[k+20+l]+ADC_Value[k+24+l]+ADC_Value[k+28+l]+ADC_Value[k+32+l]+ADC_Value[k+36+l];
//				k++;
//				if(k==4)
//				{
//						l+=40;
//						k=0;
//				}		
//	}	
				

	// 仍然是10次求和，但没有加入参考电压（即 sumref7 sumref15）和 7000
	// by 高璇
		for(int i = 0; i < 4; i ++)
		{
			for(int j = 0; j < 10; j ++)
			{
				adc_aver[i] += ADC_Value[i + j*6];
				adc_aver[i+4] += ADC_Value[i + 60 + j*6];
			}
			
			// 加和后除10取平均
			adc_aver[i] /= 10;
			adc_aver[i+4] /= 10;
		}
			
		//这里每个求和值都+7000，是为了避免出现负值。
//			adc_aver[0]=ADC_Value[0]+ADC_Value[6]+ADC_Value[12]+ADC_Value[18]+ADC_Value[24]+ADC_Value[30]+ADC_Value[36]
//					+ADC_Value[42]+ADC_Value[48]+ADC_Value[54]-sumref7+7000;//Fz
//			adc_aver[1]=ADC_Value[1]+ADC_Value[7]+ADC_Value[13]+ADC_Value[19]+ADC_Value[25]+ADC_Value[31]+ADC_Value[37]
//					+ADC_Value[43]+ADC_Value[49]+ADC_Value[55]-sumref15+7000;//Fx
//			adc_aver[2]=ADC_Value[2]+ADC_Value[8]+ADC_Value[14]+ADC_Value[20]+ADC_Value[26]+ADC_Value[32]+ADC_Value[38]
//					+ADC_Value[44]+ADC_Value[50]+ADC_Value[56]-sumref15+7000;//Fy
//			adc_aver[3]=ADC_Value[3]+ADC_Value[9]+ADC_Value[15]+ADC_Value[21]+ADC_Value[27]+ADC_Value[33]+ADC_Value[39]
//					+ADC_Value[45]+ADC_Value[51]+ADC_Value[57]-sumref7+7000;//T
//					
//			sumref7=ADC_Value[4]+ADC_Value[10]+ADC_Value[16]+ADC_Value[22]+ADC_Value[28]+ADC_Value[34]+ADC_Value[40]
//					+ADC_Value[46]+ADC_Value[52]+ADC_Value[58];//参考电压0.7的十次求和
//			sumref15=ADC_Value[5]+ADC_Value[11]+ADC_Value[17]+ADC_Value[23]+ADC_Value[29]+ADC_Value[35]+ADC_Value[41]
//					+ADC_Value[47]+ADC_Value[53]+ADC_Value[59];//参考电压1.5的十次求和
//					
//			
//			adc_aver[4]=ADC_Value[60]+ADC_Value[66]+ADC_Value[72]+ADC_Value[78]+ADC_Value[84]+ADC_Value[90]+ADC_Value[96]
//					+ADC_Value[102]+ADC_Value[108]+ADC_Value[114]-sumref7+7000;//Fz
//			adc_aver[5]=ADC_Value[61]+ADC_Value[67]+ADC_Value[73]+ADC_Value[79]+ADC_Value[85]+ADC_Value[91]+ADC_Value[97]
//					+ADC_Value[103]+ADC_Value[109]+ADC_Value[115]-sumref15+7000;//Fx
//			adc_aver[6]=ADC_Value[62]+ADC_Value[68]+ADC_Value[74]+ADC_Value[80]+ADC_Value[86]+ADC_Value[92]+ADC_Value[98]
//					+ADC_Value[104]+ADC_Value[110]+ADC_Value[116]-sumref15+7000;//Fy
//			adc_aver[7]=ADC_Value[63]+ADC_Value[69]+ADC_Value[75]+ADC_Value[81]+ADC_Value[87]+ADC_Value[93]+ADC_Value[99]
//					+ADC_Value[105]+ADC_Value[111]+ADC_Value[117]-sumref7+7000;//T
//					
//			sumref7=ADC_Value[64]+ADC_Value[70]+ADC_Value[76]+ADC_Value[82]+ADC_Value[88]+ADC_Value[94]+ADC_Value[100]
//					+ADC_Value[106]+ADC_Value[112]+ADC_Value[118];//参考电压0.7的十次求和
//			sumref15=ADC_Value[65]+ADC_Value[71]+ADC_Value[77]+ADC_Value[83]+ADC_Value[89]+ADC_Value[95]+ADC_Value[101]
//					+ADC_Value[107]+ADC_Value[113]+ADC_Value[119];//参考电压1.5的十次求和

//			adc_aver[8]=ADC_Value[118];//ref0.7参考电压的单次测量值
//			adc_aver[9]=ADC_Value[119];//ref1.5参考电压的单次测量值

				
	//adc_aver数组：1-4：z/x/y/T的十次测量值的和；5-8：z/x/y/T的十次测量值的和；9：ref0.7的单次测量值；10：ref1.5的单次测量值
	status_tx=NRF24L01_TxPacket((uint8_t *)adc_aver);//nrf射频发送数据


//	status_tx=NRF24L01_TxPacket((uint8_t *)tmp_buf);//nrf射频发送数据
	//输出发送状态  调试状态下才使用
//	printf("z-%d,x-%d,y-%d,T-%d  ",adc_aver[0],adc_aver[1],adc_aver[2],adc_aver[3]);

//  if(status_tx!=0x20)
//  {
//     count_send_fail++;
//  }  
//	else {count1++;}     

}
/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
//定时器全局中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    /* Prevent unused argument(s) compilation warning */
	//每隔一段时间输出发送成功次数 调试状态下才使用
//    if(htim->Instance==TIM3)
//    {   
//        count2++;
//		}
//		if(count2==3000)//3s输出一次丢包个数
//    {
//				printf("shibai %d.%d",count_send_fail,count1); //count1是发送成功的次数
//				count_send_fail=0;
//        count2=0;
//		}

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
     */
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
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
