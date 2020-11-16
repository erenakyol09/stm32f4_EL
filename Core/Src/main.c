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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "i2c-lcd.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define Length 50

//Transmitter Buffer
uint8_t Tx_Buffer[Length];
char 		Rx_Buffer[Length];
char 		Rx_B_Buffer[Length];
char 		Rx_B_Data[Length];
char	  convertFloat[Length];
double e      = 0;
double value  = 0;
double tValue = 0;
double a = 0;
int 	 s = 0;
uint16_t delay = 10;

// C komutu icin senaryo	
uint16_t P     = 200;
uint16_t Vrms  = 220;
uint16_t Irms  = 1;  
uint16_t pf    = 1;
uint16_t f     = 50; 
uint16_t dcCur = 5;  
uint16_t dcVol = 30;	

// A komutu ile kimlik bilgisi gonderimi
void command_A(void)
{
	uint16_t sendPacket[Length];
	
	sendPacket[0] = 1;
	sendPacket[1] = 2;
	sendPacket[2] = 3;
	
	uint8_t packetNumber = 3;
	uint16_t crc  = 0;
	// komut + paket sayisi + paket bytelari + crc
	
	// A komutu gonderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%c\n",'A'),1); 
	HAL_Delay(delay);
	// paket sayi degeri gonderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%d\n",packetNumber),1);
	HAL_Delay(delay);
	// paketin bytelari sirayla gönderilecek
	for(int i=0;i<=packetNumber-1;i++)
	{
		if(i == 0)
			{
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Device Information:\n"),1);		
				HAL_Delay(delay);
				//sendPacket'taki tüm paketler crc degiskeninde toplanacak 		
			}
		else if(i == 1)
			{
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Model:ARM Cortex-M4\n"),1);		
				HAL_Delay(delay);
				//sendPacket'taki tüm paketler crc degiskeninde toplanacak 		
			}
		else
			{
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"VersionNum:STM32F407VG\n"),1);		
				HAL_Delay(delay);
			}
				crc = crc + sendPacket[i]; 
	}
	// crc gönderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%d\n",crc),1);		
	HAL_Delay(delay);
}

// B komutu ile sabit mod secimi
void command_B(char mode,float value)
{
	uint16_t sendPacket[Length];
	
	sendPacket[0] = 1;
	sendPacket[1] = 2;
	
	uint8_t packetNumber = 2;
	uint16_t crc  = 0;
	// komut + paket sayisi + paket bytelari + crc
	
	// A komutu gonderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%c\n",'B'),1); 
	HAL_Delay(delay);
	// paket sayi degeri gonderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%d\n",packetNumber),1);
	HAL_Delay(delay);
	// paketin bytelari sirayla gönderilecek
	for(int i=0;i<=packetNumber-1;i++)
	{
		if(i == 0)
			{
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Selected Mode:%c\n",mode),1);		
				HAL_Delay(delay);
				//sendPacket'taki tüm paketler crc degiskeninde toplanacak 		
			}
		else
			{
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Selected Value:%.3f\n",value),1);		
				HAL_Delay(delay);
			}
				crc = crc + sendPacket[i]; 
	}
	// crc gönderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%d\n",crc),1);		
	HAL_Delay(delay);
}
// Vrms gibi C komutundaki gönderecegimiz degerler globalde tanimli degilse hazirlanan fonksiyon;
void command_C(uint16_t P_c,uint16_t Vrms_c,uint16_t Irms_c,uint16_t pf_c,uint16_t f_c,uint16_t dcCur_c,uint16_t dcVol_c)
{
	// ornegin gucu 300W kabul edelim, 300 > 256 oldugu icin sendPacket 16bitlik olarak ayarlandi
	uint16_t sendPacket[Length];
	uint16_t crc = 0;
	uint8_t packetNumber = 7;
	// buffera olcum sonuc atamasi yapildi, toplam 7 paket gonderilecektir
	sendPacket[0] = P_c;
	sendPacket[1] = Vrms_c;
	sendPacket[2] = Irms_c;
	sendPacket[3] = pf_c;
	sendPacket[4] = f_c;
	sendPacket[5] = dcCur_c;
	sendPacket[6] = dcVol_c;	
	
	
	// komut + paket sayisi + paket bytelari + crc
	
	// C komutu gonderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%c\n",'C'),1); 
	HAL_Delay(delay);
	// paket sayimiz yani 7 degeri gonderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%d\n",packetNumber),1);
	HAL_Delay(delay);
	// paketin bytelari sirayla gönderilecek
	for(int i=0;i <= packetNumber-1;i++)
		{
			HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%d\n",sendPacket[i]),1);		
			HAL_Delay(delay);
			//sendPacket'taki tüm paketler crc degiskeninde toplanacak 		
			crc = crc + sendPacket[i]; 
		}
	// crc gönderilecek
	HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"%d\n",crc),1);		
	HAL_Delay(delay);					
}
void bufferClean(void)
{
	for(int o=0;o<=Length;o++)
		{
			Rx_B_Buffer[o] = 0;
		}	
	for(int o=0;o<=Length;o++)
		{
			Rx_Buffer[o] = 0;
		}	
}

float chartoInt(char x)
{
	switch(x)
	{
		case '0': e = 0 ;break;
		case '1': e = 1 ;break;
		case '2': e = 2 ;break;
		case '3': e = 3 ;break;
		case '4': e = 4 ;break;
		case '5': e = 5 ;break;
		case '6': e = 6 ;break;
		case '7': e = 7 ;break;
		case '8': e = 8 ;break;
		case '9': e = 9 ;break;		
		default:break;
	}
	return e;
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	lcd_send_cmd (0x80|0x40);
	lcd_send_string("This Is Test Code");
	HAL_Delay(1000);
	lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		
		//Arayüzden Komut Bekliyoruz
		HAL_UART_Receive(&huart3,(uint8_t*)Rx_Buffer,Length,1000);			
		HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Waitting For Command\n"),1);
		HAL_Delay(delay);
		lcd_clear();
	  lcd_send_cmd (0x80|0x00);
		lcd_send_string("Enter Command:");
		lcd_send_string(Rx_Buffer);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string("A: Model");
		lcd_send_cmd (0x80|0x14);
		lcd_send_string("B: Mode");
		lcd_send_cmd (0x80|0x54);
		lcd_send_string("C: Efficiency");
		HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Commands:A,B,C\n"),1);
		HAL_Delay(delay);

			


		// arayüzden A komutu geldiginde
		if(Rx_Buffer[0] == 'A')
			{	
				lcd_clear();
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("Selected Command:");
				lcd_send_string(Rx_Buffer);				
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
				command_A();	
				lcd_send_cmd (0x80|0x14);
				lcd_send_string("      -Sended-");
			  lcd_send_cmd (0x80|0x54);
				lcd_send_string("   Model & Version");
				HAL_Delay(3000);
				Rx_Buffer[0] = 0;
			}
				
		// arayüzden B komutu geldiginde		
		if(Rx_Buffer[0] == 'B')
			{	
				lcd_clear();
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("Selected Command:");
				lcd_send_string(Rx_Buffer);
																												//Gelen datalari  Rx_B_Data adli dizide toplayacagiz
				//0. indekse 'B' yazilacak
				Rx_B_Data[0] = Rx_Buffer[0];
				Rx_Buffer[0] = 0;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET);
				
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Select Mode:\n"),1); 
				HAL_Delay(delay);
				
				while(1)
				{
					HAL_UART_Receive(&huart3,(uint8_t*)Rx_B_Buffer,Length,1);

					if(Rx_B_Buffer[0] == 'I' || Rx_B_Buffer[0] == 'R' || Rx_B_Buffer[0] == 'V' || Rx_B_Buffer[0] == 'P')					
					{
						lcd_send_cmd (0x80|0x40);
						lcd_send_string("Selected Mode:");
						lcd_send_string(Rx_B_Buffer);
						break;
					}
					else
					{
						HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Please Select Mode\n"),10); 
						HAL_Delay(100);
						HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Cur:I Res:R\n"),10); 
						HAL_Delay(1000);
						HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Volt:V Pow:P\n"),10); 
						HAL_Delay(1000);
					}
				}
				//1. indekse secilen mod yazilacak
				Rx_B_Data[1] = Rx_B_Buffer[0];
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Selected Mode:\n"),1);
				HAL_Delay(delay);
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Const %c Mode\n",Rx_B_Buffer[0]),1);
				HAL_Delay(delay);
				
				HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Enter Value:\n"),1); 
				HAL_Delay(delay);

				while(1)
				{
					HAL_UART_Receive(&huart3,(uint8_t*)Rx_B_Buffer,Length,1000);
					
					if(Rx_B_Buffer[0] != Rx_B_Data[1])					
					{

						for(int k=0;k<=Length;k++)
							{
								if(Rx_B_Buffer[k] == '.')
									{
										a = k;
									}
							}
							
						s = strlen(Rx_B_Buffer);
						HAL_Delay(1);
						//tam sayi olma durumu							
						if(a == 0)
						{

							for(int k=1;k<=s;k++)
								{									
									value = chartoInt(Rx_B_Buffer[k-1]);
									value = value * pow(10,s-k);
									tValue = tValue + value;								  					
								}
						}
						// tam sayi olmama durumu						
						if(a != 0)
						{
							for(int k=1;k<=s-a-1;k++)
								{									
									value = chartoInt(Rx_B_Buffer[k-1]);
									value = value * pow(10,s-a-k-1);
									tValue = tValue + value;								  					
								}
						  for(int k=a+1;k<=s-1;k++)
								{									
									value = chartoInt(Rx_B_Buffer[k]);
									value = value / pow(10,k-a);
									tValue = tValue + value;								  					
								}
						}
						break;
					}
					else
					{
						HAL_UART_Transmit(&huart3,Tx_Buffer,sprintf((char *)Tx_Buffer,"Please Enter Value\n"),10); 
						HAL_Delay(100);
					}
				}
				
				lcd_send_cmd (0x80|0x14);
				lcd_send_string("Selected Value:");
				lcd_send_cmd (0x80|0x54);
				lcd_send_string(Rx_B_Buffer);
				HAL_Delay(1000);
				command_B(Rx_B_Data[1],tValue);		
				tValue = 0;
				value = 0;
				bufferClean();				
			}	
			
		// arayüzden C komutu geldiginde		
	  if(Rx_Buffer[0] == 'C')
			{	
				lcd_clear();
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("Selected Command:");
				lcd_send_string(Rx_Buffer);
				
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET);		
				
				while(1)
				{
					HAL_UART_Receive(&huart3,(uint8_t*)Rx_Buffer,Length,1);
					
					lcd_send_cmd (0x80|0x14);
					lcd_send_string("  -SENDING DATA-  ");
					lcd_send_cmd (0x80|0x54);
					lcd_send_string("  Send X to exit");
					
					
					if(Rx_Buffer[0] == 'X')					
					{
						break;
					}
					else
					{						
						command_C(P,Vrms,Irms,pf,f,dcCur,dcVol);	
					}
				}
				bufferClean();
			}

    /* USER CODE END WHILE */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  huart3.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
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
