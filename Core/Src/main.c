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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lora.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_byte;  // byte recibido por interrupcion
char rx_line[64]; // linea armada
volatile uint8_t rx_idx = 0;
volatile uint8_t line_ready = 0; // 1 = hay una linea completa
char tx_buf[100];
// Se llama cada vez que llega 1 byte por USART2

LoRa myLoRa;
uint8_t Lora_stat = 0;
char buf_tx[100];
char buf_rx[128];
uint8_t send_data[9] = {0x00, 0x01, 0x05, 0x02, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8_t received_data[16]; // Buffer m�s grande para recibir comandos del maestro
uint8_t packet_size;
volatile uint8_t data_received_flag = 0; // Flag para indicar que se recibieron datos
uint32_t last_receive_time = 0;          // Timestamp de la �ltima recepci�n

// Variables para validaci�n de datos
uint8_t last_valid_packet[16];
// uint8_t packet_count = 0;
// uint8_t valid_packet_count = 0;
// uint8_t corrupted_packet_count = 0;
uint32_t last_valid_time = 0;

// Configuraci�n del esclavo
const uint8_t ESCLAVO_ID = 0x01; // ID de este esclavo
const uint8_t MAESTRO_ID = 0x00; // ID del maestro

// Variables para la trama:
uint8_t voltaje = 3;
uint8_t corriente = 2;
uint8_t potencia = 6;
uint8_t temperatura = 30;

// Declaracion de funciones
uint8_t is_valid_packet(uint8_t *data, uint8_t length);
void procesar_comando_esclavo(uint8_t *buffer, uint8_t length);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (rx_byte == '\n')
    {
      if (rx_idx > 0)
      {
        rx_line[rx_idx] = '\0';
        line_ready = 1;
      }
      rx_idx = 0;
    }
    else if (rx_byte != '\r' && rx_idx < sizeof(rx_line) - 1)
    {
      rx_line[rx_idx++] = rx_byte;
    }

    // Volver a habilitar la recepcion del proximo byte
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // arrancar recepcion
  myLoRa = newLoRa();

  myLoRa.CS_port = NSS_GPIO_Port;
  myLoRa.CS_pin = NSS_Pin;
  myLoRa.reset_port = RST_GPIO_Port;
  myLoRa.reset_pin = RST_Pin;
  myLoRa.DIO0_port = DIO0_GPIO_Port;
  myLoRa.DIO0_pin = DIO0_Pin;
  myLoRa.hSPIx = &hspi1;

  myLoRa.frequency = 434;             // 433.92;             // default = 433 MHz
  myLoRa.spredingFactor = SF_7;       // Cambiar a SF_8 para mejor inmunidad a ruido
  myLoRa.bandWidth = BW_125KHz;       // default = BW_125KHz
  myLoRa.crcRate = CR_4_5;            // Cambiar a CR_4_8 para mejor correcci�n de errores
  myLoRa.power = POWER_17db;          // Reducir potencia para evitar interferencias
  myLoRa.overCurrentProtection = 100; // default = 100 mA
  myLoRa.preamble = 12;               // Aumentar pre�mbulo para mejor sincronizaci�n

  // LoRa_reset(&myLoRa);

  LoRa_reset(&myLoRa);

  if (LoRa_init(&myLoRa) == LORA_OK)
  {
    sprintf(buf_tx, "LoRa initialized successfully\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);
    Lora_stat = 1;
  }
  else
  {
    sprintf(buf_tx, "LoRa initialization failed\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);
    Lora_stat = 0;
  }

  // START CONTINUOUS RECEIVING -----------------------------------
  LoRa_startReceiving(&myLoRa);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (data_received_flag)
    {
      data_received_flag = 0; // Limpiar flag

      // Intentar recibir los datos
      packet_size = LoRa_receive(&myLoRa, received_data, sizeof(received_data)); // HAY Q DESCOMENTARLO DESPUES

      if (packet_size > 0)
      {
        // Mostrar datos recibidos
        sprintf(buf_tx, "Received data (%d bytes): ", packet_size);
        for (int i = 0; i < packet_size; i++)
        {
          sprintf(buf_tx + strlen(buf_tx), "%02d ", received_data[i]);
        }
        sprintf(buf_tx + strlen(buf_tx), "\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);

        // Validar el paquete
        if (received_data[0] == 1)
        {
          // Enviar trama de datos:
          uint8_t respuesta[10];
          respuesta[0] = voltaje;
          respuesta[1] = corriente;
          respuesta[2] = potencia;
          respuesta[3] = temperatura;

          sprintf(buf_tx, "Enviando respuesta al maestro...\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);

          LoRa_transmit(&myLoRa, respuesta, 4, 2000);
          LoRa_startReceiving(&myLoRa);

          sprintf(buf_tx, "Respuesta enviada correctamente \r\n");
          HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);
        }
      }
      else
      {
        sprintf(buf_tx, "Interrupcion pero sin datos validos\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (line_ready)
    {
      line_ready = 0;

      int n = snprintf(tx_buf, sizeof(tx_buf), "CHAU (Datos recibidos: %d)\r\n", rx_line);
      HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, n, 100);

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // parpadea el LED
    }
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

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  // sprintf(buf_tx, "DIO0 interrupt - Data received\r\n");
  // HAL_UART_Transmit(&huart1, (uint8_t*)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);
  if (GPIO_Pin == DIO0_Pin)
  {
    // Marcar que se recibieron datos
    data_received_flag = 1;
    last_receive_time = HAL_GetTick();

    sprintf(buf_tx, "DIO0 interrupt - Data received\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);
  }
}

// Funci�n para validar si un paquete es v�lido
uint8_t is_valid_packet(uint8_t *data, uint8_t length)
{
  // Validaciones b�sicas
  if (length < 3)
    return 0; // M�nimo 3 bytes

  // Formato del maestro: ID_ESCLAVO, ID_MAESTRO, COMANDO
  // Verificar si es un comando del maestro dirigido a este esclavo
  if (data[0] == 0x01) // ID de este esclavo (0x01)
  {
    if (data[1] == 0x00) // ID del maestro (0x00)
    {
      // Validar comando (deber�a ser un comando conocido)
      if (data[2] >= 0x01 && data[2] <= 0x0F) // Comandos v�lidos
      {
        return 1; // Paquete v�lido del maestro
      }
    }
  }

  // Verificar si es una respuesta de otro esclavo al maestro
  if (data[0] >= 0x02 && data[0] <= 0x0F) // ID de otro esclavo
  {
    if (data[1] == 0x00) // Dirigido al maestro
    {
      return 1; // Posible respuesta v�lida de otro esclavo
    }
  }

  return 0; // Paquete no v�lido
}
void procesar_comando_esclavo(uint8_t *buffer, uint8_t length)
{

  // Enviar respuesta
  uint8_t respuesta[10];
  respuesta[0] = MAESTRO_ID; // Para el maestro
  respuesta[1] = ESCLAVO_ID; // De este esclavo
  respuesta[2] = 0x05;       // CMD_RESPUESTA_DATOS
  sprintf(buf_tx, "Enviando respuesta al maestro...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);

  LoRa_transmit(&myLoRa, respuesta, 10, 2000);
  LoRa_startReceiving(&myLoRa);

  sprintf(buf_tx, "Respuesta enviada correctamente \r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)buf_tx, strlen(buf_tx), HAL_MAX_DELAY);
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
