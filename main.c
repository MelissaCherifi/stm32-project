/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LDR_SEUIL        2000U
#define DISTANCE_SEUIL   10U

#define TRIG_PORT GPIOC
#define TRIG_PIN  GPIO_PIN_2

#define ECHO_PORT GPIOC
#define ECHO_PIN  GPIO_PIN_1

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);



/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define AUTH_UID_LEN 4
static const uint8_t AUTH_UID[AUTH_UID_LEN] = { 0xB3,0x0A,0xA0,0xA7 };


#define SESSION_MS           60000U   // 1 minute
#define LOOP_PERIOD_MS       50U      // cadence boucle
#define PRINT_RATE_MS        500U     // anti-spam prints

typedef enum {
  SES_IDLE = 0,
  SES_WAIT_CARD,
  SES_GRANTED,
  SES_DENIED
} session_state_t;


static uint8_t uid_is_authorized(const uint8_t *uid, uint8_t uidLen)
{
  return (uidLen == AUTH_UID_LEN) && (memcmp(uid, AUTH_UID, AUTH_UID_LEN) == 0);
}

static void print_uid_hex(const uint8_t *uid, uint8_t uidLen)
{
  for (uint8_t i = 0; i < uidLen; i++)
  {
    printf("%02X", uid[i]);
  }
}


/* ========= printf -> USART1 ========= */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/* ========= DWT timing (µs) ========= */
static void DWT_Init(void)
{
  /* DWT is available on Cortex-M3/M4/M7 (check your core) */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks) { }
}

static uint32_t micros(void)
{
  return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

/* ========= Ultrasonic (HC-SR04) ========= */
static uint32_t Ultrasonic_ReadDistanceCm(void)
{
  /* Trigger: impulsion 10µs */
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  /* attendre ECHO HIGH (timeout 30ms) */
  uint32_t t0 = micros();
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
  {
    if ((micros() - t0) > 30000U) return 0;
  }

  /* mesurer largeur impulsion HIGH */
  uint32_t start = micros();
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
  {
    if ((micros() - start) > 30000U) return 0;
  }
  uint32_t pulse_us = micros() - start;

  /* distance (cm) ~ pulse_us / 58 */
  return (pulse_us / 58U);
}

/* =========================================================
   ====================== RC522 (MFRC522) ===================
   ========================================================= */

/* ====== RC522 pin mapping ====== */
#define RC522_CS_PORT   GPIOA
#define RC522_CS_PIN    GPIO_PIN_4   // PA4

#define RC522_RST_PORT  GPIOA
#define RC522_RST_PIN   GPIO_PIN_2   // PA2

#define RC522_CS_LOW()     HAL_GPIO_WritePin(RC522_CS_PORT, RC522_CS_PIN, GPIO_PIN_RESET)
#define RC522_CS_HIGH()    HAL_GPIO_WritePin(RC522_CS_PORT, RC522_CS_PIN, GPIO_PIN_SET)
#define RC522_RST_LOW()    HAL_GPIO_WritePin(RC522_RST_PORT, RC522_RST_PIN, GPIO_PIN_RESET)
#define RC522_RST_HIGH()   HAL_GPIO_WritePin(RC522_RST_PORT, RC522_RST_PIN, GPIO_PIN_SET)

/* ===== RC522 registers ===== */
#define RC522_Reg_Command       0x01
#define RC522_Reg_ComIEn        0x02
#define RC522_Reg_ComIrq        0x04
#define RC522_Reg_DivIrq        0x05
#define RC522_Reg_Error         0x06
#define RC522_Reg_FIFOData      0x09
#define RC522_Reg_FIFOLevel     0x0A
#define RC522_Reg_Control       0x0C
#define RC522_Reg_BitFraming    0x0D
#define RC522_Reg_Coll          0x0E
#define RC522_Reg_Mode          0x11
#define RC522_Reg_TxControl     0x14
#define RC522_Reg_TxASK         0x15
#define RC522_Reg_TMode         0x2A
#define RC522_Reg_TPrescaler    0x2B
#define RC522_Reg_TReloadH      0x2C
#define RC522_Reg_TReloadL      0x2D
#define RC522_Reg_Version       0x37

#define RC522_Reg_CRCResultH    0x21
#define RC522_Reg_CRCResultL    0x22

/* ===== RC522 commands ===== */
#define PCD_Idle        0x00
#define PCD_CalcCRC     0x03
#define PCD_Transceive  0x0C
#define PCD_SoftReset   0x0F

/* ===== PICC commands ===== */
#define PICC_CMD_REQA     0x26
#define PICC_CMD_SEL_CL1  0x93
#define PICC_CMD_SEL_CL2  0x95
#define PICC_CMD_SEL_CL3  0x97
#define PICC_CMD_HLTA     0x50
#define PICC_CMD_WUPA 0x52

/* ===== status ===== */
#define MI_OK       0
#define MI_NOTAGERR 1
#define MI_ERR      2

/* ===== SPI low-level ===== */
static uint8_t RC522_SPI_Transfer(uint8_t data)
{
  uint8_t rx = 0;
  HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
  return rx;
}

static void RC522_WriteReg(uint8_t reg, uint8_t value)
{
  RC522_CS_LOW();
  RC522_SPI_Transfer((reg << 1) & 0x7E);  // write
  RC522_SPI_Transfer(value);
  RC522_CS_HIGH();
}

static uint8_t RC522_ReadReg(uint8_t reg)
{
  uint8_t value;
  RC522_CS_LOW();
  RC522_SPI_Transfer(((reg << 1) & 0x7E) | 0x80); // read
  value = RC522_SPI_Transfer(0x00);
  RC522_CS_HIGH();
  return value;
}

static void RC522_SetBitMask(uint8_t reg, uint8_t mask)
{
  RC522_WriteReg(reg, RC522_ReadReg(reg) | mask);
}

static void RC522_ClearBitMask(uint8_t reg, uint8_t mask)
{
  RC522_WriteReg(reg, RC522_ReadReg(reg) & (uint8_t)(~mask));
}

/* ===== Antenna ===== */
static void RC522_AntennaOn(void)
{
  uint8_t v = RC522_ReadReg(RC522_Reg_TxControl);
  if ((v & 0x03U) != 0x03U)
    RC522_WriteReg(RC522_Reg_TxControl, v | 0x03U);
}

/* ===== CRC (chip) ===== */
static uint8_t RC522_CalcCRC(uint8_t *data, uint8_t len, uint8_t *outL, uint8_t *outH)
{
  RC522_WriteReg(RC522_Reg_Command, PCD_Idle);
  RC522_SetBitMask(RC522_Reg_FIFOLevel, 0x80);

  for (uint8_t i = 0; i < len; i++)
    RC522_WriteReg(RC522_Reg_FIFOData, data[i]);

  RC522_WriteReg(RC522_Reg_Command, PCD_CalcCRC);

  uint16_t i = 0xFFFF;
  uint8_t n;
  do {
    n = RC522_ReadReg(RC522_Reg_DivIrq);
    i--;
  } while (i && !(n & 0x04U));

  if (!i) return MI_ERR;

  *outL = RC522_ReadReg(RC522_Reg_CRCResultL);
  *outH = RC522_ReadReg(RC522_Reg_CRCResultH);
  return MI_OK;
}

/* ===== Transceive ===== */
static uint8_t RC522_Transceive(uint8_t *sendData, uint8_t sendLen,
                                uint8_t *backData, uint16_t *backBits)
{
  uint8_t n, lastBits;
  uint16_t i;

  RC522_WriteReg(RC522_Reg_ComIEn, 0x77U | 0x80U);
  RC522_ClearBitMask(RC522_Reg_ComIrq, 0x80U);
  RC522_SetBitMask(RC522_Reg_FIFOLevel, 0x80U);
  RC522_WriteReg(RC522_Reg_Command, PCD_Idle);

  for (i = 0; i < sendLen; i++)
    RC522_WriteReg(RC522_Reg_FIFOData, sendData[i]);

  RC522_WriteReg(RC522_Reg_Command, PCD_Transceive);
  RC522_SetBitMask(RC522_Reg_BitFraming, 0x80U); // StartSend

  i = 2000;
  do {
    n = RC522_ReadReg(RC522_Reg_ComIrq);
    i--;
  } while (i && !(n & 0x01U) && !(n & 0x30U));

  RC522_ClearBitMask(RC522_Reg_BitFraming, 0x80U);

  if (!i) return MI_ERR;
  if (RC522_ReadReg(RC522_Reg_Error) & 0x1BU) return MI_ERR;
  if (n & 0x01U) return MI_NOTAGERR;

  uint8_t fifoLevel = RC522_ReadReg(RC522_Reg_FIFOLevel);
  lastBits = RC522_ReadReg(RC522_Reg_Control) & 0x07U;

  if (lastBits) *backBits = (uint16_t)((fifoLevel - 1U) * 8U + lastBits);
  else *backBits = (uint16_t)(fifoLevel * 8U);

  for (i = 0; i < fifoLevel; i++)
    backData[i] = RC522_ReadReg(RC522_Reg_FIFOData);

  return MI_OK;
}

/* ===== REQA ===== */
static uint8_t RC522_RequestA(uint8_t *atqa)
{
  uint8_t cmd = PICC_CMD_REQA;
  uint16_t backBits = 0;

  RC522_WriteReg(RC522_Reg_BitFraming, 0x07U); // 7-bit
  uint8_t st = RC522_Transceive(&cmd, 1, atqa, &backBits);
  if (st != MI_OK || backBits != 16U) return MI_ERR;
  return MI_OK;
}

static uint8_t RC522_AnticollLevel(uint8_t selCode, uint8_t out5[5])
{
  uint8_t cmd[2] = { selCode, 0x20U };
  uint8_t back[10] = {0};
  uint16_t backBits = 0;

  RC522_WriteReg(RC522_Reg_BitFraming, 0x00U);
  RC522_ClearBitMask(RC522_Reg_Coll, 0x80U);

  uint8_t st = RC522_Transceive(cmd, 2, back, &backBits);
  if (st != MI_OK || backBits != 40U) return MI_ERR;

  for (int i = 0; i < 5; i++) out5[i] = back[i];

  uint8_t bcc = out5[0] ^ out5[1] ^ out5[2] ^ out5[3];
  if (bcc != out5[4]) return MI_ERR;

  return MI_OK;
}

static uint8_t RC522_SelectLevel(uint8_t selCode, uint8_t uidIn5[5], uint8_t *sakOut)
{
  uint8_t buf[9];
  buf[0] = selCode;
  buf[1] = 0x70U;
  for (int i = 0; i < 5; i++) buf[2 + i] = uidIn5[i];

  uint8_t crcL, crcH;
  if (RC522_CalcCRC(buf, 7, &crcL, &crcH) != MI_OK) return MI_ERR;
  buf[7] = crcL;
  buf[8] = crcH;

  uint8_t back[5] = {0};
  uint16_t backBits = 0;
  uint8_t st = RC522_Transceive(buf, 9, back, &backBits);

  /* SAK (1) + CRC (2) => 24 bits */
  if (st != MI_OK || backBits != 24U) return MI_ERR;
  *sakOut = back[0];
  return MI_OK;
}

static void RC522_HaltA(void)
{
  uint8_t buf[4];
  buf[0] = PICC_CMD_HLTA;
  buf[1] = 0x00U;

  uint8_t crcL, crcH;
  if (RC522_CalcCRC(buf, 2, &crcL, &crcH) != MI_OK) return;
  buf[2] = crcL;
  buf[3] = crcH;

  uint8_t back[4];
  uint16_t backBits;
  (void)RC522_Transceive(buf, 4, back, &backBits);
}

/**
 * Read UID full length (4/7/10 bytes)
 * uid buffer must be >= 10 bytes.
 */
static uint8_t RC522_ReadUID(uint8_t uid[10], uint8_t *uidLen)
{
  uint8_t atqa[2];
  if (RC522_RequestA(atqa) != MI_OK) return MI_NOTAGERR;

  uint8_t part[5];
  uint8_t sak;

  /* CL1 */
  if (RC522_AnticollLevel(PICC_CMD_SEL_CL1, part) != MI_OK) return MI_ERR;
  if (RC522_SelectLevel(PICC_CMD_SEL_CL1, part, &sak) != MI_OK) return MI_ERR;

  if (part[0] == 0x88U)
  {
    /* UID not complete, keep 3 bytes from CL1 */
    uid[0] = part[1];
    uid[1] = part[2];
    uid[2] = part[3];

    /* CL2 */
    if (RC522_AnticollLevel(PICC_CMD_SEL_CL2, part) != MI_OK) return MI_ERR;
    if (RC522_SelectLevel(PICC_CMD_SEL_CL2, part, &sak) != MI_OK) return MI_ERR;

    if (part[0] == 0x88U)
    {
      /* UID 10 bytes: take 3 bytes from CL2 and 4 from CL3 */
      uid[3] = part[1];
      uid[4] = part[2];
      uid[5] = part[3];

      /* CL3 */
      if (RC522_AnticollLevel(PICC_CMD_SEL_CL3, part) != MI_OK) return MI_ERR;
      if (RC522_SelectLevel(PICC_CMD_SEL_CL3, part, &sak) != MI_OK) return MI_ERR;

      uid[6] = part[0];
      uid[7] = part[1];
      uid[8] = part[2];
      uid[9] = part[3];
      *uidLen = 10;
    }
    else
    {
      /* UID 7 bytes: take 4 bytes from CL2 */
      uid[3] = part[0];
      uid[4] = part[1];
      uid[5] = part[2];
      uid[6] = part[3];
      *uidLen = 7;
    }
  }
  else
  {
    /* UID 4 bytes */
    uid[0] = part[0];
    uid[1] = part[1];
    uid[2] = part[2];
    uid[3] = part[3];
    *uidLen = 4;
  }

  //RC522_HaltA();
  return MI_OK;
}

static void RC522_Init(void)
{
  RC522_CS_HIGH();

  RC522_RST_LOW();
  HAL_Delay(50);
  RC522_RST_HIGH();
  HAL_Delay(50);

  RC522_WriteReg(RC522_Reg_Command, PCD_SoftReset);
  HAL_Delay(50);

  /* Timer settings (common values used in many examples) */
  RC522_WriteReg(RC522_Reg_TMode, 0x8DU);
  RC522_WriteReg(RC522_Reg_TPrescaler, 0x3EU);
  RC522_WriteReg(RC522_Reg_TReloadL, 30U);
  RC522_WriteReg(RC522_Reg_TReloadH, 0U);

  RC522_WriteReg(RC522_Reg_TxASK, 0x40U);
  RC522_WriteReg(RC522_Reg_Mode, 0x3DU);

  RC522_AntennaOn();
}

static uint8_t RC522_GetVersion(void)
{
  return RC522_ReadReg(RC522_Reg_Version);
}

/* USER CODE END 0 */





/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  printf("UART1 OK\r\n");

  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  DWT_Init();

  printf("\r\n--- Boot ---\r\n");
  RC522_Init();
  HAL_Delay(50);

  uint8_t ver = RC522_GetVersion();
  printf("RC522 VersionReg = 0x%02X\r\n", ver);
  if (ver == 0x00 || ver == 0xFF)
  {
    printf("ERROR: RC522 not detected. Check SPI/CS/RST wiring and 3.3V power.\r\n");
  }
  printf("Approche une carte RFID...\r\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */






#define AUTH_HOLD_MS     1500U   // garde l'autorisation 1.5s après un badge OK
#define INTRUS_DELAY_MS  2000U   // 2s en mode intrus

static uint32_t auth_until_ms = 0;
static uint32_t last_intrus_ms = 0;


/* ===== variables persistantes (mettre avant while) ===== */
static uint32_t prev_distance = 0;

static session_state_t ses_state = SES_IDLE;
static uint32_t ses_until_ms = 0;
static uint32_t last_print_ms = 0;

/* anti-spam UID (évite d'imprimer 100x si carte laissée) */
static uint8_t last_uid[10] = {0};
static uint8_t last_uid_len = 0;
static uint8_t card_was_present = 0;

while (1)
{
  uint32_t now = HAL_GetTick();

  /* ===== 1) LDR (ADC) ===== */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  uint8_t darkness = (adc_value < LDR_SEUIL) ? 1U : 0U;

  /* ===== 2) Ultrason + mouvement ===== */
  uint32_t curr_distance = Ultrasonic_ReadDistanceCm();
  uint8_t mouvement = 0U;

  if (curr_distance != 0U)
  {
    if (prev_distance == 0U)
    {
      prev_distance = curr_distance;
    }
    else
    {
      int32_t diff = (int32_t)curr_distance - (int32_t)prev_distance;
      if (diff < 0) diff = -diff;
      if ((uint32_t)diff > DISTANCE_SEUIL) mouvement = 1U;
      prev_distance = curr_distance;
    }
  }

  /* ===== 3) Session : déclenchement/prolongation ===== */
  if (mouvement && darkness)
  {
    if (ses_state == SES_IDLE)
    {
      ses_state = SES_WAIT_CARD;
      last_print_ms = 0;

      memset(last_uid, 0, sizeof(last_uid));
      last_uid_len = 0;
      card_was_present = 0;
    }
    ses_until_ms = now + SESSION_MS;  // relance fenêtre 60s
  }

  /* Expiration session */
  if ((ses_state != SES_IDLE) && (now >= ses_until_ms))
  {
    ses_state = SES_IDLE;

    memset(last_uid, 0, sizeof(last_uid));
    last_uid_len = 0;
    card_was_present = 0;
  }

  /* LED : ON pendant session */
  HAL_GPIO_WritePin(LED_LDR_GPIO_Port, LED_LDR_Pin,
                    (ses_state != SES_IDLE) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* ===== 4) RFID : uniquement pendant session ===== */
  if (ses_state != SES_IDLE)
  {
    uint8_t uid[10];
    uint8_t uidLen = 0;
    uint8_t st = RC522_ReadUID(uid, &uidLen);
    uint8_t card_present = (st == MI_OK) ? 1U : 0U;

    /* évènement "nouvelle carte" (ou UID différent) */
    uint8_t new_card_event = 0U;
    if (card_present)
    {
      if (!card_was_present) new_card_event = 1U;
      else if (uidLen != last_uid_len || memcmp(uid, last_uid, uidLen) != 0) new_card_event = 1U;
    }

    if (card_present && new_card_event)
    {
      memcpy(last_uid, uid, uidLen);
      last_uid_len = uidLen;

      printf("UID = ");
      print_uid_hex(uid, uidLen);
      printf(" (len=%u)\r\n", uidLen);

      /* ENTREE SI ET SEULEMENT SI UID = B30AA0A7 et len=4 */
      if ((uidLen == AUTH_UID_LEN) && (memcmp(uid, AUTH_UID, AUTH_UID_LEN) == 0))
      {
        ses_state = SES_GRANTED;
        printf("Entrez\r\n");
      }
      else
      {
        ses_state = SES_DENIED;
        printf("Intrusion\r\n");
      }

      last_print_ms = now;
    }

    /* ré-armement quand la carte est retirée */
    card_was_present = card_present;
  }

  /* ===== 5) Affichage périodique (anti-spam) ===== */
  if (ses_state != SES_IDLE)
  {
    if ((now - last_print_ms) >= PRINT_RATE_MS)
    {
      last_print_ms = now;

      if (ses_state == SES_WAIT_CARD)      printf("Presentez votre carte svp\r\n");
      else if (ses_state == SES_GRANTED)   printf("Entrez\r\n");
      else                                 printf("Intrusion\r\n");
    }
  }

  HAL_Delay(LOOP_PERIOD_MS);
}







   /* USER CODE END WHILE */

   /* USER CODE BEGIN 3 */
     /* rien */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_LDR_Pin|ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin
                          |LED2_Pin|SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LDR_Pin ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin
                           LED2_Pin SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = LED_LDR_Pin|ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin
                          |LED2_Pin|SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
