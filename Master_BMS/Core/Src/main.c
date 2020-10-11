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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//first SLAVEBMSA(14cell)
float C101, C102, C103, C104, C105, C106, C107, C108, C109, C110, C111, C112, C113, C114;
//second SLAVEBMSB(14cell)
float C201, C202, C203, C204, C205, C206, C207, C208, C209, C210, C211, C212, C213, C214;
//third SLAVEBMSC(14cell)
float C301, C302, C303, C304, C305, C306, C307, C308, C309, C310, C311, C312, C313, C314;
//firth SLAVEBMSD(14cell)
float C401, C402, C403, C404, C405, C406, C407, C408, C409, C410, C411, C412, C413, C414;
//fifth SLAVEBMSE(14cell)
float C501, C502, C503, C504, C505, C506, C507, C508, C509, C510, C511, C512, C513, C514;

uint32_t Temp[70];
uint16_t Max_temp, Min_temp;
uint16_t temp00 ,temp01 ,temp02 ,temp03 ,temp04 ,temp05 ,temp06 ,temp07 ,temp08 ,temp09 ,temp10 ,temp11 ,temp12 ,temp13 ,temp14 ,temp15 ,temp16 ,temp17 ,temp18 ,temp19 ,temp20;
uint16_t temp21 ,temp22 ,temp23 ,temp24 ,temp25 ,temp26 ,temp27 ,temp28 ,temp29 ,temp30 ,temp31 ,temp32 ,temp33, temp34 ,temp35 ,temp36 ,temp37 ,temp38 ,temp39 ,temp40;
uint16_t temp41 ,temp42 ,temp43 ,temp44 ,temp45 ,temp46 ,temp47 ,temp48 ,temp49 ,temp50 ,temp51 ,temp52 ,temp53 ,temp54 ,temp55 ,temp56 ,temp57 ,temp58 ,temp59 ,temp60;
uint16_t temp61 ,temp62 ,temp63 ,temp64 ,temp65 ,temp66 ,temp67 ,temp68 ,temp69;
uint32_t k = 0;

uint64_t Current = 0;


uint16_t Sum_of_Voltage = 0;

uint16_t stack = 0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _StructureSLAVEBMS {
	float CV[18U];
	float GV[9U];
	float REF;
	float SC;
	float ITMP;
	float VA;
	float VD;
	float VUV;	/* CFGR */
	float VOV;	/* CFGR */
	unsigned short iITMP;

	unsigned char COV[18U];
	unsigned char CUV[18U];
	unsigned char REV;
	unsigned char MUXFAIL;
	unsigned char THSD;

	unsigned char GPIO_SET[9U];	/* CFGR */
	unsigned char REFON;		/* CFGR */
	unsigned char DTEN;			/* CFGR */
	unsigned char ADCOPT;		/* CFGR */
	unsigned char DCC[18U];
	unsigned char DCTO;
	unsigned char MUTE;
	unsigned char FDRF;
	unsigned char PS;
	unsigned char DTMEN;
	unsigned char DCC0;
} StructureSLAVEBMS;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilterConfig;
uint8_t tx_data[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t rx_data[3]={0x00,0x00,0x00};
uint32_t Mailbox;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* SPI control definitions */
#define SLAVEBMSFORWARD_CS_H()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_SET);
#define SLAVEBMSFORWARD_CS_L()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_RESET);
#define SLAVEBMSREVERSE_CS_H()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_SET);
#define SLAVEBMSREVERSE_CS_L()		HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port, SPI1_NSS1_Pin, GPIO_PIN_RESET);
#define SLAVEBMSACCESS_FWD			(unsigned char)(0x01)
#define SLAVEBMSACCESS_REV			(unsigned char)(0x02)
#define SLAVEBMSACCESS_CRC_MISMATCH	(unsigned char)(0xFF)
#define SLAVEBMSACCESS_CRC_MATCH	(unsigned char)(0x00)

#define SLAVEBMS_CMD_PLADC		(unsigned short)(0x0714)	/* Poll ADC conversion status */

#define SLAVEBMS_CMD_CLRCELL		(unsigned short)(0x0711)	/* Clear cell voltage Data_Array group */
#define SLAVEBMS_CMD_CLRAUX		(unsigned short)(0x0712)	/* Clear auxiliary Data_Array group */
#define SLAVEBMS_CMD_CLRSTAT   	(unsigned short)(0x0713)	/* Clear status Data_Array group */

/* SLAVEBMS Order */
#define SLAVEBMS_CMD_WRCFGA		(unsigned short)(0x0001)
#define SLAVEBMS_CMD_WRCFGB		(unsigned short)(0x0024)
#define SLAVEBMS_CMD_RDCFGA		(unsigned short)(0x0002)
#define SLAVEBMS_CMD_RDCFGB		(unsigned short)(0x0026)
#define SLAVEBMS_CMD_RDCVA		(unsigned short)(0x0004)
#define SLAVEBMS_CMD_RDCVB		(unsigned short)(0x0006)
#define SLAVEBMS_CMD_RDCVC		(unsigned short)(0x0008)
#define SLAVEBMS_CMD_RDCVD		(unsigned short)(0x000A)
#define SLAVEBMS_CMD_RDCVE		(unsigned short)(0x0009)
#define SLAVEBMS_CMD_RDCVF		(unsigned short)(0x000B)
#define SLAVEBMS_CMD_RDAUXA		(unsigned short)(0x000C)
#define SLAVEBMS_CMD_RDAUXB		(unsigned short)(0x000E)
#define SLAVEBMS_CMD_RDAUXC		(unsigned short)(0x000D)
#define SLAVEBMS_CMD_RDAUXD		(unsigned short)(0x000F)
#define SLAVEBMS_CMD_RDSTATA		(unsigned short)(0x0010)
#define SLAVEBMS_CMD_RDSTATB		(unsigned short)(0x0012)
#define SLAVEBMS_CMD_ADCV		(unsigned short)(0x0260)
#define SLAVEBMS_CMD_ADAX		(unsigned short)(0x0460)
#define SLAVEBMS_CMD_ADSTAT		(unsigned short)(0x0468)

#define SLAVEBMS_CBD_MD_27K		(unsigned short)((unsigned short)(0x0001) << 7)		/* ADCOPT = 0, FAST      */
#define SLAVEBMS_CBD_MD_7K		(unsigned short)((unsigned short)(0x0002) << 7)		/* ADCOPT = 0, NORMAL */
#define SLAVEBMS_CBD_MD_26		(unsigned short)((unsigned short)(0x0003) << 7)		/* ADCOPT = 0, FILTERED */
#define SLAVEBMS_CBD_MD_14K		(unsigned short)((unsigned short)(0x0001) << 7)		/* ADCOPT = 1, FAST      */
#define SLAVEBMS_CBD_MD_3K		(unsigned short)((unsigned short)(0x0002) << 7)		/* ADCOPT = 1, NORMAL */
#define SLAVEBMS_CBD_MD_2K		(unsigned short)((unsigned short)(0x0003) << 7)		/* ADCOPT = 1, FILTERED */
#define SLAVEBMS_CBD_DCP			(unsigned short)(0x0010)	/* Discharge permit */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned char SLAVEBMS_Access(unsigned char Value);
void SLAVEBMS_Wakeup(unsigned char direction);
void SLAVEBMS_InitCRCTable(void);
unsigned short SLAVEBMS_CRC_calc(unsigned char len, unsigned char *data);
unsigned char SLAVEBMS_ReadDataGroup(unsigned char direction, unsigned short Order, unsigned char Counting, unsigned char *BF);
void SLAVEBMS_WriteSingleCmd(unsigned char direction, unsigned short RequestCMD);
void SLAVEBMS_PollADCDone(unsigned char direction, unsigned char *Result);
void Delay_(void);

unsigned short SLAVEBMS_CRC_Table[256];
unsigned char SLAVEBMS_Function1 = 0U;

StructureSLAVEBMS SLAVEBMSA;
StructureSLAVEBMS SLAVEBMSB;
StructureSLAVEBMS SLAVEBMSC;
StructureSLAVEBMS SLAVEBMSD;
StructureSLAVEBMS SLAVEBMSE;

SPI_HandleTypeDef hspi1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Delay_(void)
{
	volatile unsigned long Delay = 4444;
	while(Delay--);
}

void SLAVEBMS_InitCRCTable(void)
{
	unsigned short Indicator = 0U;
	unsigned short Bit;
	unsigned short PEC;

	for(Indicator = 0U; Indicator < 256; Indicator++)
	{
		PEC = Indicator << 7U;
		for(Bit = 8U; Bit > 0; --Bit)
		{
			if(PEC & 0x4000)
			{
				PEC = (PEC << 1U);
				PEC = (PEC ^ (unsigned short)(0x4599));
			}
			else
			{
				PEC = (PEC << 1U);
			}
		}
		SLAVEBMS_CRC_Table[Indicator] = PEC & 0xFFFF;
	}
}

unsigned short SLAVEBMS_CRC_calc(unsigned char len, unsigned char *data)
{
	unsigned short Indicator;
	unsigned short PEC;
	unsigned short address;

	PEC = 16;
	for(Indicator = 0; Indicator < len; Indicator++)
	{
		address = ((PEC >> 7) ^ data[Indicator]) & 0xff;
		PEC = (PEC << 8) ^ SLAVEBMS_CRC_Table[address];
	}

	return (PEC*2);
}


unsigned char SLAVEBMS_Access(unsigned char Value)
{
	HAL_SPI_TransmitReceive(&hspi1, &Value, &Value, 1, 10);
	return (unsigned char)(Value);
}


void SLAVEBMS_Wakeup(unsigned char direction)
{
	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		SLAVEBMS_Access(0x00);
		SLAVEBMS_Access(0x00);
		SLAVEBMSFORWARD_CS_H();
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		SLAVEBMS_Access(0x00);
		SLAVEBMS_Access(0x00);
		SLAVEBMSREVERSE_CS_H();
	}
}

unsigned char SLAVEBMS_ReadDataGroup(unsigned char direction, unsigned short Order, unsigned char Counting, unsigned char *BF)
{
	unsigned char TX_BF[4U];
	unsigned short CRC_calc;
	unsigned short CRC_extracted;
	unsigned char Indicator;
	unsigned char ReadCount = Counting * 8U;

	for(Indicator = 0U; Indicator < ReadCount; Indicator++)
	{
		*(BF + Indicator) = 0x00;
	}

	TX_BF[0U] = (unsigned char)((Order >> 8U) & 0x00FFU);
	TX_BF[1U] = (unsigned char)((Order >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, TX_BF);
	TX_BF[2U] = (unsigned char)((CRC_calc >> 8U) & 0x00FFU);
	TX_BF[3U] = (unsigned char)((CRC_calc >> 0U) & 0x00FFU);

	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Delay_();
		for(Indicator = 0U; Indicator < ReadCount; Indicator++)
		{
			*(BF + Indicator) = SLAVEBMS_Access((unsigned char)(0x00));
		}
		SLAVEBMSFORWARD_CS_H();
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Delay_();
		for(Indicator = 0U; Indicator < ReadCount; Indicator++)
		{
			*(BF + Indicator) = SLAVEBMS_Access((unsigned char)(0x00));
		}
		SLAVEBMSREVERSE_CS_H();
	}

	for(Indicator = 0U; Indicator < Counting; Indicator++)
	{
		CRC_calc = SLAVEBMS_CRC_calc(6U, (BF + (Indicator * 8U)));
		CRC_extracted = ((unsigned short)(*(BF + (Indicator * 8U) + 6)) << 8);
		CRC_extracted |= *(BF + (Indicator * 8U) + 7);

		if(CRC_calc != CRC_extracted)
		{
			return SLAVEBMSACCESS_CRC_MISMATCH;
		}
	}

	return SLAVEBMSACCESS_CRC_MATCH;
}

void SLAVEBMS_WriteSingleCmd(unsigned char direction, unsigned short RequestCMD)
{
	unsigned short CRC_calc;
	unsigned char TX_BF[4U];
	unsigned char Indicator;

	TX_BF[0U] = (unsigned char)((RequestCMD >> 8U) & 0x00FFU);
	TX_BF[1U] = (unsigned char)((RequestCMD >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, TX_BF);
	TX_BF[2U] = (unsigned char)((CRC_calc >> 8U) & 0x00FFU);
	TX_BF[3U] = (unsigned char)((CRC_calc >> 0U) & 0x00FFU);

	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		SLAVEBMSFORWARD_CS_H();
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		SLAVEBMSREVERSE_CS_H();
	}
}

void SLAVEBMS_PollADCDone(unsigned char direction, unsigned char *Result)
{
	unsigned short CRC_calc;
	unsigned char TX_BF[4U];
	unsigned char Indicator;

	TX_BF[0U] = (unsigned char)((SLAVEBMS_CMD_PLADC >> 8U) & 0x00FFU);
	TX_BF[1U] = (unsigned char)((SLAVEBMS_CMD_PLADC >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, TX_BF);
	TX_BF[2U] = (unsigned char)((CRC_calc >> 8U) & 0x00FFU);
	TX_BF[3U] = (unsigned char)((CRC_calc >> 0U) & 0x00FFU);

	if(direction == SLAVEBMSACCESS_FWD)
	{
		SLAVEBMSFORWARD_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Indicator = 0U;
		while(1)
		{
			HAL_Delay(1);
			if(SLAVEBMS_Access((unsigned char)(0xFF)) == (unsigned char)(0xFF))
			{
				SLAVEBMSFORWARD_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MATCH;
				break;
			}
			Indicator++;
			if(Indicator > 16)
			{
				SLAVEBMSFORWARD_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MISMATCH;
				break;
			}
		}
	}
	else if(direction == SLAVEBMSACCESS_REV)
	{
		SLAVEBMSREVERSE_CS_L();
		for(Indicator = 0U; Indicator < 4U; Indicator++)
		{
			SLAVEBMS_Access(TX_BF[Indicator]);
		}
		Indicator = 0U;
		while(1)
		{
			HAL_Delay(1);
			if(SLAVEBMS_Access((unsigned char)(0xFF)) == (unsigned char)(0xFF))
			{
				SLAVEBMSREVERSE_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MATCH;
				break;
			}
			Indicator++;
			if(Indicator > 16)
			{
				SLAVEBMSREVERSE_CS_H();
				*Result = SLAVEBMSACCESS_CRC_MISMATCH;
				break;
			}
		}
	}
}

void Reading_Voltage(void)
{
	unsigned char Data_Array[44U]; //Order array 4 + (Configuration array 6 + CRC 2)*5
	unsigned short CRC_calc;
	unsigned char ConversionResult;

	SLAVEBMS_Wakeup(SLAVEBMSACCESS_FWD);
	HAL_Delay(10);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRCFGA >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRCFGA >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)(0xFE);
	Data_Array[5] = (unsigned char)(0x0E);
	Data_Array[6] = (unsigned char)(0x18);
	Data_Array[7] = (unsigned char)(0xA4);
	Data_Array[8] = (unsigned char)(0x00);
	Data_Array[9] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)(0xFE);
	Data_Array[13] = (unsigned char)(0x0E);
	Data_Array[14] = (unsigned char)(0x18);
	Data_Array[15] = (unsigned char)(0xA4);
	Data_Array[16] = (unsigned char)(0x00);
	Data_Array[17] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)(0xFE);
	Data_Array[21] = (unsigned char)(0x0E);
	Data_Array[22] = (unsigned char)(0x18);
	Data_Array[23] = (unsigned char)(0xA4);
	Data_Array[24] = (unsigned char)(0x00);
	Data_Array[25] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)(0xFE);
	Data_Array[29] = (unsigned char)(0x0E);
	Data_Array[30] = (unsigned char)(0x18);
	Data_Array[31] = (unsigned char)(0xA4);
	Data_Array[32] = (unsigned char)(0x00);
	Data_Array[33] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)(0xFE);
	Data_Array[37] = (unsigned char)(0x0E);
	Data_Array[38] = (unsigned char)(0x18);
	Data_Array[39] = (unsigned char)(0xA4);
	Data_Array[40] = (unsigned char)(0x00);
	Data_Array[41] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);


	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);

	Data_Array[0] = (unsigned char)((SLAVEBMS_CMD_WRCFGB >> 8U) & 0x00FFU);
	Data_Array[1] = (unsigned char)((SLAVEBMS_CMD_WRCFGB >> 0U) & 0x00FFU);
	CRC_calc = SLAVEBMS_CRC_calc(2U, &Data_Array[0U]);
	Data_Array[2] = (unsigned char)(CRC_calc >> 8);
	Data_Array[3] = (unsigned char)(CRC_calc >> 0);

	Data_Array[4] = (unsigned char)(0x0F);
	Data_Array[5] = (unsigned char)(0x80);
	Data_Array[6] = (unsigned char)(0x00);
	Data_Array[7] = (unsigned char)(0x00);
	Data_Array[8] = (unsigned char)(0x00);
	Data_Array[9] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[4]);
	Data_Array[10] = (unsigned char)(CRC_calc >> 8);
	Data_Array[11] = (unsigned char)(CRC_calc >> 0);

	Data_Array[12] = (unsigned char)(0x0F);
	Data_Array[13] = (unsigned char)(0x80);
	Data_Array[14] = (unsigned char)(0x00);
	Data_Array[15] = (unsigned char)(0x00);
	Data_Array[16] = (unsigned char)(0x00);
	Data_Array[17] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[12]);
	Data_Array[18] = (unsigned char)(CRC_calc >> 8);
	Data_Array[19] = (unsigned char)(CRC_calc >> 0);

	Data_Array[20] = (unsigned char)(0x0F);
	Data_Array[21] = (unsigned char)(0x80);
	Data_Array[22] = (unsigned char)(0x00);
	Data_Array[23] = (unsigned char)(0x00);
	Data_Array[24] = (unsigned char)(0x00);
	Data_Array[25] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[20]);
	Data_Array[26] = (unsigned char)(CRC_calc >> 8);
	Data_Array[27] = (unsigned char)(CRC_calc >> 0);

	Data_Array[28] = (unsigned char)(0x0F);
	Data_Array[29] = (unsigned char)(0x80);
	Data_Array[30] = (unsigned char)(0x00);
	Data_Array[31] = (unsigned char)(0x00);
	Data_Array[32] = (unsigned char)(0x00);
	Data_Array[33] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[28]);
	Data_Array[34] = (unsigned char)(CRC_calc >> 8);
	Data_Array[35] = (unsigned char)(CRC_calc >> 0);

	Data_Array[36] = (unsigned char)(0x0F);
	Data_Array[37] = (unsigned char)(0x80);
	Data_Array[38] = (unsigned char)(0x00);
	Data_Array[39] = (unsigned char)(0x00);
	Data_Array[40] = (unsigned char)(0x00);
	Data_Array[41] = (unsigned char)(0x00);
	CRC_calc = SLAVEBMS_CRC_calc(6U, &Data_Array[36]);
	Data_Array[42] = (unsigned char)(CRC_calc >> 8);
	Data_Array[43] = (unsigned char)(CRC_calc >> 0);

	SLAVEBMSFORWARD_CS_L();
	for(CRC_calc = 0U; CRC_calc < 44; CRC_calc++)
	{
		SLAVEBMS_Access(Data_Array[CRC_calc]);
	}
	SLAVEBMSFORWARD_CS_H();
	HAL_Delay(1);


	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_CLRCELL);
	HAL_Delay(2);
	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_ADCV | SLAVEBMS_CBD_MD_7K);
	HAL_Delay(1);
	SLAVEBMS_PollADCDone(SLAVEBMSACCESS_FWD, &ConversionResult);
	if(ConversionResult == SLAVEBMSACCESS_CRC_MATCH)
	{
			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVA, 5, Data_Array);	//기존 코딩?��?�� ?��?�� ( 기존?�� CRC Mismatch ?��?��?��?�� CV?�� ???��?? ?���???????????????�??????????????? ?���??????????????? ?��?��)


			SLAVEBMSA.CV[0] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
			SLAVEBMSA.CV[1] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
			SLAVEBMSA.CV[2] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

			SLAVEBMSB.CV[0] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
			SLAVEBMSB.CV[1] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
			SLAVEBMSB.CV[2] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

			SLAVEBMSC.CV[0] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
			SLAVEBMSC.CV[1] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
			SLAVEBMSC.CV[2] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

			SLAVEBMSD.CV[0] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
			SLAVEBMSD.CV[1] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
			SLAVEBMSD.CV[2] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

			SLAVEBMSE.CV[0] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
			SLAVEBMSE.CV[1] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
			SLAVEBMSE.CV[2] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVB, 5, Data_Array);

			SLAVEBMSA.CV[3] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
			SLAVEBMSA.CV[4] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
			SLAVEBMSA.CV[5] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

			SLAVEBMSB.CV[3] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
			SLAVEBMSB.CV[4] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
			SLAVEBMSB.CV[5] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

			SLAVEBMSC.CV[3] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
			SLAVEBMSC.CV[4] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
			SLAVEBMSC.CV[5] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

			SLAVEBMSD.CV[3] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
			SLAVEBMSD.CV[4] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
			SLAVEBMSD.CV[5] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

			SLAVEBMSE.CV[3] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
			SLAVEBMSE.CV[4] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
			SLAVEBMSE.CV[5] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVC, 5, Data_Array);

			SLAVEBMSA.CV[6] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
			SLAVEBMSA.CV[7] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
			SLAVEBMSA.CV[8] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

			SLAVEBMSB.CV[6] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
			SLAVEBMSB.CV[7] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
			SLAVEBMSB.CV[8] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

			SLAVEBMSC.CV[6] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
			SLAVEBMSC.CV[7] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
			SLAVEBMSC.CV[8] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

			SLAVEBMSD.CV[6] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
			SLAVEBMSD.CV[7] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
			SLAVEBMSD.CV[8] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

			SLAVEBMSE.CV[6] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
			SLAVEBMSE.CV[7] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
			SLAVEBMSE.CV[8] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVD, 5, Data_Array);

			SLAVEBMSA.CV[9] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
			SLAVEBMSA.CV[10] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
			SLAVEBMSA.CV[11] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

			SLAVEBMSB.CV[9] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
			SLAVEBMSB.CV[10] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
			SLAVEBMSB.CV[11] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

			SLAVEBMSC.CV[9] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
			SLAVEBMSC.CV[10] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
			SLAVEBMSC.CV[11] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

			SLAVEBMSD.CV[9] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
			SLAVEBMSD.CV[10] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
			SLAVEBMSD.CV[11] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

			SLAVEBMSE.CV[9] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
			SLAVEBMSE.CV[10] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
			SLAVEBMSE.CV[11] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVE, 5, Data_Array);

			SLAVEBMSA.CV[12] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
			SLAVEBMSA.CV[13] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
			SLAVEBMSA.CV[14] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

			SLAVEBMSB.CV[12] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
			SLAVEBMSB.CV[13] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
			SLAVEBMSB.CV[14] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

			SLAVEBMSC.CV[12] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
			SLAVEBMSC.CV[13] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
			SLAVEBMSC.CV[14] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

			SLAVEBMSD.CV[12] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
			SLAVEBMSD.CV[13] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
			SLAVEBMSD.CV[14] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

			SLAVEBMSE.CV[12] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
			SLAVEBMSE.CV[13] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
			SLAVEBMSE.CV[14] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);

			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDCVF, 5, Data_Array);

			SLAVEBMSA.CV[15] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
			SLAVEBMSA.CV[16] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
			SLAVEBMSA.CV[17] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

			SLAVEBMSB.CV[15] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
			SLAVEBMSB.CV[16] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
			SLAVEBMSB.CV[17] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

			SLAVEBMSC.CV[15] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
			SLAVEBMSC.CV[16] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
			SLAVEBMSC.CV[17] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

			SLAVEBMSD.CV[15] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
			SLAVEBMSD.CV[16] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
			SLAVEBMSD.CV[17] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

			SLAVEBMSE.CV[15] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
			SLAVEBMSE.CV[16] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
			SLAVEBMSE.CV[17] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);

	}


	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_CLRAUX);
	HAL_Delay(2);
	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_ADAX | SLAVEBMS_CBD_MD_7K);
	HAL_Delay(1);
	SLAVEBMS_PollADCDone(SLAVEBMSACCESS_FWD, &ConversionResult);

	if(ConversionResult == SLAVEBMSACCESS_CRC_MATCH)
		{
			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXA, 5, Data_Array);
		}
		else
		{
		}
	SLAVEBMSA.GV[0] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
	SLAVEBMSA.GV[1] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
	SLAVEBMSA.GV[2] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

	SLAVEBMSB.GV[0] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
	SLAVEBMSB.GV[1] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
	SLAVEBMSB.GV[2] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

	SLAVEBMSC.GV[0] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
	SLAVEBMSC.GV[1] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
	SLAVEBMSC.GV[2] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

	SLAVEBMSD.GV[0] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
	SLAVEBMSD.GV[1] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
	SLAVEBMSD.GV[2] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

	SLAVEBMSE.GV[0] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
	SLAVEBMSE.GV[1] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
	SLAVEBMSE.GV[2] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


	SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXB, 5, Data_Array);

	SLAVEBMSA.GV[3] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
	SLAVEBMSA.GV[4] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
	SLAVEBMSA.REF = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

	SLAVEBMSB.GV[3] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
	SLAVEBMSB.GV[4] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
	SLAVEBMSB.REF = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

	SLAVEBMSC.GV[3] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
	SLAVEBMSC.GV[4] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
	SLAVEBMSC.REF = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

	SLAVEBMSD.GV[3] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
	SLAVEBMSD.GV[4] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
	SLAVEBMSD.REF = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

	SLAVEBMSE.GV[3] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
	SLAVEBMSE.GV[4] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
	SLAVEBMSE.REF = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


	SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXC, 5, Data_Array);


	SLAVEBMSA.GV[5] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
	SLAVEBMSA.GV[6] = (float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001);
	SLAVEBMSA.GV[7] = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

	SLAVEBMSB.GV[5] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
	SLAVEBMSB.GV[6] = (float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001);
	SLAVEBMSB.GV[7] = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

	SLAVEBMSC.GV[5] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
	SLAVEBMSC.GV[6] = (float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001);
	SLAVEBMSC.GV[7] = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

	SLAVEBMSD.GV[5] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
	SLAVEBMSD.GV[6] = (float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001);
	SLAVEBMSD.GV[7] = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

	SLAVEBMSE.GV[5] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);
	SLAVEBMSE.GV[6] = (float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001);
	SLAVEBMSE.GV[7] = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


	SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDAUXD, 5, Data_Array);


	SLAVEBMSA.GV[8] = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
	SLAVEBMSB.GV[8] = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
	SLAVEBMSC.GV[8] = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
	SLAVEBMSD.GV[8] = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
	SLAVEBMSE.GV[8] = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);



	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_CLRSTAT);
	HAL_Delay(2);
	SLAVEBMS_WriteSingleCmd(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_ADSTAT | SLAVEBMS_CBD_MD_7K);
	HAL_Delay(1);
	SLAVEBMS_PollADCDone(SLAVEBMSACCESS_FWD, &ConversionResult);
		if(ConversionResult == SLAVEBMSACCESS_CRC_MATCH)
		{
			SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDSTATA, 5, Data_Array);
		}
		else
		{
		}

	SLAVEBMSA.SC = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.003);
	SLAVEBMSA.ITMP = ((float)(((unsigned short)(Data_Array[3]) << 8) | Data_Array[2]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
	SLAVEBMSA.VA = (float)(((unsigned short)(Data_Array[5]) << 8) | Data_Array[4]) * (float)(0.0001);

	SLAVEBMSB.SC = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.003);
	SLAVEBMSB.ITMP = ((float)(((unsigned short)(Data_Array[11]) << 8) | Data_Array[10]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
	SLAVEBMSB.VA = (float)(((unsigned short)(Data_Array[13]) << 8) | Data_Array[12]) * (float)(0.0001);

	SLAVEBMSC.SC = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.003);
	SLAVEBMSC.ITMP = ((float)(((unsigned short)(Data_Array[19]) << 8) | Data_Array[18]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
	SLAVEBMSC.VA = (float)(((unsigned short)(Data_Array[21]) << 8) | Data_Array[20]) * (float)(0.0001);

	SLAVEBMSD.SC = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.003);
	SLAVEBMSD.ITMP = ((float)(((unsigned short)(Data_Array[27]) << 8) | Data_Array[26]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
	SLAVEBMSD.VA = (float)(((unsigned short)(Data_Array[29]) << 8) | Data_Array[28]) * (float)(0.0001);

	SLAVEBMSE.SC = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.003);
	SLAVEBMSE.ITMP = ((float)(((unsigned short)(Data_Array[35]) << 8) | Data_Array[34]) * (float)(0.0001) / (float)(0.0076)) - (float)(276.);
	SLAVEBMSE.VA = (float)(((unsigned short)(Data_Array[37]) << 8) | Data_Array[36]) * (float)(0.0001);


	SLAVEBMS_ReadDataGroup(SLAVEBMSACCESS_FWD, SLAVEBMS_CMD_RDSTATB, 5, Data_Array);

	SLAVEBMSA.VD = (float)(((unsigned short)(Data_Array[1]) << 8) | Data_Array[0]) * (float)(0.0001);
	SLAVEBMSB.VD = (float)(((unsigned short)(Data_Array[9]) << 8) | Data_Array[8]) * (float)(0.0001);
	SLAVEBMSC.VD = (float)(((unsigned short)(Data_Array[17]) << 8) | Data_Array[16]) * (float)(0.0001);
	SLAVEBMSD.VD = (float)(((unsigned short)(Data_Array[25]) << 8) | Data_Array[24]) * (float)(0.0001);
	SLAVEBMSE.VD = (float)(((unsigned short)(Data_Array[33]) << 8) | Data_Array[32]) * (float)(0.0001);


}


void Save_voltage()
{
	  C101 = SLAVEBMSA.CV[0];
	  C102 = SLAVEBMSA.CV[1];
	  C103 = SLAVEBMSA.CV[2];
	  C104 = SLAVEBMSA.CV[3];
	  C105 = SLAVEBMSA.CV[4];
	  C106 = SLAVEBMSA.CV[5];
	  C107 = SLAVEBMSA.CV[6];
	  C108 = SLAVEBMSA.CV[7];
	  C109 = SLAVEBMSA.CV[8];
	  C110 = SLAVEBMSA.CV[9];
	  C111 = SLAVEBMSA.CV[10];
	  C112 = SLAVEBMSA.CV[11];
	  C113 = SLAVEBMSA.CV[12];
	  C114 = SLAVEBMSA.CV[13];
	  C201 = SLAVEBMSB.CV[0];
	  C202 = SLAVEBMSB.CV[1];
	  C203 = SLAVEBMSB.CV[2];
	  C204 = SLAVEBMSB.CV[3];
	  C205 = SLAVEBMSB.CV[4];
	  C206 = SLAVEBMSB.CV[5];
	  C207 = SLAVEBMSB.CV[6];
	  C208 = SLAVEBMSB.CV[7];
	  C209 = SLAVEBMSB.CV[8];
	  C210 = SLAVEBMSB.CV[9];
	  C211 = SLAVEBMSB.CV[10];
	  C212 = SLAVEBMSB.CV[11];
	  C213 = SLAVEBMSB.CV[12];
	  C214 = SLAVEBMSB.CV[13];
	  C301 = SLAVEBMSC.CV[0];
	  C302 = SLAVEBMSC.CV[1];
	  C303 = SLAVEBMSC.CV[2];
	  C304 = SLAVEBMSC.CV[3];
	  C305 = SLAVEBMSC.CV[4];
	  C306 = SLAVEBMSC.CV[5];
	  C307 = SLAVEBMSC.CV[6];
	  C308 = SLAVEBMSC.CV[7];
	  C309 = SLAVEBMSC.CV[8];
	  C310 = SLAVEBMSC.CV[9];
	  C311 = SLAVEBMSC.CV[10];
	  C312 = SLAVEBMSC.CV[11];
	  C313 = SLAVEBMSC.CV[12];
	  C314 = SLAVEBMSC.CV[13];
	  C401 = SLAVEBMSD.CV[0];
	  C402 = SLAVEBMSD.CV[1];
	  C403 = SLAVEBMSD.CV[2];
	  C404 = SLAVEBMSD.CV[3];
	  C405 = SLAVEBMSD.CV[4];
	  C406 = SLAVEBMSD.CV[5];
	  C407 = SLAVEBMSD.CV[6];
	  C408 = SLAVEBMSD.CV[7];
	  C409 = SLAVEBMSD.CV[8];
	  C410 = SLAVEBMSD.CV[9];
	  C411 = SLAVEBMSD.CV[10];
	  C412 = SLAVEBMSD.CV[11];
	  C413 = SLAVEBMSD.CV[12];
	  C414 = SLAVEBMSD.CV[13];
	  C501 = SLAVEBMSE.CV[0];
	  C502 = SLAVEBMSE.CV[1];
	  C503 = SLAVEBMSE.CV[2];
	  C504 = SLAVEBMSE.CV[3];
	  C505 = SLAVEBMSE.CV[4];
	  C506 = SLAVEBMSE.CV[5];
	  C507 = SLAVEBMSE.CV[6];
	  C508 = SLAVEBMSE.CV[7];
	  C509 = SLAVEBMSE.CV[8];
	  C510 = SLAVEBMSE.CV[9];
	  C511 = SLAVEBMSE.CV[10];
	  C512 = SLAVEBMSE.CV[11];
	  C513 = SLAVEBMSE.CV[12];
	  C514 = SLAVEBMSE.CV[13];
	  temp00 = Temp[0];
	  temp01 = Temp[1];
	  temp02 = Temp[2];
	  temp03 = Temp[3];
	  temp04 = Temp[4];
	  temp05 = Temp[5];
	  temp06 = Temp[6];
	  temp07 = Temp[7];
	  temp08 = Temp[8];
	  temp09 = Temp[9];
	  temp10 = Temp[10];
	  temp11 = Temp[11];
	  temp12 = Temp[12];
	  temp13 = Temp[13];
	  temp14 = Temp[14];
	  temp15 = Temp[15];
	  temp16 = Temp[16];
	  temp17 = Temp[17];
	  temp18 = Temp[18];
	  temp19 = Temp[19];
	  temp20 = Temp[20];
	  temp21 = Temp[21];
	  temp22 = Temp[22];
	  temp23 = Temp[23];
	  temp24 = Temp[24];
	  temp25 = Temp[25];
	  temp26 = Temp[26];
	  temp27 = Temp[27];
	  temp28 = Temp[28];
	  temp29 = Temp[29];
	  temp30 = Temp[30];
	  temp31 = Temp[31];
	  temp32 = Temp[32];
	  temp33 = Temp[33];
	  temp34 = Temp[34];
	  temp35 = Temp[35];
	  temp36 = Temp[36];
	  temp37 = Temp[37];
	  temp38 = Temp[38];
	  temp39 = Temp[39];
	  temp40 = Temp[40];
	  temp41 = Temp[41];
	  temp42 = Temp[42];
	  temp43 = Temp[43];
	  temp44 = Temp[44];
	  temp45 = Temp[45];
	  temp46 = Temp[46];
	  temp47 = Temp[47];
	  temp48 = Temp[48];
	  temp49 = Temp[49];
	  temp50 = Temp[50];
	  temp51 = Temp[51];
	  temp52 = Temp[52];
	  temp53 = Temp[53];
	  temp54 = Temp[54];
	  temp55 = Temp[55];
	  temp56 = Temp[56];
	  temp57 = Temp[57];
	  temp58 = Temp[58];
	  temp59 = Temp[59];
	  temp60 = Temp[60];
	  temp61 = Temp[61];
	  temp62 = Temp[62];
	  temp63 = Temp[63];
	  temp64 = Temp[64];
	  temp65 = Temp[65];
	  temp66 = Temp[66];
	  temp67 = Temp[67];
	  temp68 = Temp[68];
	  temp69 = Temp[69];



	  Sum_of_Voltage = SLAVEBMSA.SC + SLAVEBMSB.SC + SLAVEBMSC.SC + SLAVEBMSD.SC + SLAVEBMSE.SC;
}

void Voltage_Drop_Sense()
{
    	while((C101<=2.50) || (C102<=2.50) || (C103<=2.50) || (C104<=2.50) || (C105<=2.50) || (C106<=2.50) || (C107<=2.50) ||
    		   (C108<=2.50) || (C109<=2.50) || (C110<=2.50) || (C111<=2.50) || (C112<=2.50) || (C113<=2.50) || (C114<=2.50) ||
    		   (C201<=2.50) || (C202<=2.50) || (C203<=2.50) || (C204<=2.50) || (C205<=2.50) || (C206<=2.50) || (C207<=2.50) ||
    		   (C208<=2.50) || (C209<=2.50) || (C210<=2.50) || (C211<=2.50) || (C212<=2.50) || (C213<=2.50) || (C214<=2.50) ||
    		   (C301<=2.50) || (C302<=2.50) || (C303<=2.50) || (C304<=2.50) || (C305<=2.50) || (C306<=2.50) || (C307<=2.50) ||
    		   (C308<=2.50) || (C309<=2.50) || (C310<=2.50) || (C311<=2.50) || (C312<=2.50) || (C313<=2.50) || (C314<=2.50) ||
    		   (C401<=2.50) || (C402<=2.50) || (C403<=2.50) || (C404<=2.50) || (C405<=2.50) || (C406<=2.50) || (C407<=2.50) ||
    		   (C408<=2.50) || (C409<=2.50) || (C410<=2.50) || (C411<=2.50) || (C412<=2.50) || (C413<=2.50) || (C414<=2.50) ||
    		   (C501<=2.50) || (C502<=2.50) || (C503<=2.50) || (C504<=2.50) || (C505<=2.50) || (C506<=2.50) || (C507<=2.50) ||
    		   (C508<=2.50) || (C509<=2.50) || (C510<=2.50) || (C511<=2.50) || (C512<=2.50) || (C513<=2.50) || (C514<=2.50))
    	{
                 stack++;

                 Reading_Voltage();

                 Save_voltage();

                 if (stack == 10)
                 {
                	 HAL_Delay(1000);
                	 HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_RESET);//(1)
                	 HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);//(2)

                   	 while(1)
                   	 {
                   	   Reading_Voltage(); //SLAVEBMSA + B + C + D + E

                   	   Save_voltage(); //cell array

                   	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                   	   HAL_Delay(200);
                   	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                   	   HAL_Delay(200);
                   	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                   	   HAL_Delay(200);
                   	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                   	   HAL_Delay(3000);
                   	 }
                 }
    	}

    	stack = 0;
}


void  Control_Cell_UVE()	//control with voltage
{
        // UVR : 2.50V Per Cell -> DSG ON>=2.50
        if((C101<=2.50) || (C102<=2.50) || (C103<=2.50) || (C104<=2.50) || (C105<=2.50) || (C106<=2.50) || (C107<=2.50) ||
   		  (C108<=2.50) || (C109<=2.50) || (C110<=2.50) || (C111<=2.50) || (C112<=2.50) || (C113<=2.50) || (C114<=2.50) ||
		  (C201<=2.50) || (C202<=2.50) || (C203<=2.50) || (C204<=2.50) || (C205<=2.50) || (C206<=2.50) || (C207<=2.50) ||
		  (C208<=2.50) || (C209<=2.50) || (C210<=2.50) || (C211<=2.50) || (C212<=2.50) || (C213<=2.50) || (C214<=2.50) ||
		  (C301<=2.50) || (C302<=2.50) || (C303<=2.50) || (C304<=2.50) || (C305<=2.50) || (C306<=2.50) || (C307<=2.50) ||
		  (C308<=2.50) || (C309<=2.50) || (C310<=2.50) || (C311<=2.50) || (C312<=2.50) || (C313<=2.50) || (C314<=2.50) ||
		  (C401<=2.50) || (C402<=2.50) || (C403<=2.50) || (C404<=2.50) || (C405<=2.50) || (C406<=2.50) || (C407<=2.50) ||
		  (C408<=2.50) || (C409<=2.50) || (C410<=2.50) || (C411<=2.50) || (C412<=2.50) || (C413<=2.50) || (C414<=2.50) ||
		  (C501<=2.50) || (C502<=2.50) || (C503<=2.50) || (C504<=2.50) || (C505<=2.50) || (C506<=2.50) || (C507<=2.50) ||
		  (C508<=2.50) || (C509<=2.50) || (C510<=2.50) || (C511<=2.50) || (C512<=2.50) || (C513<=2.50) || (C514<=2.50))
           {
        		HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);//(1)
        		HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_RESET);//(2)

                while(1)
                {
                	Reading_Voltage(); //SLAVEBMSA + B + C + D + E

                	Save_voltage(); //cell array

            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            		HAL_Delay(3000);
                }
           }
}

void Control_Cell_OVE()
{
        // OVR : 4.20V Per Cell -> CHG ON>=4.20
        if((C101>=4.20) || (C102>=4.20) || (C103>=4.20) || (C104>=4.20) || (C105>=4.20) || (C106>=4.20) || (C107>=4.20) ||
    		(C108>=4.20) || (C109>=4.20) || (C110>=4.20) || (C111>=4.20) || (C112>=4.20) || (C113>=4.20) || (C114>=4.20) ||
    		(C201>=4.20) || (C202>=4.20) || (C203>=4.20) || (C204>=4.20) || (C205>=4.20) || (C206>=4.20) || (C207>=4.20) ||
    		(C208>=4.20) || (C209>=4.20) || (C210>=4.20) || (C211>=4.20) || (C212>=4.20) || (C213>=4.20) || (C214>=4.20) ||
    		(C301>=4.20) || (C302>=4.20) || (C303>=4.20) || (C304>=4.20) || (C305>=4.20) || (C306>=4.20) || (C307>=4.20) ||
    		(C308>=4.20) || (C309>=4.20) || (C310>=4.20) || (C311>=4.20) || (C312>=4.20) || (C313>=4.20) || (C314>=4.20) ||
    		(C401>=4.20) || (C402>=4.20) || (C403>=4.20) || (C404>=4.20) || (C405>=4.20) || (C406>=4.20) || (C407>=4.20) ||
    		(C408>=4.20) || (C409>=4.20) || (C410>=4.20) || (C411>=4.20) || (C412>=4.20) || (C413>=4.20) || (C414>=4.20) ||
    		(C501>=4.20) || (C502>=4.20) || (C503>=4.20) || (C504>=4.20) || (C505>=4.20) || (C506>=4.20) || (C507>=4.20) ||
    		(C508>=4.20) || (C509>=4.20) || (C510>=4.20) || (C511>=4.20) || (C512>=4.20) || (C513>=4.20) || (C514>=4.20))
           {
        		HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_RESET);//(1)
        		HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);//(2)

                while(1)
                {
                	Reading_Voltage(); //SLAVEBMSA + B + C + D + E

                	Save_voltage(); //cell array

            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            		HAL_Delay(200);
            		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            		HAL_Delay(3000);
                }
           }

}

void Transmit_Data(CAN_TxHeaderTypeDef* TxHeader)
{
	uint8_t Sum_of_Voltage_1 = ((Sum_of_Voltage & 0xff00)>>8);
	uint8_t Sum_of_Voltage_2 = (Sum_of_Voltage & 0x00ff);
	uint8_t Max_temp_1 = ((Max_temp & 0xff00)>>8);
	uint8_t Max_temp_2 = (Max_temp & 0x00ff);
	uint8_t Min_temp_1 = ((Min_temp & 0xff00)>>8);
	uint8_t Min_temp_2 = (Min_temp & 0x00ff);


	tx_data[0] = Sum_of_Voltage_1;
	tx_data[1] = Sum_of_Voltage_2;
	tx_data[2] = Max_temp_1;
	tx_data[3] = Max_temp_2;
	tx_data[4] = Min_temp_1;
	tx_data[5] = Min_temp_2;

	{
		Mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

		if(Mailbox){
		HAL_CAN_AddTxMessage(&hcan1, TxHeader, tx_data, &Mailbox);
		}
	}

}

void Power_On_Event()
{
	HAL_Delay(5000);	//PRECHARGE

	HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_RESET);

	HAL_Delay(1000);

	HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);

	HAL_Delay(1000);

}


void Select_Address(unsigned char number)  //for temperature sensor
{
	switch(number){

	case 1:
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A1_Pin|A0_Pin, GPIO_PIN_RESET);//1
	//0000
	break;

	case 2:
	HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_SET);//2
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A1_Pin, GPIO_PIN_RESET);//0001
	break;

	case 3:
	HAL_GPIO_WritePin(GPIOC, A1_Pin, GPIO_PIN_SET);//3
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A0_Pin, GPIO_PIN_RESET);//0010
	break;

	case 4:
	HAL_GPIO_WritePin(GPIOC, A1_Pin|A0_Pin, GPIO_PIN_SET);//4
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin, GPIO_PIN_RESET);//0011
	break;

	case 5:
	HAL_GPIO_WritePin(GPIOC, A2_Pin, GPIO_PIN_SET);//5
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A1_Pin|A0_Pin, GPIO_PIN_RESET);//0100
	break;

	case 6:
	HAL_GPIO_WritePin(GPIOC, A2_Pin|A0_Pin, GPIO_PIN_SET);//6
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A1_Pin, GPIO_PIN_RESET);//0101
	break;

	case 7:
	HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin, GPIO_PIN_SET);//7
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A0_Pin, GPIO_PIN_RESET);//0110
	break;

	case 8:
	HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin|A0_Pin, GPIO_PIN_SET);//8
	HAL_GPIO_WritePin(GPIOC, A3_Pin, GPIO_PIN_RESET);//0111
	break;

	case 9:
	HAL_GPIO_WritePin(GPIOC, A3_Pin, GPIO_PIN_SET);//9
	HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin|A0_Pin, GPIO_PIN_RESET);//1000
	break;

	case 10:
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A0_Pin, GPIO_PIN_SET);//10
	HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin, GPIO_PIN_RESET);//1001
	break;

	case 11:
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A1_Pin, GPIO_PIN_SET);//11
	HAL_GPIO_WritePin(GPIOC, A2_Pin|A1_Pin, GPIO_PIN_RESET);//1010
	break;

	case 12:
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A1_Pin|A0_Pin, GPIO_PIN_SET);//12
	HAL_GPIO_WritePin(GPIOC, A2_Pin, GPIO_PIN_RESET);//1011
	break;

	case 13:
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin, GPIO_PIN_SET);//13
	HAL_GPIO_WritePin(GPIOC, A1_Pin|A0_Pin, GPIO_PIN_RESET);//1100
	break;

	case 14:
	HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A0_Pin, GPIO_PIN_SET);//14
	HAL_GPIO_WritePin(GPIOC, A1_Pin, GPIO_PIN_RESET);//1101
	break;
	}
}

void Reading_Temp()  // TEMP1:PA0, TEMP2:PC2, TEMP3:PC3, TEMP4:PC4, TEMP5:PC5
{
		for(uint8_t i = 1; i < 15; i++)
		{
			Select_Address(i); // select MUX(14:0)

			k = i * 5 - 5;

			if(i != 1)
			{
				uint32_t *j = (Temp) + k;
				HAL_ADC_Start_DMA(&hadc1, j , 5);
			}

			else
			{
				HAL_ADC_Start_DMA(&hadc1, Temp, 5);
			}

			HAL_Delay(2);
		 }

		Max_temp = Temp[0];

		for (uint8_t q=0; q<70; q++)
		{
			if(Max_temp<Temp[q])
			{
				Max_temp = Temp[q];
			}
		}

		Min_temp = Temp[0];

		for (uint8_t q=0; q<70; q++)
		{
			if(Min_temp>Temp[q])
			{
				Min_temp = Temp[q];
			}
		}

		if(Max_temp > 2678)	//Max_temp > 60'C
		{
			HAL_Delay(1000);
			HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_RESET);//(1)
			HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);//(2)

			while(1)
			{

			}
		}
}


void CAN_TX_Config()
{
	  TxHeader.StdId = 0x321;
	  TxHeader.ExtId = 0x00;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.DLC = 8;
	  TxHeader.TransmitGlobalTime = DISABLE;

	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;
}

void Master_BMS_CAN_Configuration(CAN_FilterTypeDef *FilterType, CAN_RxHeaderTypeDef *RxHeader)
{
   FilterType->FilterIdHigh = 0x0000;
   FilterType->FilterIdLow = 0x0000;
   FilterType->FilterMaskIdHigh = 0x0000;
   FilterType->FilterMaskIdLow = 0x0000;
   FilterType->FilterFIFOAssignment = CAN_RX_FIFO0;
   FilterType->FilterBank = 0;
   FilterType->FilterMode = CAN_FILTERMODE_IDMASK;
   FilterType->FilterScale = CAN_FILTERSCALE_32BIT;
   FilterType->FilterActivation = ENABLE;
   FilterType->SlaveStartFilterBank = 14;


      if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
      {
         Error_Handler();
      }

      RxHeader->StdId = 0x321;
      RxHeader->ExtId = 0x01;
      RxHeader->IDE = CAN_ID_STD;
      RxHeader->RTR = CAN_RTR_DATA;
      RxHeader->DLC = 8;
      RxHeader->Timestamp = 0;
      RxHeader->FilterMatchIndex = 0;

      if(HAL_CAN_Start(&hcan1) != HAL_OK)
      {
              Error_Handler();
      }
}

void Master_BMS_CAN_Receive(CAN_HandleTypeDef *can, CAN_RxHeaderTypeDef *RxHeader, uint8_t rx_data[])
{
     HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, RxHeader, rx_data);

     Current = ((uint64_t)rx_data[0]<<8) + ((uint64_t)rx_data[1]);
}

void Control_Current()
{
	if(Current > 250)
	{
		HAL_Delay(1000);
		HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_RESET);//(1)
		HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_RESET);//(2)

		while(1)
		{

		}
	}
}

void CHG_DSG_ON()
{
	HAL_GPIO_WritePin(DSG_GPIO_Port, DSG_Pin, GPIO_PIN_SET);//(1)
	HAL_GPIO_WritePin(CHG_GPIO_Port, CHG_Pin, GPIO_PIN_SET);//(2)
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SLAVEBMS_InitCRCTable();

  CAN_TX_Config();

  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  HAL_CAN_Start(&hcan1);

  Reading_Voltage();

  Save_voltage();

  Control_Cell_UVE();

  Control_Cell_OVE();

  Power_On_Event();

  CHG_DSG_ON();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		 HAL_Delay(10);

		 Reading_Voltage(); //SLAVEBMSA + B + C + D + E

		 Save_voltage(); //cell array

		 Control_Cell_OVE(); //with voltage

		 Voltage_Drop_Sense();

		 Reading_Temp();

		 Transmit_Data(&TxHeader);

		 Master_BMS_CAN_Receive(&hcan1, &RxHeader, rx_data);

		 Control_Current();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
