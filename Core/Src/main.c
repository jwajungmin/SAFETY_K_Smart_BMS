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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void Master_BMS_CAN_Configuration(CAN_FilterTypeDef *FilterType, CAN_RxHeaderTypeDef *RxHeader);
void Master_BMS_CAN_Receive(CAN_HandleTypeDef *can, CAN_RxHeaderTypeDef *RxHeader, uint8_t rx_data[]);
void CAN_TX_Config(void);
void Current_ADC_DMA(ADC_HandleTypeDef *hadc, uint32_t data[], uint32_t Length);
void Transmit_Data(CAN_TxHeaderTypeDef* TxHeader);
void Data_Logging_Config(uint32_t Logging_Start_Page_ADDR, uint32_t Logging_END_Page_ADDR);
void Write_data(uint64_t *Wx_data0, uint64_t *Wx_data1);
void Erase_data(FLASH_EraseInitTypeDef *EraseInitType);
void Bluetooth_Tx(uint8_t *Parameter);
void Logging_Online(void);
uint32_t GetPage(uint32_t Addr);
uint32_t GetBank(uint32_t Addr);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 2 Kbytes */
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000800) /* Base @ of Page 1, 2 Kbytes */
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08001000) /* Base @ of Page 2, 2 Kbytes */
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08001800) /* Base @ of Page 3, 2 Kbytes */
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) /* Base @ of Page 4, 2 Kbytes */
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08002800) /* Base @ of Page 5, 2 Kbytes */
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08003000) /* Base @ of Page 6, 2 Kbytes */
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08003800) /* Base @ of Page 7, 2 Kbytes */
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) /* Base @ of Page 8, 2 Kbytes */
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) /* Base @ of Page 9, 2 Kbytes */
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08005000) /* Base @ of Page 10, 2 Kbytes */
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08005800) /* Base @ of Page 11, 2 Kbytes */
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08006000) /* Base @ of Page 12, 2 Kbytes */
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08006800) /* Base @ of Page 13, 2 Kbytes */
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08007000) /* Base @ of Page 14, 2 Kbytes */
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08007800) /* Base @ of Page 15, 2 Kbytes */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08008000) /* Base @ of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08008800) /* Base @ of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08009000) /* Base @ of Page 18, 2 Kbytes */
#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08009800) /* Base @ of Page 19, 2 Kbytes */
#define ADDR_FLASH_PAGE_20    ((uint32_t)0x0800A000) /* Base @ of Page 20, 2 Kbytes */
#define ADDR_FLASH_PAGE_21    ((uint32_t)0x0800A800) /* Base @ of Page 21, 2 Kbytes */
#define ADDR_FLASH_PAGE_22    ((uint32_t)0x0800B000) /* Base @ of Page 22, 2 Kbytes */
#define ADDR_FLASH_PAGE_23    ((uint32_t)0x0800B800) /* Base @ of Page 23, 2 Kbytes */
#define ADDR_FLASH_PAGE_24    ((uint32_t)0x0800C000) /* Base @ of Page 24, 2 Kbytes */
#define ADDR_FLASH_PAGE_25    ((uint32_t)0x0800C800) /* Base @ of Page 25, 2 Kbytes */
#define ADDR_FLASH_PAGE_26    ((uint32_t)0x0800D000) /* Base @ of Page 26, 2 Kbytes */
#define ADDR_FLASH_PAGE_27    ((uint32_t)0x0800D800) /* Base @ of Page 27, 2 Kbytes */
#define ADDR_FLASH_PAGE_28    ((uint32_t)0x0800E000) /* Base @ of Page 28, 2 Kbytes */
#define ADDR_FLASH_PAGE_29    ((uint32_t)0x0800E800) /* Base @ of Page 29, 2 Kbytes */
#define ADDR_FLASH_PAGE_30    ((uint32_t)0x0800F000) /* Base @ of Page 30, 2 Kbytes */
#define ADDR_FLASH_PAGE_31    ((uint32_t)0x0800F800) /* Base @ of Page 31, 2 Kbytes */
#define ADDR_FLASH_PAGE_32    ((uint32_t)0x08010000) /* Base @ of Page 32, 2 Kbytes */
#define ADDR_FLASH_PAGE_33    ((uint32_t)0x08010800) /* Base @ of Page 33, 2 Kbytes */
#define ADDR_FLASH_PAGE_34    ((uint32_t)0x08011000) /* Base @ of Page 34, 2 Kbytes */
#define ADDR_FLASH_PAGE_35    ((uint32_t)0x08011800) /* Base @ of Page 35, 2 Kbytes */
#define ADDR_FLASH_PAGE_36    ((uint32_t)0x08012000) /* Base @ of Page 36, 2 Kbytes */
#define ADDR_FLASH_PAGE_37    ((uint32_t)0x08012800) /* Base @ of Page 37, 2 Kbytes */
#define ADDR_FLASH_PAGE_38    ((uint32_t)0x08013000) /* Base @ of Page 38, 2 Kbytes */
#define ADDR_FLASH_PAGE_39    ((uint32_t)0x08013800) /* Base @ of Page 39, 2 Kbytes */
#define ADDR_FLASH_PAGE_40    ((uint32_t)0x08014000) /* Base @ of Page 40, 2 Kbytes */
#define ADDR_FLASH_PAGE_41    ((uint32_t)0x08014800) /* Base @ of Page 41, 2 Kbytes */
#define ADDR_FLASH_PAGE_42    ((uint32_t)0x08015000) /* Base @ of Page 42, 2 Kbytes */
#define ADDR_FLASH_PAGE_43    ((uint32_t)0x08015800) /* Base @ of Page 43, 2 Kbytes */
#define ADDR_FLASH_PAGE_44    ((uint32_t)0x08016000) /* Base @ of Page 44, 2 Kbytes */
#define ADDR_FLASH_PAGE_45    ((uint32_t)0x08016800) /* Base @ of Page 45, 2 Kbytes */
#define ADDR_FLASH_PAGE_46    ((uint32_t)0x08017000) /* Base @ of Page 46, 2 Kbytes */
#define ADDR_FLASH_PAGE_47    ((uint32_t)0x08017800) /* Base @ of Page 47, 2 Kbytes */
#define ADDR_FLASH_PAGE_48    ((uint32_t)0x08018000) /* Base @ of Page 48, 2 Kbytes */
#define ADDR_FLASH_PAGE_49    ((uint32_t)0x08018800) /* Base @ of Page 49, 2 Kbytes */
#define ADDR_FLASH_PAGE_50    ((uint32_t)0x08019000) /* Base @ of Page 50, 2 Kbytes */
#define ADDR_FLASH_PAGE_51    ((uint32_t)0x08019800) /* Base @ of Page 51, 2 Kbytes */
#define ADDR_FLASH_PAGE_52    ((uint32_t)0x0801A000) /* Base @ of Page 52, 2 Kbytes */
#define ADDR_FLASH_PAGE_53    ((uint32_t)0x0801A800) /* Base @ of Page 53, 2 Kbytes */
#define ADDR_FLASH_PAGE_54    ((uint32_t)0x0801B000) /* Base @ of Page 54, 2 Kbytes */
#define ADDR_FLASH_PAGE_55    ((uint32_t)0x0801B800) /* Base @ of Page 55, 2 Kbytes */
#define ADDR_FLASH_PAGE_56    ((uint32_t)0x0801C000) /* Base @ of Page 56, 2 Kbytes */
#define ADDR_FLASH_PAGE_57    ((uint32_t)0x0801C800) /* Base @ of Page 57, 2 Kbytes */
#define ADDR_FLASH_PAGE_58    ((uint32_t)0x0801D000) /* Base @ of Page 58, 2 Kbytes */
#define ADDR_FLASH_PAGE_59    ((uint32_t)0x0801D800) /* Base @ of Page 59, 2 Kbytes */
#define ADDR_FLASH_PAGE_60    ((uint32_t)0x0801E000) /* Base @ of Page 60, 2 Kbytes */
#define ADDR_FLASH_PAGE_61    ((uint32_t)0x0801E800) /* Base @ of Page 61, 2 Kbytes */
#define ADDR_FLASH_PAGE_62    ((uint32_t)0x0801F000) /* Base @ of Page 62, 2 Kbytes */
#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0801F800) /* Base @ of Page 63, 2 Kbytes */
#define ADDR_FLASH_PAGE_64    ((uint32_t)0x08020000) /* Base @ of Page 64, 2 Kbytes */
#define ADDR_FLASH_PAGE_65    ((uint32_t)0x08020800) /* Base @ of Page 65, 2 Kbytes */
#define ADDR_FLASH_PAGE_66    ((uint32_t)0x08021000) /* Base @ of Page 66, 2 Kbytes */
#define ADDR_FLASH_PAGE_67    ((uint32_t)0x08021800) /* Base @ of Page 67, 2 Kbytes */
#define ADDR_FLASH_PAGE_68    ((uint32_t)0x08022000) /* Base @ of Page 68, 2 Kbytes */
#define ADDR_FLASH_PAGE_69    ((uint32_t)0x08022800) /* Base @ of Page 69, 2 Kbytes */
#define ADDR_FLASH_PAGE_70    ((uint32_t)0x08023000) /* Base @ of Page 70, 2 Kbytes */
#define ADDR_FLASH_PAGE_71    ((uint32_t)0x08023800) /* Base @ of Page 71, 2 Kbytes */
#define ADDR_FLASH_PAGE_72    ((uint32_t)0x08024000) /* Base @ of Page 72, 2 Kbytes */
#define ADDR_FLASH_PAGE_73    ((uint32_t)0x08024800) /* Base @ of Page 73, 2 Kbytes */
#define ADDR_FLASH_PAGE_74    ((uint32_t)0x08025000) /* Base @ of Page 74, 2 Kbytes */
#define ADDR_FLASH_PAGE_75    ((uint32_t)0x08025800) /* Base @ of Page 75, 2 Kbytes */
#define ADDR_FLASH_PAGE_76    ((uint32_t)0x08026000) /* Base @ of Page 76, 2 Kbytes */
#define ADDR_FLASH_PAGE_77    ((uint32_t)0x08026800) /* Base @ of Page 77, 2 Kbytes */
#define ADDR_FLASH_PAGE_78    ((uint32_t)0x08027000) /* Base @ of Page 78, 2 Kbytes */
#define ADDR_FLASH_PAGE_79    ((uint32_t)0x08027800) /* Base @ of Page 79, 2 Kbytes */
#define ADDR_FLASH_PAGE_80    ((uint32_t)0x08028000) /* Base @ of Page 80, 2 Kbytes */
#define ADDR_FLASH_PAGE_81    ((uint32_t)0x08028800) /* Base @ of Page 81, 2 Kbytes */
#define ADDR_FLASH_PAGE_82    ((uint32_t)0x08029000) /* Base @ of Page 82, 2 Kbytes */
#define ADDR_FLASH_PAGE_83    ((uint32_t)0x08029800) /* Base @ of Page 83, 2 Kbytes */
#define ADDR_FLASH_PAGE_84    ((uint32_t)0x0802A000) /* Base @ of Page 84, 2 Kbytes */
#define ADDR_FLASH_PAGE_85    ((uint32_t)0x0802A800) /* Base @ of Page 85, 2 Kbytes */
#define ADDR_FLASH_PAGE_86    ((uint32_t)0x0802B000) /* Base @ of Page 86, 2 Kbytes */
#define ADDR_FLASH_PAGE_87    ((uint32_t)0x0802B800) /* Base @ of Page 87, 2 Kbytes */
#define ADDR_FLASH_PAGE_88    ((uint32_t)0x0802C000) /* Base @ of Page 88, 2 Kbytes */
#define ADDR_FLASH_PAGE_89    ((uint32_t)0x0802C800) /* Base @ of Page 89, 2 Kbytes */
#define ADDR_FLASH_PAGE_90    ((uint32_t)0x0802D000) /* Base @ of Page 90, 2 Kbytes */
#define ADDR_FLASH_PAGE_91    ((uint32_t)0x0802D800) /* Base @ of Page 91, 2 Kbytes */
#define ADDR_FLASH_PAGE_92    ((uint32_t)0x0802E000) /* Base @ of Page 92, 2 Kbytes */
#define ADDR_FLASH_PAGE_93    ((uint32_t)0x0802E800) /* Base @ of Page 93, 2 Kbytes */
#define ADDR_FLASH_PAGE_94    ((uint32_t)0x0802F000) /* Base @ of Page 94, 2 Kbytes */
#define ADDR_FLASH_PAGE_95    ((uint32_t)0x0802F800) /* Base @ of Page 95, 2 Kbytes */
#define ADDR_FLASH_PAGE_96    ((uint32_t)0x08030000) /* Base @ of Page 96, 2 Kbytes */
#define ADDR_FLASH_PAGE_97    ((uint32_t)0x08030800) /* Base @ of Page 97, 2 Kbytes */
#define ADDR_FLASH_PAGE_98    ((uint32_t)0x08031000) /* Base @ of Page 98, 2 Kbytes */
#define ADDR_FLASH_PAGE_99    ((uint32_t)0x08031800) /* Base @ of Page 99, 2 Kbytes */
#define ADDR_FLASH_PAGE_100   ((uint32_t)0x08032000) /* Base @ of Page 100, 2 Kbytes */
#define ADDR_FLASH_PAGE_101   ((uint32_t)0x08032800) /* Base @ of Page 101, 2 Kbytes */
#define ADDR_FLASH_PAGE_102   ((uint32_t)0x08033000) /* Base @ of Page 102, 2 Kbytes */
#define ADDR_FLASH_PAGE_103   ((uint32_t)0x08033800) /* Base @ of Page 103, 2 Kbytes */
#define ADDR_FLASH_PAGE_104   ((uint32_t)0x08034000) /* Base @ of Page 104, 2 Kbytes */
#define ADDR_FLASH_PAGE_105   ((uint32_t)0x08034800) /* Base @ of Page 105, 2 Kbytes */
#define ADDR_FLASH_PAGE_106   ((uint32_t)0x08035000) /* Base @ of Page 106, 2 Kbytes */
#define ADDR_FLASH_PAGE_107   ((uint32_t)0x08035800) /* Base @ of Page 107, 2 Kbytes */
#define ADDR_FLASH_PAGE_108   ((uint32_t)0x08036000) /* Base @ of Page 108, 2 Kbytes */
#define ADDR_FLASH_PAGE_109   ((uint32_t)0x08036800) /* Base @ of Page 109, 2 Kbytes */
#define ADDR_FLASH_PAGE_110   ((uint32_t)0x08037000) /* Base @ of Page 110, 2 Kbytes */
#define ADDR_FLASH_PAGE_111   ((uint32_t)0x08037800) /* Base @ of Page 111, 2 Kbytes */
#define ADDR_FLASH_PAGE_112   ((uint32_t)0x08038000) /* Base @ of Page 112, 2 Kbytes */
#define ADDR_FLASH_PAGE_113   ((uint32_t)0x08038800) /* Base @ of Page 113, 2 Kbytes */
#define ADDR_FLASH_PAGE_114   ((uint32_t)0x08039000) /* Base @ of Page 114, 2 Kbytes */
#define ADDR_FLASH_PAGE_115   ((uint32_t)0x08039800) /* Base @ of Page 115, 2 Kbytes */
#define ADDR_FLASH_PAGE_116   ((uint32_t)0x0803A000) /* Base @ of Page 116, 2 Kbytes */
#define ADDR_FLASH_PAGE_117   ((uint32_t)0x0803A800) /* Base @ of Page 117, 2 Kbytes */
#define ADDR_FLASH_PAGE_118   ((uint32_t)0x0803B000) /* Base @ of Page 118, 2 Kbytes */
#define ADDR_FLASH_PAGE_119   ((uint32_t)0x0803B800) /* Base @ of Page 119, 2 Kbytes */
#define ADDR_FLASH_PAGE_120   ((uint32_t)0x0803C000) /* Base @ of Page 120, 2 Kbytes */
#define ADDR_FLASH_PAGE_121   ((uint32_t)0x0803C800) /* Base @ of Page 121, 2 Kbytes */
#define ADDR_FLASH_PAGE_122   ((uint32_t)0x0803D000) /* Base @ of Page 122, 2 Kbytes */
#define ADDR_FLASH_PAGE_123   ((uint32_t)0x0803D800) /* Base @ of Page 123, 2 Kbytes */
#define ADDR_FLASH_PAGE_124   ((uint32_t)0x0803E000) /* Base @ of Page 124, 2 Kbytes */
#define ADDR_FLASH_PAGE_125   ((uint32_t)0x0803E800) /* Base @ of Page 125, 2 Kbytes */
#define ADDR_FLASH_PAGE_126   ((uint32_t)0x0803F000) /* Base @ of Page 126, 2 Kbytes */
#define ADDR_FLASH_PAGE_127   ((uint32_t)0x0803F800) /* Base @ of Page 127, 2 Kbytes */
#define ADDR_FLASH_PAGE_128   ((uint32_t)0x08040000) /* Base @ of Page 128, 2 Kbytes */
#define ADDR_FLASH_PAGE_129   ((uint32_t)0x08040800) /* Base @ of Page 129, 2 Kbytes */
#define ADDR_FLASH_PAGE_130   ((uint32_t)0x08041000) /* Base @ of Page 130, 2 Kbytes */
#define ADDR_FLASH_PAGE_131   ((uint32_t)0x08041800) /* Base @ of Page 131, 2 Kbytes */
#define ADDR_FLASH_PAGE_132   ((uint32_t)0x08042000) /* Base @ of Page 132, 2 Kbytes */
#define ADDR_FLASH_PAGE_133   ((uint32_t)0x08042800) /* Base @ of Page 133, 2 Kbytes */
#define ADDR_FLASH_PAGE_134   ((uint32_t)0x08043000) /* Base @ of Page 134, 2 Kbytes */
#define ADDR_FLASH_PAGE_135   ((uint32_t)0x08043800) /* Base @ of Page 135, 2 Kbytes */
#define ADDR_FLASH_PAGE_136   ((uint32_t)0x08044000) /* Base @ of Page 136, 2 Kbytes */
#define ADDR_FLASH_PAGE_137   ((uint32_t)0x08044800) /* Base @ of Page 137, 2 Kbytes */
#define ADDR_FLASH_PAGE_138   ((uint32_t)0x08045000) /* Base @ of Page 138, 2 Kbytes */
#define ADDR_FLASH_PAGE_139   ((uint32_t)0x08045800) /* Base @ of Page 139, 2 Kbytes */
#define ADDR_FLASH_PAGE_140   ((uint32_t)0x08046000) /* Base @ of Page 140, 2 Kbytes */
#define ADDR_FLASH_PAGE_141   ((uint32_t)0x08046800) /* Base @ of Page 141, 2 Kbytes */
#define ADDR_FLASH_PAGE_142   ((uint32_t)0x08047000) /* Base @ of Page 142, 2 Kbytes */
#define ADDR_FLASH_PAGE_143   ((uint32_t)0x08047800) /* Base @ of Page 143, 2 Kbytes */
#define ADDR_FLASH_PAGE_144   ((uint32_t)0x08048000) /* Base @ of Page 144, 2 Kbytes */
#define ADDR_FLASH_PAGE_145   ((uint32_t)0x08048800) /* Base @ of Page 145, 2 Kbytes */
#define ADDR_FLASH_PAGE_146   ((uint32_t)0x08049000) /* Base @ of Page 146, 2 Kbytes */
#define ADDR_FLASH_PAGE_147   ((uint32_t)0x08049800) /* Base @ of Page 147, 2 Kbytes */
#define ADDR_FLASH_PAGE_148   ((uint32_t)0x0804a000) /* Base @ of Page 148, 2 Kbytes */
#define ADDR_FLASH_PAGE_149   ((uint32_t)0x0804a800) /* Base @ of Page 149, 2 Kbytes */
#define ADDR_FLASH_PAGE_150   ((uint32_t)0x0804b000) /* Base @ of Page 150, 2 Kbytes */
#define ADDR_FLASH_PAGE_151   ((uint32_t)0x0804b800) /* Base @ of Page 151, 2 Kbytes */
#define ADDR_FLASH_PAGE_152   ((uint32_t)0x0804c000) /* Base @ of Page 152, 2 Kbytes */
#define ADDR_FLASH_PAGE_153   ((uint32_t)0x0804c800) /* Base @ of Page 153, 2 Kbytes */
#define ADDR_FLASH_PAGE_154   ((uint32_t)0x0804d000) /* Base @ of Page 154, 2 Kbytes */
#define ADDR_FLASH_PAGE_155   ((uint32_t)0x0804d800) /* Base @ of Page 155, 2 Kbytes */
#define ADDR_FLASH_PAGE_156   ((uint32_t)0x0804e000) /* Base @ of Page 156, 2 Kbytes */
#define ADDR_FLASH_PAGE_157   ((uint32_t)0x0804e800) /* Base @ of Page 157, 2 Kbytes */
#define ADDR_FLASH_PAGE_158   ((uint32_t)0x0804f000) /* Base @ of Page 158, 2 Kbytes */
#define ADDR_FLASH_PAGE_159   ((uint32_t)0x0804f800) /* Base @ of Page 159, 2 Kbytes */
#define ADDR_FLASH_PAGE_160   ((uint32_t)0x08050000) /* Base @ of Page 160, 2 Kbytes */
#define ADDR_FLASH_PAGE_161   ((uint32_t)0x08050800) /* Base @ of Page 161, 2 Kbytes */
#define ADDR_FLASH_PAGE_162   ((uint32_t)0x08051000) /* Base @ of Page 162, 2 Kbytes */
#define ADDR_FLASH_PAGE_163   ((uint32_t)0x08051800) /* Base @ of Page 163, 2 Kbytes */
#define ADDR_FLASH_PAGE_164   ((uint32_t)0x08052000) /* Base @ of Page 164, 2 Kbytes */
#define ADDR_FLASH_PAGE_165   ((uint32_t)0x08052800) /* Base @ of Page 165, 2 Kbytes */
#define ADDR_FLASH_PAGE_166   ((uint32_t)0x08053000) /* Base @ of Page 166, 2 Kbytes */
#define ADDR_FLASH_PAGE_167   ((uint32_t)0x08053800) /* Base @ of Page 167, 2 Kbytes */
#define ADDR_FLASH_PAGE_168   ((uint32_t)0x08054000) /* Base @ of Page 168, 2 Kbytes */
#define ADDR_FLASH_PAGE_169   ((uint32_t)0x08054800) /* Base @ of Page 169, 2 Kbytes */
#define ADDR_FLASH_PAGE_170   ((uint32_t)0x08055000) /* Base @ of Page 170, 2 Kbytes */
#define ADDR_FLASH_PAGE_171   ((uint32_t)0x08055800) /* Base @ of Page 171, 2 Kbytes */
#define ADDR_FLASH_PAGE_172   ((uint32_t)0x08056000) /* Base @ of Page 172, 2 Kbytes */
#define ADDR_FLASH_PAGE_173   ((uint32_t)0x08056800) /* Base @ of Page 173, 2 Kbytes */
#define ADDR_FLASH_PAGE_174   ((uint32_t)0x08057000) /* Base @ of Page 174, 2 Kbytes */
#define ADDR_FLASH_PAGE_175   ((uint32_t)0x08057800) /* Base @ of Page 175, 2 Kbytes */
#define ADDR_FLASH_PAGE_176   ((uint32_t)0x08058000) /* Base @ of Page 176, 2 Kbytes */
#define ADDR_FLASH_PAGE_177   ((uint32_t)0x08058800) /* Base @ of Page 177, 2 Kbytes */
#define ADDR_FLASH_PAGE_178   ((uint32_t)0x08059000) /* Base @ of Page 178, 2 Kbytes */
#define ADDR_FLASH_PAGE_179   ((uint32_t)0x08059800) /* Base @ of Page 179, 2 Kbytes */
#define ADDR_FLASH_PAGE_180   ((uint32_t)0x0805a000) /* Base @ of Page 180, 2 Kbytes */
#define ADDR_FLASH_PAGE_181   ((uint32_t)0x0805a800) /* Base @ of Page 181, 2 Kbytes */
#define ADDR_FLASH_PAGE_182   ((uint32_t)0x0805b000) /* Base @ of Page 182, 2 Kbytes */
#define ADDR_FLASH_PAGE_183   ((uint32_t)0x0805b800) /* Base @ of Page 183, 2 Kbytes */
#define ADDR_FLASH_PAGE_184   ((uint32_t)0x0805c000) /* Base @ of Page 184, 2 Kbytes */
#define ADDR_FLASH_PAGE_185   ((uint32_t)0x0805c800) /* Base @ of Page 185, 2 Kbytes */
#define ADDR_FLASH_PAGE_186   ((uint32_t)0x0805d000) /* Base @ of Page 186, 2 Kbytes */
#define ADDR_FLASH_PAGE_187   ((uint32_t)0x0805d800) /* Base @ of Page 187, 2 Kbytes */
#define ADDR_FLASH_PAGE_188   ((uint32_t)0x0805e000) /* Base @ of Page 188, 2 Kbytes */
#define ADDR_FLASH_PAGE_189   ((uint32_t)0x0805e800) /* Base @ of Page 189, 2 Kbytes */
#define ADDR_FLASH_PAGE_190   ((uint32_t)0x0805f000) /* Base @ of Page 190, 2 Kbytes */
#define ADDR_FLASH_PAGE_191   ((uint32_t)0x0805f800) /* Base @ of Page 191, 2 Kbytes */
#define ADDR_FLASH_PAGE_192   ((uint32_t)0x08060000) /* Base @ of Page 192, 2 Kbytes */
#define ADDR_FLASH_PAGE_193   ((uint32_t)0x08060800) /* Base @ of Page 193, 2 Kbytes */
#define ADDR_FLASH_PAGE_194   ((uint32_t)0x08061000) /* Base @ of Page 194, 2 Kbytes */
#define ADDR_FLASH_PAGE_195   ((uint32_t)0x08061800) /* Base @ of Page 195, 2 Kbytes */
#define ADDR_FLASH_PAGE_196   ((uint32_t)0x08062000) /* Base @ of Page 196, 2 Kbytes */
#define ADDR_FLASH_PAGE_197   ((uint32_t)0x08062800) /* Base @ of Page 197, 2 Kbytes */
#define ADDR_FLASH_PAGE_198   ((uint32_t)0x08063000) /* Base @ of Page 198, 2 Kbytes */
#define ADDR_FLASH_PAGE_199   ((uint32_t)0x08063800) /* Base @ of Page 199, 2 Kbytes */
#define ADDR_FLASH_PAGE_200   ((uint32_t)0x08064000) /* Base @ of Page 200, 2 Kbytes */
#define ADDR_FLASH_PAGE_201   ((uint32_t)0x08064800) /* Base @ of Page 201, 2 Kbytes */
#define ADDR_FLASH_PAGE_202   ((uint32_t)0x08065000) /* Base @ of Page 202, 2 Kbytes */
#define ADDR_FLASH_PAGE_203   ((uint32_t)0x08065800) /* Base @ of Page 203, 2 Kbytes */
#define ADDR_FLASH_PAGE_204   ((uint32_t)0x08066000) /* Base @ of Page 204, 2 Kbytes */
#define ADDR_FLASH_PAGE_205   ((uint32_t)0x08066800) /* Base @ of Page 205, 2 Kbytes */
#define ADDR_FLASH_PAGE_206   ((uint32_t)0x08067000) /* Base @ of Page 206, 2 Kbytes */
#define ADDR_FLASH_PAGE_207   ((uint32_t)0x08067800) /* Base @ of Page 207, 2 Kbytes */
#define ADDR_FLASH_PAGE_208   ((uint32_t)0x08068000) /* Base @ of Page 208, 2 Kbytes */
#define ADDR_FLASH_PAGE_209   ((uint32_t)0x08068800) /* Base @ of Page 209, 2 Kbytes */
#define ADDR_FLASH_PAGE_210   ((uint32_t)0x08069000) /* Base @ of Page 210, 2 Kbytes */
#define ADDR_FLASH_PAGE_211   ((uint32_t)0x08069800) /* Base @ of Page 211, 2 Kbytes */
#define ADDR_FLASH_PAGE_212   ((uint32_t)0x0806a000) /* Base @ of Page 212, 2 Kbytes */
#define ADDR_FLASH_PAGE_213   ((uint32_t)0x0806a800) /* Base @ of Page 213, 2 Kbytes */
#define ADDR_FLASH_PAGE_214   ((uint32_t)0x0806b000) /* Base @ of Page 214, 2 Kbytes */
#define ADDR_FLASH_PAGE_215   ((uint32_t)0x0806b800) /* Base @ of Page 215, 2 Kbytes */
#define ADDR_FLASH_PAGE_216   ((uint32_t)0x0806c000) /* Base @ of Page 216, 2 Kbytes */
#define ADDR_FLASH_PAGE_217   ((uint32_t)0x0806c800) /* Base @ of Page 217, 2 Kbytes */
#define ADDR_FLASH_PAGE_218   ((uint32_t)0x0806d000) /* Base @ of Page 218, 2 Kbytes */
#define ADDR_FLASH_PAGE_219   ((uint32_t)0x0806d800) /* Base @ of Page 219, 2 Kbytes */
#define ADDR_FLASH_PAGE_220   ((uint32_t)0x0806e000) /* Base @ of Page 220, 2 Kbytes */
#define ADDR_FLASH_PAGE_221   ((uint32_t)0x0806e800) /* Base @ of Page 221, 2 Kbytes */
#define ADDR_FLASH_PAGE_222   ((uint32_t)0x0806f000) /* Base @ of Page 222, 2 Kbytes */
#define ADDR_FLASH_PAGE_223   ((uint32_t)0x0806f800) /* Base @ of Page 223, 2 Kbytes */
#define ADDR_FLASH_PAGE_224   ((uint32_t)0x08070000) /* Base @ of Page 224, 2 Kbytes */
#define ADDR_FLASH_PAGE_225   ((uint32_t)0x08070800) /* Base @ of Page 225, 2 Kbytes */
#define ADDR_FLASH_PAGE_226   ((uint32_t)0x08071000) /* Base @ of Page 226, 2 Kbytes */
#define ADDR_FLASH_PAGE_227   ((uint32_t)0x08071800) /* Base @ of Page 227, 2 Kbytes */
#define ADDR_FLASH_PAGE_228   ((uint32_t)0x08072000) /* Base @ of Page 228, 2 Kbytes */
#define ADDR_FLASH_PAGE_229   ((uint32_t)0x08072800) /* Base @ of Page 229, 2 Kbytes */
#define ADDR_FLASH_PAGE_230   ((uint32_t)0x08073000) /* Base @ of Page 230, 2 Kbytes */
#define ADDR_FLASH_PAGE_231   ((uint32_t)0x08073800) /* Base @ of Page 231, 2 Kbytes */
#define ADDR_FLASH_PAGE_232   ((uint32_t)0x08074000) /* Base @ of Page 232, 2 Kbytes */
#define ADDR_FLASH_PAGE_233   ((uint32_t)0x08074800) /* Base @ of Page 233, 2 Kbytes */
#define ADDR_FLASH_PAGE_234   ((uint32_t)0x08075000) /* Base @ of Page 234, 2 Kbytes */
#define ADDR_FLASH_PAGE_235   ((uint32_t)0x08075800) /* Base @ of Page 235, 2 Kbytes */
#define ADDR_FLASH_PAGE_236   ((uint32_t)0x08076000) /* Base @ of Page 236, 2 Kbytes */
#define ADDR_FLASH_PAGE_237   ((uint32_t)0x08076800) /* Base @ of Page 237, 2 Kbytes */
#define ADDR_FLASH_PAGE_238   ((uint32_t)0x08077000) /* Base @ of Page 238, 2 Kbytes */
#define ADDR_FLASH_PAGE_239   ((uint32_t)0x08077800) /* Base @ of Page 239, 2 Kbytes */
#define ADDR_FLASH_PAGE_240   ((uint32_t)0x08078000) /* Base @ of Page 240, 2 Kbytes */
#define ADDR_FLASH_PAGE_241   ((uint32_t)0x08078800) /* Base @ of Page 241, 2 Kbytes */
#define ADDR_FLASH_PAGE_242   ((uint32_t)0x08079000) /* Base @ of Page 242, 2 Kbytes */
#define ADDR_FLASH_PAGE_243   ((uint32_t)0x08079800) /* Base @ of Page 243, 2 Kbytes */
#define ADDR_FLASH_PAGE_244   ((uint32_t)0x0807a000) /* Base @ of Page 244, 2 Kbytes */
#define ADDR_FLASH_PAGE_245   ((uint32_t)0x0807a800) /* Base @ of Page 245, 2 Kbytes */
#define ADDR_FLASH_PAGE_246   ((uint32_t)0x0807b000) /* Base @ of Page 246, 2 Kbytes */
#define ADDR_FLASH_PAGE_247   ((uint32_t)0x0807b800) /* Base @ of Page 247, 2 Kbytes */
#define ADDR_FLASH_PAGE_248   ((uint32_t)0x0807c000) /* Base @ of Page 248, 2 Kbytes */
#define ADDR_FLASH_PAGE_249   ((uint32_t)0x0807c800) /* Base @ of Page 249, 2 Kbytes */
#define ADDR_FLASH_PAGE_250   ((uint32_t)0x0807d000) /* Base @ of Page 250, 2 Kbytes */
#define ADDR_FLASH_PAGE_251   ((uint32_t)0x0807d800) /* Base @ of Page 251, 2 Kbytes */
#define ADDR_FLASH_PAGE_252   ((uint32_t)0x0807e000) /* Base @ of Page 252, 2 Kbytes */
#define ADDR_FLASH_PAGE_253   ((uint32_t)0x0807e800) /* Base @ of Page 253, 2 Kbytes */
#define ADDR_FLASH_PAGE_254   ((uint32_t)0x0807f000) /* Base @ of Page 254, 2 Kbytes */
#define ADDR_FLASH_PAGE_255   ((uint32_t)0x0807f800) /* Base @ of Page 255, 2 Kbytes */
#define FLASH_USER_START_ADDR ADDR_FLASH_PAGE_22 // Start @ of user Flash area
#define FLASH_USER_END_ADDR   ADDR_FLASH_PAGE_255 + FLASH_PAGE_SIZE - 1 // End @ of user Flash area

#define Speed_GPIO_PIN               GPIOA
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef Master_BMS_RxHeader;
CAN_FilterTypeDef sFilterConfig;
FLASH_EraseInitTypeDef EraseInitStruct;
uint8_t Master_BMS_Rx_data[8];
uint8_t Bluetooth_TxData[8];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint64_t Voltage, Min_Voltage, Current, Max_temperature, Min_temperature, Max_Current, Max_temperature_adc, Min_temperature_adc;
static uint32_t FirstPage, NbOfPages, BankNumber, Address, Mailbox;
static uint32_t page = 0, PAGEError = 0, ADC_DMA_data[4];
static double Operating_time=0, Operating_time_Update=0;
static float Write_Flag=0, Operating_Flag=0;
uint64_t Wx_data0, Wx_data1;
static uint8_t tx_data[8];


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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  Master_BMS_CAN_Configuration(&sFilterConfig, &Master_BMS_RxHeader);

  CAN_TX_Config();

  Data_Logging_Config(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);

  Erase_data(&EraseInitStruct);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	 		  Operating_time= HAL_GetTick();


	 		  Master_BMS_CAN_Receive(&hcan1, &Master_BMS_RxHeader, Master_BMS_Rx_data);

	 		  Current_ADC_DMA(&hadc1, ADC_DMA_data, 1);

	 		  Logging_Online();




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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
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

void CAN_TX_Config(void)
{
     TxHeader.StdId = 0x321;
     TxHeader.ExtId = 0x00;
     TxHeader.IDE = CAN_ID_STD;
     TxHeader.RTR = CAN_RTR_DATA;
     TxHeader.DLC = 8;
     TxHeader.TransmitGlobalTime = DISABLE;

}

void Transmit_Data(CAN_TxHeaderTypeDef* TxHeader)
{

   uint8_t Current_1 = ((Max_Current & 0xff00)>>8);
   uint8_t Current_2 = (Max_Current & 0x00ff);
   tx_data[0] = Current_1;
   tx_data[1] = Current_2;

   {
      Mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

      if(Mailbox){
      HAL_CAN_AddTxMessage(&hcan1, TxHeader, tx_data, &Mailbox);
      }
   }

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
     Max_temperature_adc = ((uint64_t)rx_data[2]<<8) + ((uint64_t)rx_data[3]);
     Min_temperature_adc = ((uint64_t)rx_data[4]<<8) + ((uint64_t)rx_data[5]);
     Max_temperature = (4.066/(1000000000)*(double)(Max_temperature_adc*Max_temperature_adc*Max_temperature_adc)) - (2.224/(100000)*(double)(Max_temperature_adc*Max_temperature_adc))+(0.063*(double)(Max_temperature_adc))-27.71;
     Min_temperature = (4.066/(1000000000)*(double)(Min_temperature_adc*Min_temperature_adc*Min_temperature_adc)) - (2.224/(100000)*(double)(Min_temperature_adc*Min_temperature_adc))+(0.063*(double)(Min_temperature_adc))-27.71;
     Voltage = ((uint64_t)rx_data[0]<<8) + ((uint64_t)rx_data[1]);
     rx_data[6] = ((uint8_t)((Max_Current>>8)&0x000F));
     rx_data[7] = ((uint8_t)(Max_Current&0x000F));

     if(Operating_Flag == 0 )
     {
    	 Min_Voltage = 0x9999;
     }

     if (Min_Voltage>Voltage)
     {
   		 Min_Voltage = Voltage;
     }



}

void Current_ADC_DMA(ADC_HandleTypeDef *hadc, uint32_t data[], uint32_t Length)
{
	      HAL_ADC_Start_DMA(hadc, data, Length);
	      Current = ((((float)data[0])*0.121)+10.12);
	      if( Operating_Flag == 0 )
	         {
	        	 Max_Current = 0;
	         }

	      if(Max_Current<Current)
	         {
	    	     Max_Current=Current;
	         }


}

void Data_Logging_Config(uint32_t Logging_Start_Page_ADDR, uint32_t Logging_END_Page_ADDR)
{
	 HAL_FLASH_Unlock();

	  FirstPage = GetPage(Logging_Start_Page_ADDR); // Get the 1st page to erase
	  NbOfPages = GetPage(Logging_END_Page_ADDR) - FirstPage + 1; // Get the number of pages to erase form 1st page
	  BankNumber = GetBank(Logging_Start_Page_ADDR); // Get the Bank
	  Address = FLASH_USER_START_ADDR;

}

void Logging_Online(void)
{


		 		  HAL_FLASH_Unlock();
		 		 if((Operating_time-Operating_time_Update)>=1000)
		 			   {
		 			          Operating_time_Update = Operating_time;
		 			 		  Write_data( &Wx_data0, &Wx_data1);
		 			 		  Bluetooth_Tx(Master_BMS_Rx_data);
		 			 		  Transmit_Data(&TxHeader);

		 			 		  Operating_Flag=0;
		 	   		   }
		 	     else
		 		  	   {
		 			 		  Operating_Flag=1;
		 		  	   }




	 			  HAL_FLASH_Lock();
	 			  Write_Flag = (uint64_t)(Operating_time/1000);


}

void Write_data(uint64_t *Wx_data0, uint64_t*Wx_data1)
{
		  *(Wx_data0) = ((Min_Voltage << 48) | (Max_Current <<32) |(Max_temperature << 16) | (Min_temperature));

	 if((HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(Wx_data0))==HAL_OK))

		  {
			  Address = Address + 8;
			  *(Wx_data0)=0;
		  }
		  else
		  {
			  HAL_FLASH_GetError();
		  }
}

void Erase_data(FLASH_EraseInitTypeDef *EraseInitType)
{
	  // Fill EraseInit structure
	  EraseInitType->TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitType->Banks = BankNumber;
	  EraseInitType->Page = FirstPage;
	  EraseInitType->NbPages = NbOfPages;

	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	  {
		  // Error Occurred while writing data in Flash memory.
		  HAL_FLASH_GetError();

	  }
}

uint32_t GetPage(uint32_t Addr)
{

	if(Addr<(FLASH_BASE + FLASH_BANK_SIZE))
	{
		// Bank 1
		page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
	}
	else
	{
		page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
	}
	return page;
}

uint32_t GetBank(uint32_t Addr)
{
	return FLASH_BANK_1;
}

void Bluetooth_Tx(uint8_t *Parameter)
{


	for(int i=0; i<8; i++)
	{

		Bluetooth_TxData[i] = *(Parameter + i); // full voltage value

	}



   HAL_UART_Transmit(&huart4, Bluetooth_TxData, 8, 1000);


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
