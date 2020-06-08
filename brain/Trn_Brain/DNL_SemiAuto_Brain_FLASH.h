#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_6_BANK2      /* Start @ of user Flash area Bank1 */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_SECTOR_7_BANK2 - 1)  /* End @ of user Flash area Bank1*/


uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0, Index = 0;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint64_t data64 = 0;
__IO uint64_t DT50Left = 0;
__IO uint64_t DT50Pitc = 0;
__IO uint64_t DT50Rigt = 0;
uint16_t DT50Left_1000;
uint16_t DT50Left_2000;
uint16_t DT50Rigt_1000;
uint16_t DT50Rigt_2000;
uint16_t DT50Pitc_1000;
uint16_t DT50Pitc_2000;

//uint64_t FlashWord[4] = { 0x1020304050607080,
//                          0x1121314151617181,
//                          0x1222324252627282,
//                          0x1323334353637383
//                        };

uint16_t adcLeft_1000 = 2000;
uint16_t adcLeft_2000 = 3000;
uint16_t adcPitc_1000 = 2200;
uint16_t adcPitc_2000 = 3200;
uint16_t adcRigt_1000 = 2100;
uint16_t adcRigt_2000 = 3100;

uint64_t FlashWord[4];

FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if(((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
     ((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2)))
  {
    sector = FLASH_SECTOR_0;
  }
  else if(((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2)))
  {
    sector = FLASH_SECTOR_1;
  }
  else if(((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2)))
  {
    sector = FLASH_SECTOR_2;
  }
  else if(((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2)))
  {
    sector = FLASH_SECTOR_3;
  }
  else if(((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2)))
  {
    sector = FLASH_SECTOR_4;
  }
  else if(((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2)))
  {
    sector = FLASH_SECTOR_5;
  }
  else if(((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2)))
  {
    sector = FLASH_SECTOR_6;
  }
  else if(((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
          ((Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7_BANK2)))
  {
     sector = FLASH_SECTOR_7;
  }
  else
  {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}

void writeFLASH()
{
	FlashWord[0] = (adcLeft_2000<<0)|(adcLeft_1000<<16);
	FlashWord[1] = (adcPitc_2000<<0)|(adcPitc_1000<<16);
	FlashWord[2] = (adcRigt_2000<<0)|(adcRigt_1000<<16);

//	FlashWord[0] = 0;
//	FlashWord[1] = 0;
//	FlashWord[2] = 0;

	HAL_FLASH_Unlock();
	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Banks         = FLASH_BANK_2;
	EraseInitStruct.Sector        = FirstSector;
	EraseInitStruct.NbSectors     = NbOfSectors;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
	/*
	  Error occurred while sector erase.
	  User can add here some code to deal with this error.
	  SECTORError will contain the faulty sector and then to know the code error on this sector,
	  user can call function 'HAL_FLASH_GetError()'
	*/
	/* Infinite loop */
		while (1)
		{
		}
	}

	Address = FLASH_USER_START_ADDR;

	while (Address < FLASH_USER_END_ADDR)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)FlashWord)) == HAL_OK)
		{
			Address = Address + 32; /* increment for the next Flash word*/
		}
		else
		{
		  /* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
		  while (1)
		  {
		  }
		}
	}
	HAL_FLASH_Lock();
}

void readFLASH()
{
	Address = FLASH_USER_START_ADDR;
	MemoryProgramStatus = 0x0;

	DT50Left = *(uint64_t*) FLASH_USER_START_ADDR;
	DT50Left_2000 = (DT50Left>>0)&0xFFFF;
	DT50Left_1000 = (DT50Left>>16)&0xFFFF;
	__DSB();
	DT50Pitc = *(uint64_t*) (FLASH_USER_START_ADDR+8);
	DT50Pitc_2000 = (DT50Pitc>>0)&0xFFFF;
	DT50Pitc_1000 = (DT50Pitc>>16)&0xFFFF;
	__DSB();
	DT50Rigt = *(uint64_t*) (FLASH_USER_START_ADDR+16);
	DT50Rigt_2000 = (DT50Rigt>>0)&0xFFFF;
	DT50Rigt_1000 = (DT50Rigt>>16)&0xFFFF;
	__DSB();
//	word4 = *(uint64_t*) (FLASH_USER_START_ADDR+24);
//	__DSB();
//	while (Address < FLASH_USER_END_ADDR)
//	{
//		tracking++;
//	for(Index = 0; Index<4; Index++)
//	{
//
//	  data64 = *(uint64_t*)Address;
//	  __DSB();
//	  if(data64 != FlashWord[Index])
//	  {
//		MemoryProgramStatus++;
//	  }
//	  Address +=8;
//		}
//	}
}

void getSample()
{
	int sumADC_Left, sumADC_Pitc, sumADC_Rigt;

	//lấy mẫu left 1000
	sumADC_Left = 0;
	while(0)	//chờ nhấn nút
	{}
	HAL_Delay(100);
	for(int i = 0; i < 100; ++i)
	{
		sumADC_Left += adc3Value[_left];
	}
	adcLeft_1000 = sumADC_Left/100;

	//lấy mẫu left 2000
	sumADC_Left = 0;
	while(0)	//chờ nhấn nút
	{}
	HAL_Delay(100);
	for(int i = 0; i < 100; ++i)
	{
		sumADC_Left += adc3Value[_left];
	}
	adcLeft_2000 = sumADC_Left/100;

	//lấy mẫu pitc 1000
	sumADC_Pitc = 0;
	while(0)	//chờ nhấn nút
	{}
	HAL_Delay(100);
	for(int i = 0; i < 100; ++i)
	{
		sumADC_Pitc += adc3Value[_pitc];
	}
	adcPitc_1000 = sumADC_Pitc/100;

	//lấy mẫu pitc 2000
	sumADC_Pitc = 0;
	while(0)	//chờ nhấn nút
	{}
	HAL_Delay(100);
	for(int i = 0; i < 100; ++i)
	{
		sumADC_Pitc += adc3Value[_pitc];
	}
	adcPitc_2000 = sumADC_Pitc/100;

	//lấy mẫu rigt 1000
	sumADC_Rigt = 0;
	while(0)	//chờ nhấn nút
	{}
	HAL_Delay(100);
	for(int i = 0; i < 100; ++i)
	{
		sumADC_Rigt += adc3Value[_pitc];
	}
	adcPitc_1000 = sumADC_Rigt/100;

	//lấy mẫu rigt 2000
	sumADC_Rigt = 0;
	while(0)	//chờ nhấn nút
	{}
	HAL_Delay(100);
	for(int i = 0; i < 100; ++i)
	{
		sumADC_Rigt += adc3Value[_pitc];
	}
	adcPitc_2000 = sumADC_Rigt/100;

	//ghi vào flash
	writeFLASH();
}




















































//#include "stm32h7xx_hal.h"
////Typedefs
////1. data size
//typedef enum
//{
//	DATA_TYPE_8=0,
//	DATA_TYPE_16,
//	DATA_TYPE_32,
//}DataTypeDef;
//
////Private variables
////1. sector start address
//static uint32_t MY_SectorAddrs;
//static uint8_t MY_SectorNum;
//static uint8_t MY_Bank;
//
////functions definitions
////1. Erase Sector
//static void MY_FLASH_EraseSector(uint8_t FLASH_BANK, uint8_t FLASH_VOLTAGE_RANGE)
//{
//	HAL_FLASH_Unlock();
//	//Erase the required Flash sector
//	FLASH_Erase_Sector(MY_SectorNum, FLASH_BANK, FLASH_VOLTAGE_RANGE);
//	HAL_FLASH_Lock();
//}
//
////2. Set Sector Adress
//void MY_FLASH_SetSectorAddrs(uint8_t bank, uint8_t sector, uint32_t addrs)
//{
//	MY_Bank = bank;
//	MY_SectorNum = sector;
//	MY_SectorAddrs = addrs;
//}
//
////3. Write Flash
//void MY_FLASH_WriteN(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType, int FLASH_BANK, int FLASH_VOLTAGE_RANGE)
//{
//	uint32_t flashAddress = MY_SectorAddrs + idx;
//
//	//Erase sector before write
//	MY_FLASH_EraseSector(FLASH_BANK, FLASH_VOLTAGE_RANGE);
//
//	//Unlock Flash
//	HAL_FLASH_Unlock();
//	//Write to Flash
//	switch(dataType)
//	{
//		case DATA_TYPE_8:
//				for(uint32_t i=0; i<Nsize; i++)
//				{
//					HAL_FLASH_Program(FLASH_PSIZE_BYTE, flashAddress , ((uint8_t *)wrBuf)[i]);
//					flashAddress++;
//				}
//			break;
//
//		case DATA_TYPE_16:
//				for(uint32_t i=0; i<Nsize; i++)
//				{
//					HAL_FLASH_Program(FLASH_PSIZE_HALF_WORD, flashAddress , ((uint16_t *)wrBuf)[i]);
//					flashAddress+=2;
//				}
//			break;
//
//		case DATA_TYPE_32:
//				for(uint32_t i=0; i<Nsize; i++)
//				{
//					HAL_FLASH_Program(FLASH_PSIZE_WORD, flashAddress , ((uint32_t *)wrBuf)[i]);
//					flashAddress+=4;
//				}
//			break;
//	}
//	//Lock the Flash space
//	HAL_FLASH_Lock();
//}
////4. Read Flash
//void MY_FLASH_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType)
//{
//	uint32_t flashAddress = MY_SectorAddrs + idx;
//
//	switch(dataType)
//	{
//		case DATA_TYPE_8:
//				for(uint32_t i=0; i<Nsize; i++)
//				{
//					*((uint8_t *)rdBuf + i) = *(uint8_t *)flashAddress;
//					flashAddress++;
//				}
//			break;
//
//		case DATA_TYPE_16:
//				for(uint32_t i=0; i<Nsize; i++)
//				{
//					*((uint16_t *)rdBuf + i) = *(uint16_t *)flashAddress;
//					flashAddress+=2;
//				}
//			break;
//
//		case DATA_TYPE_32:
//				for(uint32_t i=0; i<Nsize; i++)
//				{
//					*((uint32_t *)rdBuf + i) = *(uint32_t *)flashAddress;
//					flashAddress+=4;
//				}
//			break;
//	}
//}
