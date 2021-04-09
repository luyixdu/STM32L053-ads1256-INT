#ifndef _ADS1256_H
#define _ADS1256_H
/****************************加载头文件区***********************/
#include "stm32l0xx_hal.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "math.h"
#include "stdlib.h"

    #define CS_0() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
	#define CS_1() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)

	#define DRDY_IS_LOW()	(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin) == 0)
	
/* 增益选项 */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* 增益1（缺省） */
	ADS1256_GAIN_2			= (1),	/* 增益2 */
	ADS1256_GAIN_4			= (2),	/* 增益4 */
	ADS1256_GAIN_8			= (3),	/* 增益8 */
	ADS1256_GAIN_16			= (4),	/* 增益16 */
	ADS1256_GAIN_32			= (5),	/* 增益32 */
	ADS1256_GAIN_64			= (6),	/* 增益64 */
}ADS1256_GAIN_E;

/* 采样速率选项 */
/* 数据转换率选择
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

/****************************申明函数区************************/
#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* 增益 */
	ADS1256_DRATE_E DataRate;	/* 数据输出速率 */
	int32_t AdcNow[8];			/* 8路ADC采集结果（实时）有符号数 */
	uint8_t Channel;			/* 当前通道 */
	uint8_t ScanMode;			/* 扫描模式，0表示单端8路， 1表示差分4路 */
}ADS1256_VAR_T;

extern ADS1256_VAR_T g_tADS1256;

void ads1256Init(void);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

uint8_t ADS1256_ReadChipID(void);
void ADS1256_StartScan(uint8_t _ucScanMode);
void ADS1256_StopScan(void);
int32_t ADS1256_GetAdc(uint8_t _ch);



void delayXus(uint16_t us);
void disableInterrupt(void);
void enableInterrupt(void);

float filterlowerpass(float adc);
float kalman_filter(float ADC_Value);

#endif
