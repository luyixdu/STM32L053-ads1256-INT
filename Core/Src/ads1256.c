#include "ads1256.h"
int32_t adcVaule[8] = {0x00};
float voltage[8] = {0.00};
float filterVoltage[8] = {0.00};
float filterVoltage2[8] = {0.00};

uint8_t exti_flag = 0;

void delayXus(uint16_t us) {
    uint16_t diff = 0xffff - 5 - us;
    //设置定时器的计数值
    __HAL_TIM_SET_COUNTER(&htim6, diff);
    //启动定时器计数
    HAL_TIM_Base_Start(&htim6);
    //判定计数结束
    while(diff < 0xffff - 5) {
        diff = __HAL_TIM_GET_COUNTER(&htim6);
    }
    //延时完成关闭定时器计数
    HAL_TIM_Base_Stop(&htim6);
}

/*
*   功  能:实现SPI协议总线发送一个字节的数据信息
*   参  数:待发送的数据信息
*   返回值:无
*/
void spiWriteByte(uint8_t txData) {
    uint8_t tempData = 0x00;
    HAL_SPI_TransmitReceive(&hspi1, &txData, &tempData, 1, 100);
}

/*
*   功  能:实现SPI协议总监接受一个字节的数据信息
*   参  数:无
*   返回值:接受到的数据信息
*/
uint8_t spiReadByte(void) {
    uint8_t tempDataT = 0xff;
    uint8_t tempData = 0x00;
    HAL_SPI_TransmitReceive(&hspi1, &tempDataT, &tempData, 1, 100);
    return tempData;
}

/*
*   功  能:向ads1256寄存器中写入一个字节的数据
*   参  数:regAdd寄存器地址 regData待写入的数据信息
*   返回值:无
*/
void spiWriteRegData(uint8_t regAdd, uint8_t regData) {
    //拉低SPI协议的CS引脚
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    //等待RDY的引脚变低
    while(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin));
    //写入寄存地地址
    spiWriteByte(WREG | (regAdd & 0x0F));
    //写入即将写入数据的个数
    spiWriteByte(0x00);
    //写入数据信息
    spiWriteByte(regData);
    //拉高SPI协议的CS引脚
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/*
*   功  能:初始化ads1256
*   参  数:无
*   返回值:无
*/
void ads1256Init(void) {
    disableInterrupt();
	DBG("init\r\n");
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    while(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin));
    //开启芯片的自校准
	DBG("init\r\n");
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    spiWriteByte(SELFCAL);
	DBG("init\r\n");
    while(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin));
	DBG("init\r\n");
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    //设置ads1256的状态寄存器
    spiWriteRegData(STATUS, 0x06);      //数据发送高位在前 自动校准 启用buf
    spiWriteRegData(MUX, MUXP_AIN0 | MUXN_AIN1); //单端模式
    //设置ads1256的增益
    spiWriteRegData(ADCON, GAIN_1);
    //设置ads采样速率
    spiWriteRegData(DRATE, RATE_30000);
    //设置IO状态
    spiWriteRegData(IO, 0x00);
    //再次进行校准
    while(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin));
    //开启芯片的自校准
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    spiWriteByte(SELFCAL);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	DBG("init\r\n");
    enableInterrupt();
}

/*
*   功  能:从ads1256中读取出相关数据信息，单端
*   参  数:下一次转换通道
*   返回值:读取到的数据信息
*/
int32_t ads1256ReadValue(uint8_t channel) {
    int32_t sum[8] = {0.00};
    //等待准备好信号变低
    while(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin));
    //设置下次转换的通道
    spiWriteRegData(MUX, channel | MUXN_AINCOM);//单端

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    spiWriteByte(SYNC);     //发送同步命令
    delayXus(5);
    spiWriteByte(WAKEUP);   //发送唤醒命令
    delayXus(5);            //延时一下
    spiWriteByte(RDATA);    //发送读数据命令
    delayXus(25);
    sum[channel] |= (spiReadByte() << 16);
    sum[channel] |= (spiReadByte() << 8);
    sum[channel] |= (spiReadByte());
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    if(sum[channel] > 0x7fffff)
        sum[channel] -= 0x1000000;
    adcVaule[channel] = sum[channel];
    voltage[channel] = (float)(adcVaule[channel] * 5.0 / 8388607); //计算电压
    DBG("\r\n\t通道%dADC采集数据:%0x,\t电压值%f\n", channel, adcVaule[channel], voltage[channel]);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    return sum[channel];
}
/*
*   功  能:实现ads的增益设置
*/
void setGain(uint8_t gain) {
    disableInterrupt();
    while(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin));
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    switch(gain) {
    case 0:
        spiWriteRegData(ADCON, GAIN_1);
        break;
    case 1:
        spiWriteRegData(ADCON, GAIN_2);
        break;
    default:
        break;
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    enableInterrupt();
}
/*
*   功  能:设置ads1256的采集速率
*/
void setRate(uint8_t rate) {
    disableInterrupt();
    while(HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin));
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    switch(rate) {
    case 0:
        spiWriteRegData(DRATE, RATE_2_5);
        break;
    case 1:
        spiWriteRegData(DRATE, RATE_10);
        break;
    default:
        break;
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    enableInterrupt();
}
/*
*   功  能:实现屏蔽所有中断函数
*/
void disableInterrupt(void) {
    __set_PRIMASK(1);
}

/*
*   功  能:开启全局中断
*/
void enableInterrupt(void) {
    __set_PRIMASK(0);
}
/*
*   功  能:实现外部中断回调函数
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    int32_t sum = 0x00;
    disableInterrupt();
//	DBG("TEST");
    if((GPIO_Pin == DRDY_INT_Pin) && (HAL_GPIO_ReadPin(DRDY_INT_GPIO_Port, DRDY_INT_Pin) == GPIO_PIN_RESET)) {
        //设置下次转换的通道
	    exti_flag = 1;
/*
        spiWriteRegData(MUX, MUXP_AIN0 | MUXN_AINCOM);
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
        spiWriteByte(SYNC);     //发送同步命令
        delayXus(5);
        spiWriteByte(WAKEUP);   //发送唤醒命令
        delayXus(5);            //延时一下
        spiWriteByte(RDATA);    //发送读数据命令
        delayXus(25);
        sum |= (spiReadByte() << 16);
        sum |= (spiReadByte() << 8);
        sum |= (spiReadByte());
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
        if(sum > 0x7fffff)
            sum -= 0x1000000;
        adcVaule = sum;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
*/
    }
    enableInterrupt();
}

/*
float filterlowerpass(float adc) {
    //y(n) = qX(n)+(1-q)Y(n-1)
    float filterVaule = 0;
    static float adcold = 0;
    filterVaule = 0.5 * adc + 0.5 * adcold;
    adcold = adc;
    return filterVaule;
}

float kalman_filter(float ADC_Value) {
    float x_k1_k1, x_k_k1;
    static float ADC_OLD_Value;
    float Z_k;
    static float P_k1_k1;

    static float Q = 0.0001;
    static float R = 5;
    static float Kg = 0;
    static float P_k_k1 = 1;

    float kalman_adc;
    static float kalman_adc_old = 0;
    Z_k = ADC_Value;

    if (abs(kalman_adc_old - ADC_Value) >= 10) {
        x_k1_k1 = ADC_Value * 0.382 + kalman_adc_old * 0.618;
    } else {
        x_k1_k1 = kalman_adc_old;
    }
    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1 / (P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg) * P_k_k1;
    P_k_k1 = P_k1_k1;

    ADC_OLD_Value = ADC_Value;
    kalman_adc_old = kalman_adc;

    return kalman_adc;
}
*/
