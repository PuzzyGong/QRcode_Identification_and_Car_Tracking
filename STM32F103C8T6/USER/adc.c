#include "adc.h"

static uint16_t adc_buff[1];

static void gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init( GPIOA, &GPIO_InitStruct );
}

static void dma_init()
{
    DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );
	DMA_DeInit(DMA1_Channel1);
	
	DMA_InitStruct.DMA_PeripheralBaseAddr = ( uint32_t )( &ADC1->DR );
	DMA_InitStruct.DMA_MemoryBaseAddr = ( uint32_t )adc_buff;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	
	DMA_InitStruct.DMA_BufferSize = 1;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

static void adc__init()
{
    ADC_InitTypeDef ADC_InitStruct;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_Init(ADC1, &ADC_InitStruct);

	RCC_ADCCLKConfig( RCC_PCLK2_Div8 );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5 );
    //can add
	
}
void adc_init()
{
    gpio_init();
	dma_init();
	adc__init();
	ADC_DMACmd( ADC1, ENABLE );
    ADC_Cmd( ADC1, ENABLE );
	
	ADC_StartCalibration( ADC1 );
	while( ADC_GetCalibrationStatus( ADC1 ) );
    ADC_SoftwareStartConvCmd( ADC1, ENABLE );
}

void adc_getdata(uint16_t* u16_value)
{
    *u16_value = adc_buff[0];
}
