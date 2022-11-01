
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_i2c.h"
#include "VL6180.h"

/**
  * ��������: I2C����λ�ӳ٣����400KHz
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
static void I2C_Delay(void)
{
	uint8_t i;

	/*��
	 	�����ʱ����ͨ���߼������ǲ��Եõ��ġ�
		CPU��Ƶ72MHzʱ�����ڲ�Flash����, MDK���̲��Ż�
		ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
		ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
	 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
        
    IAR���̱���Ч�ʸߣ���������Ϊ7
	*/
	for (i = 0; i < 29; i++);
}

/**
  * ��������: CPU����I2C���������ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
	I2C_SDA_HIGH();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_LOW();
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
}

/**
  * ��������: CPU����I2C����ֹͣ�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_LOW();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_HIGH();
}

/**
  * ��������: CPU��I2C�����豸����8bit����
  * �������: Byte �� �ȴ����͵��ֽ�
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
	for (i = 0; i < 8; i++)
	{		
		if (Byte & 0x80)//提取当前字节最高有效位
		{
			I2C_SDA_HIGH();
		}
		else
		{
			I2C_SDA_LOW();
		}
		I2C_Delay();
		I2C_SCL_HIGH();
		I2C_Delay();	
		I2C_SCL_LOW();//仅能在SCL为低电平时改变SDA的电平
		if (i == 7)
		{
			I2C_SDA_HIGH(); //
		}
		Byte <<= 1;//左移一位，以读取下一位
		I2C_Delay();
	}
}


/**
  * ��������: CPU��I2C�����豸��ȡ8bit����
  * �������: ��
  * �� �� ֵ: ����������
  * ˵    ������
  */
uint8_t I2C_ReadByte(uint8_t ack)
{
	uint8_t i;
	uint8_t value;//读取值

	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_HIGH();
		I2C_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_LOW();//仅能在SCL为低电平时改变SDA的电平
		I2C_Delay();
	}
  if(ack==0)
		I2C_NAck();
	else
		I2C_Ack();
	return value;
}

/**
  * ��������: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
  * �������: ��
  * �� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
  * ˵    ������
  */
uint8_t I2C_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_HIGH();	/* CPU�ͷ�SDA���� */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	I2C_Delay();
	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_LOW();
	I2C_Delay();
	return re;
}

/**
  * ��������: CPU����һ��ACK�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Ack(void)
{
	I2C_SDA_LOW();	/* CPU����SDA = 0 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����1��ʱ�� */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
	I2C_SDA_HIGH();	/* CPU�ͷ�SDA���� */
}

/**
  * ��������: CPU����1��NACK�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_NAck(void)
{
	I2C_SDA_HIGH();	/* CPU����SDA = 1 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����1��ʱ�� */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();	
}

/**
  * ��������: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��GPIOʱ�� */
	//I2C_GPIO_CLK_ENABLE();

  //GPIO_InitStruct.Pin = I2C_SCL_PIN|I2C_SDA_PIN;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  //HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

  /* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
  I2C_Stop();
}

/**
  * ��������: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
  * �������: _Address���豸��I2C���ߵ�ַ
  * �� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
  * ˵    �����ڷ���I2C�豸ǰ�����ȵ��� I2C_CheckDevice() ���I2C�豸�Ƿ��������ú���������GPIO
  */
uint8_t I2C_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	//I2C_InitGPIO();		/* ����GPIO */	
	I2C_Start();		/* ���������ź� */
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	I2C_SendByte(_Address | I2C_WR);
	ucAck = I2C_WaitAck();	/* ����豸��ACKӦ�� */
	I2C_Stop();			/* ����ֹͣ�ź� */
	return ucAck;
}




uint8_t VL6180X_WriteByte(uint16_t reg,uint8_t data)
{
	uint8_t Index_H = (uint8_t)(reg >> 8); //16位寄存器高8位
	uint8_t Index_L = (uint8_t)(reg & 0xFF);//16位寄存器低8位
	
	I2C_Start();
	I2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0);//发送7位设备地址及1位写方向位，实现写功能
	if(I2C_WaitAck())
	{
		I2C_Stop();	
		return 1;		
	}
	I2C_SendByte(Index_H);//发送寄存器高8位
	I2C_WaitAck();
	I2C_SendByte(Index_L);//发送寄存器低8位
	I2C_WaitAck();
	I2C_SendByte(data);//发送数据
	if(I2C_WaitAck())
	{
		I2C_Stop();	 
		return 1;		 
	}
	I2C_Stop();
	return 0;	
}

//VL6180X��ȡ8λ����
uint8_t VL6180X_ReadByte(uint16_t reg)
{
	uint8_t res;
	uint8_t Index_H = (uint8_t)(reg >> 8);
	uint8_t Index_L = (uint8_t)(reg & 0xff);

	/*发送要读取的设备及寄存器的地址*/
    I2C_Start(); 
	I2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0);//发送7位设备地址及1位写方向位，实现写功能
	I2C_WaitAck();
    I2C_SendByte(Index_H);	//д�Ĵ�����ַ
    I2C_WaitAck();		//�ȴ�Ӧ��
	I2C_SendByte(Index_L);	//д�Ĵ�����ַ
	I2C_WaitAck();	
	
	/*发送要读取的设备的地址，并将总线方向调整为“读”*/
    I2C_Start();
	I2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|1);//发送7位设备地址及1位读方向位，实现读功能
    I2C_WaitAck();		//�ȴ�Ӧ�� 
	res=I2C_ReadByte(0);//��ȡ����,����nACK 
    I2C_Stop();			//����һ��ֹͣ���� 
	return res;
}




//��ȡ16λ����
//VL6180X��ȡ16λ����
uint8_t VL6180X_Read_HalfWold(uint16_t reg)
{
	uint16_t res;
	uint8_t Index_H = (uint8_t)(reg >> 8);
	uint8_t Index_L = (uint8_t)(reg & 0xff);
    I2C_Start(); 
	I2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0);//����������ַ+д����	
	I2C_WaitAck();		//�ȴ�Ӧ�� 
    I2C_SendByte(Index_H);	//д�Ĵ�����ַ
    I2C_WaitAck();		//�ȴ�Ӧ��
	I2C_SendByte(Index_L);	//д�Ĵ�����ַ
	I2C_WaitAck();	
	
    I2C_Start();
	I2C_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|1);//����������ַ+������	
    I2C_WaitAck();		//�ȴ�Ӧ�� 
	res = I2C_ReadByte(1);//��ȡ����,����ACK 
	res <<= 8;
	res |= I2C_ReadByte(0);//��ȡ����,����nACK 
    I2C_Stop();			//����һ��ֹͣ���� 
	return res;
}

