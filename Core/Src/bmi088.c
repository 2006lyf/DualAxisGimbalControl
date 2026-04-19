/* bmi088.c */
#include "bmi088.h"
#include "spi.h"

static SPI_HandleTypeDef *hspi = &hspi1;

/* SPI读写函数 */
static uint8_t SPI_ReadWriteByte(uint8_t tx_data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(hspi, &tx_data, &rx_data, 1, 100);
    return rx_data;
}

/* 读取加速度计寄存器 */
static uint8_t Accel_ReadReg(uint8_t reg)
{
    uint8_t tx_data[2] = {reg | 0x80, 0x00};  // 读操作，最高位为1
    uint8_t rx_data[2];
    
    ACCEL_CS_LOW();
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, 100);
    ACCEL_CS_HIGH();
    
    return rx_data[1];
}

/* 写入加速度计寄存器 */
static void Accel_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg & 0x7F, data};  // 写操作，最高位为0
    
    ACCEL_CS_LOW();
    HAL_SPI_Transmit(hspi, tx_data, 2, 100);
    ACCEL_CS_HIGH();
}

/* 读取陀螺仪寄存器 */
static uint8_t Gyro_ReadReg(uint8_t reg)
{
    uint8_t tx_data[2] = {reg | 0x80, 0x00};
    uint8_t rx_data[2];
    
    GYRO_CS_LOW();
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, 100);
    GYRO_CS_HIGH();
    
    return rx_data[1];
}

/* 写入陀螺仪寄存器 */
static void Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg & 0x7F, data};
    
    GYRO_CS_LOW();
    HAL_SPI_Transmit(hspi, tx_data, 2, 100);
    GYRO_CS_HIGH();
}

/* BMI088 初始化 */
uint8_t BMI088_Init(void)
{
    uint8_t accel_id, gyro_id;
    
    /* 读取芯片ID验证通信 */
    accel_id = Accel_ReadReg(BMI088_ACC_CHIP_ID);
    gyro_id = Gyro_ReadReg(BMI088_GYRO_CHIP_ID);
    
    /* 加速度计ID应为0x1E，陀螺仪ID应为0x0F */
    if(accel_id != 0x1E || gyro_id != 0x0F) {
        return 0;  // 初始化失败
    }
    
    /* 配置加速度计 */
    Accel_WriteReg(BMI088_ACC_PWR_CONF, 0x00);     // 正常模式
    Accel_WriteReg(BMI088_ACC_PWR_CTRL, 0x04);     // 使能
    HAL_Delay(5);
    
    Accel_WriteReg(BMI088_ACC_CONF, 0xA0);         // 1600Hz 带宽
    Accel_WriteReg(BMI088_ACC_RANGE, 0x01);        // ±24G 量程
    
    /* 配置陀螺仪 */
    Gyro_WriteReg(BMI088_GYRO_RANGE, 0x00);        // ±2000°/s
    Gyro_WriteReg(BMI088_GYRO_BANDWIDTH, 0x80);     // 2000Hz 带宽
    Gyro_WriteReg(BMI088_GYRO_LPM1, 0x00);          // 正常模式
    
    HAL_Delay(50);
    
    return 1;  // 初始化成功
}

/* 读取所有数据 */
void BMI088_Read(imu_data_t *data)
{
    uint8_t rx_buf[12];
    int16_t temp_raw;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    
    /* 读取加速度计数据 (6字节) */
    uint8_t accel_cmd = BMI088_ACC_DATA | 0x80;
    ACCEL_CS_LOW();
    HAL_SPI_Transmit(hspi, &accel_cmd, 1, 100);
    HAL_SPI_Receive(hspi, rx_buf, 6, 100);
    ACCEL_CS_HIGH();
    
    accel_raw[0] = (int16_t)(rx_buf[1] << 8 | rx_buf[0]);
    accel_raw[1] = (int16_t)(rx_buf[3] << 8 | rx_buf[2]);
    accel_raw[2] = (int16_t)(rx_buf[5] << 8 | rx_buf[4]);
    
    /* 读取陀螺仪数据 (6字节) */
    uint8_t gyro_cmd = BMI088_GYRO_DATA | 0x80;
    GYRO_CS_LOW();
    HAL_SPI_Transmit(hspi, &gyro_cmd, 1, 100);
    HAL_SPI_Receive(hspi, rx_buf, 6, 100);
    GYRO_CS_HIGH();
    
    gyro_raw[0] = (int16_t)(rx_buf[1] << 8 | rx_buf[0]);
    gyro_raw[1] = (int16_t)(rx_buf[3] << 8 | rx_buf[2]);
    gyro_raw[2] = (int16_t)(rx_buf[5] << 8 | rx_buf[4]);
    
    /* 单位转换 */
    // 加速度: ±24G 对应 32768 -> 24/32768 = 0.000732 G/LSB
    data->accel[0] = accel_raw[0] * 0.000732f;
    data->accel[1] = accel_raw[1] * 0.000732f;
    data->accel[2] = accel_raw[2] * 0.000732f;
    
    // 陀螺仪: ±2000°/s 对应 32768 -> 2000/32768 = 0.061 °/s/LSB
    data->gyro[0] = gyro_raw[0] * 0.061f;  // yaw
    data->gyro[1] = gyro_raw[1] * 0.061f;  // pitch
    data->gyro[2] = gyro_raw[2] * 0.061f;  // roll
    
    /* 读取温度 (可选) */
    data->temp = 23.0f;  // 默认值
}

/* 只读陀螺仪 */
void BMI088_ReadGyro(float gyro[3])
{
    uint8_t rx_buf[6];
    int16_t gyro_raw[3];
    
    uint8_t cmd = BMI088_GYRO_DATA | 0x80;
    GYRO_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, 100);
    HAL_SPI_Receive(hspi, rx_buf, 6, 100);
    GYRO_CS_HIGH();
    
    gyro_raw[0] = (int16_t)(rx_buf[1] << 8 | rx_buf[0]);
    gyro_raw[1] = (int16_t)(rx_buf[3] << 8 | rx_buf[2]);
    gyro_raw[2] = (int16_t)(rx_buf[5] << 8 | rx_buf[4]);
    
    gyro[0] = gyro_raw[0] * 0.061f;
    gyro[1] = gyro_raw[1] * 0.061f;
    gyro[2] = gyro_raw[2] * 0.061f;
}