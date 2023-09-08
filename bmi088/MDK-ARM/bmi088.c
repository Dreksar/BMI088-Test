#include "bmi088.h"
#include "main.h"
#include "math.h"
#include "MahonyAHRS.h"
extern SPI_HandleTypeDef hspi1;

struct _IMU_Vale IMU_Vale;

void BMI088_read_write_byte(uint8_t txdata,uint8_t *rxdata,uint8_t len)
{
	HAL_SPI_TransmitReceive(&hspi1,&txdata,rxdata,len,50);
}

void BMI088_read_acc_reg(uint8_t reg,uint8_t *return_data,uint8_t len)
{
	BMI088_ACC_CS_L();
	BMI088_read_write_byte(reg|0x80,return_data,1);
	BMI088_read_write_byte(reg|0x80,return_data,1);
	BMI088_read_write_byte(0xff,return_data,len);
	BMI088_ACC_CS_H();
}

void BMI088_read_gyro_reg(uint8_t reg,uint8_t *return_data,uint8_t len)
{
	BMI088_GYRO_CS_L();
	BMI088_read_write_byte(reg|0x80,return_data,1);
	BMI088_read_write_byte(0xff,return_data,len);
	BMI088_GYRO_CS_H();
}
void BMI088_write_acc_reg(uint8_t reg,uint8_t data)
{
	uint8_t rx;
	BMI088_ACC_CS_L();
	BMI088_read_write_byte(reg,&rx,1);
	BMI088_read_write_byte(data,&rx,1);
	BMI088_ACC_CS_H();
}

#define BMI088_WRITE_ACC_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t ids[]={0,0};

uint8_t BMI088_init()
{
	uint8_t IMU_State=BMI088_NO_ERROR;
	uint8_t reg=0;
	
	BMI088_read_acc_reg(BMI088_ACC_CHIP_ID,&ids[0],1);
	BMI088_read_gyro_reg(BMI088_GYRO_CHIP_ID,&ids[1],1);
	HAL_Delay(20);
	
	if(ids[0]!=BMI088_ACC_CHIP_ID_VALUE||ids[1]!=BMI088_GYRO_CHIP_ID_VALUE)
	{
		IMU_State=BMI088_NO_SENSOR;
		return IMU_State;
	}
	for(int write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
		{
			BMI088_write_acc_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
      HAL_Delay(100);

      BMI088_read_acc_reg(write_BMI088_accel_reg_data_error[write_reg_num][0],&reg,1);
      HAL_Delay(100);
			
			if (reg != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
					IMU_State=write_BMI088_accel_reg_data_error[write_reg_num][2];
					break;
				}

		}
		return IMU_State;
}

float BMI088_ACCEL_SEN=BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN=BMI088_GYRO_2000_SEN;
void BMI088_Read(float *acc,float *gyro)
{
	uint8_t regs[6]={0};
	int16_t temp_raw;
	BMI088_read_acc_reg(BMI088_ACCEL_XOUT_L,regs,6);
	
	temp_raw=(int16_t)((regs[1]<<8)|regs[0]);
	acc[0]=temp_raw*BMI088_ACCEL_SEN;
	temp_raw=(int16_t)((regs[3]<<8)|regs[2]);
	acc[1]=temp_raw*BMI088_ACCEL_SEN;
	temp_raw=(int16_t)((regs[5]<<8)|regs[4]);
	acc[2]=temp_raw*BMI088_ACCEL_SEN;
	
	BMI088_read_gyro_reg(BMI088_GYRO_X_L,regs,6);
	temp_raw=(int16_t)((regs[1]<<8)|regs[0]);
	gyro[0]=temp_raw*BMI088_GYRO_SEN;
	temp_raw=(int16_t)((regs[3]<<8)|regs[2]);
	gyro[1]=temp_raw*BMI088_GYRO_SEN;
	temp_raw=(int16_t)((regs[5]<<8)|regs[4]);
	gyro[2]=temp_raw*BMI088_GYRO_SEN;
	
	
}
	
void IMU_update()
{
	float acc[3]={0};
	float gyro[3]={0};
	BMI088_Read(acc,gyro);
	IMU_Vale.AccX=acc[0];
	IMU_Vale.AccY=acc[1];
	IMU_Vale.AccZ=acc[2];
	
	IMU_Vale.GyroX=gyro[0];
	IMU_Vale.GyroY=gyro[1];
	IMU_Vale.GyroZ=gyro[2];
}

float my_sqrt(float n)
{
	long i;
	float x,y;
	const float f=1.5F;
	x=n*0.5F;
	y=n;
	i=* (long *)&y;
	i=0x5f3759df-(i>>1);
	y=*(float *)&i;
	y=y*(f-(x*y*y));
	y=y*(f-(x*y*y));
	return n*y;
}


void AHRS(float gx,float gy,float gz,float ax,float ay,float az)
{
	#define halfT 0.001f
	float Kp=400.0f;
	float Ki=0.3f;
	static float q0=1,q1=0,q2=0,q3=0;
	static float exInt=0,eyInt=0,ezInt=0;
	float norm;
	float vx,vy,vz;
	float ex,ey,ez;
	float q0q0=q0*q0;
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
	norm=sqrt(ax*ax+ay*ay+az*az);
	ax=ax/norm;
	ay=ay/norm;
	az=az/norm;
	vx=2*(q1q3-q0q2);
	vx=2*(q0q1+q2q3);
	vx=q0q0-q1q1-q2q2+q3q3;
	
	ex=(ay*vz-az*vy);
	ey=(az*vx-ax*vz);
	ez=(ax*vy-ay*vx);
	exInt+=ex*Ki;
	eyInt+=ey*Ki;
	ezInt+=ez*Ki;
	gx+=Kp*ex+exInt;
	gy+=Kp*ey+eyInt;
	gz+=Kp*ez+ezInt;
	q0+=(-q1*gx-q2*gy-q3*gz)*halfT;
	q1+=(q0*gx+q2*gz-q3*gy)*halfT;
	q2+=(q0*gy-q1*gz+q3*gx)*halfT;
	q3+=(q0*gz+q1*gy-q2*gx)*halfT;
	norm=my_sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0/norm;
	q1=q1/norm;
	q2=q2/norm;
	q3=q3/norm;
	IMU_Vale.Pitch=asinf(2*(q0*q2-q1*q3))*57.2957795f;
	IMU_Vale.Roll=asinf(2*(q0*q1+q2*q3))*57.2957795f;
	IMU_Vale.Yaw=atan2(2*q1q2+2*q0q3,-2*q2q2-2*q3q3+1)*57.2957795f;
}

	void get_angle( float *yaw, float *pitch, float *roll)
{
*yaw = atan2f(2.0f*(q0*q3+q1*q2), 2.0f*(q0*q0+q1*q1)-1.0f)*57.2957795f;
*pitch = asinf(-2.0f*(q1*q3-q0*q2))*57.2957795f;
*roll = atan2f(2.0f*(q0*q1+q2*q3),2.0f*(q0*q0+q3*q3)-1.0f)*57.2957795f;
}
void IMU_AHRS()
{
	IMU_update();
	//AHRS(IMU_Vale.GyroX,IMU_Vale.GyroY,IMU_Vale.GyroZ,IMU_Vale.AccX,IMU_Vale.AccY, IMU_Vale.AccZ);
	MahonyAHRSupdateIMU(IMU_Vale.GyroX,IMU_Vale.GyroY,IMU_Vale.GyroZ,IMU_Vale.AccX,IMU_Vale.AccY, IMU_Vale.AccZ);
	get_angle(&IMU_Vale.Yaw,&IMU_Vale.Pitch,&IMU_Vale.Roll);
}


