#include "Robstrite.h"
#include "string.h"
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -44.0f 
 #define V_MAX 44.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -17.0f
 #define T_MAX 17.0f

uint32_t Mailbox; // 定义邮箱变量

/*******************************************************************************
* @功能     		: RobStrite电机实例化的构造函数
* @参数         : CAN ID
* @返回值 			: void
* @概述  				: 初始化电机ID。
*******************************************************************************/
RobStrite_Motor::RobStrite_Motor(uint8_t CAN_Id)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0x1F;	
	Motor_Set_All.set_motor_mode = move_control_mode;
}
RobStrite_Motor::RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0x1F;	
	Motor_Set_All.set_motor_mode = move_control_mode;
	Motor_Offset_MotoFunc = Offset_MotoFunc;

}
/*******************************************************************************
* @功能     		: uint16_t型转float型浮点数
* @参数1        : 需要转换的值
* @参数2        : x的最小值
* @参数3        : x的最大值
* @参数4        : 需要转换的进制数
* @返回值 			: 十进制的float型浮点数
* @概述  				: None
*******************************************************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}
/*******************************************************************************
* @功能     		: float浮点数转int型
* @参数1        : 需要转换的值
* @参数2        : x的最小值
* @参数3        : x的最大值
* @参数4        : 需要转换的进制数
* @返回值 			: 十进制的int型整数
* @概述  				: None
*******************************************************************************/
int float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x = x_max;
	else if(x < x_min) x = x_min;
	return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}
/*******************************************************************************
* @功能     		: uint8_t数组转float浮点数
* @参数        	: 需要转换的数组
* @返回值 			: 十进制的float型浮点数
* @概述  				: None
*******************************************************************************/
float Byte_to_float(uint8_t* bytedata)  
{  
	uint32_t data = bytedata[7]<<24|bytedata[6]<<16|bytedata[5]<<8|bytedata[4];
	float data_float = *(float*)(&data);
  return data_float;  
}  
//int count_num = 0 ;
/*******************************************************************************
* @功能     		: 接收处理函数		（通信类型2 17应答帧 0应答帧）
* @参数1        : 接收到的数据
* @参数2        : 接收到的CANID
* @返回值 			: None
* @概述  				: drw只有通过通信17发送以后才有值
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId)
{
	if (uint8_t((ID_ExtId&0xFF00)>>8) == CAN_ID)
	{		
		//count_num++;
		if (int((ID_ExtId&0x3F000000)>>24) == 2)
		{
			Pos_Info.Angle =  uint16_to_float(DataFrame[0]<<8|DataFrame[1],P_MIN,P_MAX,16);
			Pos_Info.Speed =  uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
			Pos_Info.Torque = uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);				
			Pos_Info.Temp = (DataFrame[6]<<8|DataFrame[7])*0.1;
			error_code = uint8_t((ID_ExtId&0x3F0000)>>16);
			Pos_Info.pattern = uint8_t((ID_ExtId&0xC00000)>>22);
		}
		else if (int((ID_ExtId&0x3F000000)>>24) == 17)
		{
			for (int index_num = 0; index_num <= 13; index_num++)
			{
				if ((DataFrame[1]<<8|DataFrame[0]) == Index_List[index_num])
					switch(index_num)
					{
						case 0:
							drw.run_mode.data = uint8_t(DataFrame[4]);
							break;
						case 1:
							drw.iq_ref.data = Byte_to_float(DataFrame);
							break;
						case 2:
							drw.spd_ref.data = Byte_to_float(DataFrame);
							break;
						case 3:
							drw.imit_torque.data = Byte_to_float(DataFrame);
							break;
						case 4:
							drw.cur_kp.data = Byte_to_float(DataFrame);
							break;
						case 5:
							drw.cur_ki.data = Byte_to_float(DataFrame);
							break;
						case 6:
							drw.cur_filt_gain.data = Byte_to_float(DataFrame);
							break;
						case 7:
							drw.loc_ref.data = Byte_to_float(DataFrame);
							break;
						case 8:
							drw.limit_spd.data = Byte_to_float(DataFrame);
							break;
						case 9:
							drw.limit_cur.data = Byte_to_float(DataFrame);
							break;	
						case 10:
							drw.mechPos.data = Byte_to_float(DataFrame);
							break;	
						case 11:
							drw.iqf.data = Byte_to_float(DataFrame);
							break;	
						case 12:
							drw.mechVel.data =Byte_to_float(DataFrame);
							break;	
						case 13:
							drw.VBUS.data = Byte_to_float(DataFrame);
							break;	
					}	
			}
		}
		else if ((uint8_t)((ID_ExtId & 0xFF)) == 0xFE)
		{
			CAN_ID = uint8_t((ID_ExtId & 0xFF00)>>8);	
		}	
	}
}
/*******************************************************************************
* @功能     		: RobStrite电机获取设备ID和MCU（通信类型0）
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Get_CAN_ID()
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_Get_ID<<24|Master_CAN_ID <<8|CAN_ID;
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStrite电机运控模式  （通信类型1）
* @参数1        : 力矩（-4Nm~4Nm）
* @参数2        : 目标角度(-4π~4π)
* @参数3        : 目标角速度(-30rad/s~30rad/s)
* @参数4        : Kp(0.0~500.0)
* @参数5        : Kp(0.0~5.0)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd)
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage; 					//发送邮箱
	Motor_Set_All.set_Torque = Torque;
	Motor_Set_All.set_angle = Angle;	
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_Kp = Kp;
	Motor_Set_All.set_Kd = Kd;
	if (drw.run_mode.data != 0 && Pos_Info.pattern == 2)
	{
		Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);		//设置电机模式
		Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = move_control_mode;
	}
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotionControl<<24|float_to_uint(Motor_Set_All.set_Torque,T_MIN,T_MAX,16)<<8|CAN_ID;
	txdata[0] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16)>>8; 
	txdata[1] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16); 
	txdata[2] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16)>>8; 
	txdata[3] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16); 
	txdata[4] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16)>>8; 
	txdata[5] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16); 
	txdata[6] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16)>>8; 
	txdata[7] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16); 
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStrite电机位置模式 
* @参数1        : 目标角速度(-30rad/s~30rad/s)
* @参数2        : 目标角度(-4π~4π)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_Pos_control(float Speed, float acceleration, float Angle)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_angle = Angle;
	if (drw.run_mode.data != 1 && Pos_Info.pattern == 2)
	{
		Set_RobStrite_Motor_parameter(0X7005, Pos_control_mode, Set_mode);		//设置电机模式
		Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = Pos_control_mode;
	}
	Set_RobStrite_Motor_parameter(0X7017, Motor_Set_All.set_speed, Set_parameter);
	Set_RobStrite_Motor_parameter(0X7025, Motor_Set_All.set_acceleration, Set_parameter);	
	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
}
/*******************************************************************************
* @功能     		: RobStrite电机速度模式 
* @参数1        : 目标角速度(-30rad/s~30rad/s)
* @参数2        : 目标电流限制(0~23A)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
uint8_t count_set_motor_mode_Speed = 0;
void RobStrite_Motor::RobStrite_Motor_Speed_control(float Speed,float acceleration, float limit_cur)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_limit_cur = limit_cur;
//	if (Motor_Set_All.set_motor_mode != 2 && Pos_Info.pattern == 2)
//	{
//		Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);		//设置电机模式
//		//Get_RobStrite_Motor_parameter(0x7005);
	Motor_Set_All.set_motor_mode = Speed_control_mode;
//	}
//	if (count_set_motor_mode_Speed % 2 == 0)
	Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);		//设置电机模式
	count_set_motor_mode_Speed++;
	Set_RobStrite_Motor_parameter(0X7018, Motor_Set_All.set_limit_cur, Set_parameter);
  Set_RobStrite_Motor_parameter(0X7022, Motor_Set_All.set_acceleration, Set_parameter);	
	Set_RobStrite_Motor_parameter(0X700A, Motor_Set_All.set_speed, Set_parameter);
}
/*******************************************************************************
* @功能     		: RobStrite电机电流模式
* @参数         : 目标电流(-23~23A)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
uint8_t count_set_motor_mode = 0;
void RobStrite_Motor::RobStrite_Motor_current_control(float current)
{
	Motor_Set_All.set_current = current;
	output = Motor_Set_All.set_current;
	if (Pos_Info.pattern == 2 && Motor_Set_All.set_motor_mode != 3)
	{
		Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);		//设置电机模式
		//Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = Elect_control_mode;
	}
	if (count_set_motor_mode % 50 == 0)
		Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);		//设置电机模式
	count_set_motor_mode++;
	Set_RobStrite_Motor_parameter(0X7006, Motor_Set_All.set_current, Set_parameter);
}
/*******************************************************************************
* @功能     		: RobStrite电机零点模式
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_Set_Zero_control()
{
	Set_RobStrite_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);					//设置电机模式
}
/*******************************************************************************
* @功能     		: RobStrite电机使能 （通信类型3）
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::Enable_Motor()
{
	uint8_t txdata[8] = {0};				//发送数据
	CAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|CAN_ID;
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStrite电机失能 （通信类型4）
* @参数         : 是否清除错误位（0不清除 1清除）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::Disenable_Motor(uint8_t clear_error)
{
	uint8_t txdata[8] = {0};					   	//发送数据
	CAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	
	txdata[0] = clear_error;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|CAN_ID;
	
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStrite电机写入参数 （通信类型18）
* @参数1        : 参数地址
* @参数2        : 参数数值
* @参数3        : 选择是传入控制模式 还是其他参数 （Set_mode设置控制模式 Set_parameter设置参数）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = Index;
	txdata[1] = Index>>8;
	txdata[2] = 0x00;
	txdata[3] = 0x00;	
	if (Value_mode == 'p')
	{
		memcpy(&txdata[4],&Value,4);
	}
	else if (Value_mode == 'j')
	{
		Motor_Set_All.set_motor_mode = int(Value);
		txdata[4] = (uint8_t)Value;
		txdata[5] = 0x00;	
		txdata[6] = 0x00;	
		txdata[7] = 0x00;	
	}
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStrite电机单个参数读取 （通信类型17）
* @参数         : 参数地址
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::Get_RobStrite_Motor_parameter(uint16_t Index)
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	txdata[0] = Index;
	txdata[1] = Index>>8;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_GetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStrite电机设置CAN_ID （通信类型7）
* @参数         : 修改后（预设）CANID
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStrite_Motor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disenable_Motor(0);
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_Can_ID<<24|Set_CAN_ID<<16|Master_CAN_ID<<8|CAN_ID;
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStrite电机设置机械零点 （通信类型6）
* @参数         : None
* @返回值 			: void
* @概述  				: 会把当前电机位置设为机械零位， 会先失能电机, 再使能电机
*******************************************************************************/
void RobStrite_Motor::Set_ZeroPos()
{
	Disenable_Motor(0);							//失能电机
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = 1;
  HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailbox); // 发送CAN消息
	Enable_Motor();
}

/*******************************************************************************
* @功能     		: RobStrite电机数据的参数地址初始化
* @参数         : 数据的参数地址数组
* @返回值 			: void
* @概述  				: 会在创建电机类时自动调用
*******************************************************************************/
data_read_write::data_read_write(const uint16_t *index_list)
{
	run_mode.index = index_list[0];
	iq_ref.index = index_list[1];
	spd_ref.index = index_list[2];
	imit_torque.index = index_list[3];
	cur_kp.index = index_list[4];
	cur_ki.index = index_list[5];
	cur_filt_gain.index = index_list[6];
	loc_ref.index = index_list[7];
	limit_spd.index = index_list[8];
	limit_cur.index = index_list[9];
	mechPos.index = index_list[10];
	iqf.index = index_list[11];
	mechVel.index = index_list[12];
	VBUS.index = index_list[13];	
	rotation.index = index_list[14];
}
