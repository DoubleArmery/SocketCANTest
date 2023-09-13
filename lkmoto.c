// LK 电机驱动文件
// by xingyi wei
#include "lkmoto.h"
#include <stdio.h>
// data 长度 DLC
#define LEN 8 
// #define hcan1 hcan11
//can 总线 send 函数 移植仅需修改此函数
// id 0x140 + (1~32)
// CAN_TxHeaderTypeDef canTxHeader;
// CAN_RxHeaderTypeDef canRxHeader;
uint8_t canTxData[8];
uint8_t canRxData[8];



// 电机属性结构体
// 16位属性 pid 加速度 电机id 速度 单圈角度 多圈角度 编码器值 温度 电压 电流
struct MotoProperty
{
    // uint16_t pi[3];
    //创建pi 结构体
    // 位置环 速度环  转矩环
    struct PI
    {
        uint8_t p;
        uint8_t i;
        uint8_t d;
    } pi[3];

    uint32_t acc;
    uint8_t id;
    uint16_t speed;
    uint16_t angle;     //单圈角度
    uint32_t angle2_0; //多圈角度低字节
    uint32_t angle2_1; //多圈角度高字节
    uint16_t encoder_i;//编码器原始值
    uint16_t encoder;//编码器值
    uint16_t encoder_zero;//编码器零点
    uint8_t temp;
    uint16_t voltage;
    uint16_t current; //电流
    uint8_t status; //错误状态
};

// 电机属性结构体数组
struct MotoProperty motor[4];







void can_send(uint8_t *buf,uint8_t id)
{
    struct can_frame tx_frame;
    tx_frame.can_id = 0x140 + id;
    tx_frame.can_dlc = 8;
    for (int i = 0; i < 8; i++)
    {
        tx_frame.data[i] = buf[i];
    }
    can0_tx(&tx_frame);
}

//消息处理函数 名称 命令数据
// 读取 PID 参数命令 0x30
// 写入 PID 参数到 RAM 命令 0x31
// 写入 PID 参数到 ROM 命令 0x32
// 读取加速度命令 0x33
// 写入加速度到 RAM 命令 0x34
// 读取编码器命令 0x90
// 写入编码器值到 ROM 作为电机零点命令 0x91
// 写入当前位置到 ROM 作为电机零点命令 0x19
// 读取多圈角度命令 0x92
// 读取单圈角度命令 0x94
// 清除电机角度命令（设置电机初始位置） 0x95
// 读取电机状态 1 和错误标志命令 0x9A
// 清除电机错误标志命令 0x9B
// 读取电机状态 2 命令 0x9C
// 读取电机状态 3 命令 0x9D
// 电机关闭命令 0x80
// 电机停止命令 0x81
// 电机运行命令 0x88
// 转矩开环控制命令 0xA0
// 转矩闭环控制命令 0xA1
// 速度闭环控制命令 0xA2
// 位置闭环控制命令 1 0xA3
// 位置闭环控制命令 2 0xA4
// 位置闭环控制命令 3 0xA5
// 位置闭环控制命令 4 0xA6
// 位置闭环控制命令 5 0xA7
// 位置闭环控制命令 6 0xA8
// void can_msg_process(uint8_t id,uint8_t *buf)   // 见 receiveprocess 
// {  
//     switch (buf[0])
//     {
//     case 0x30:
//         /* code */
//         break;
    
//     default:
//         break;
//     }
// }





//只需要移植上述两个函数即可

// 发送
//  读取 PID 参数命令 0x30
// 主机发送该命令读取当前电机的的 PID 参数
// 数据域 说明 数据
// DATA[0] 命令字节 0x30
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

//回复 
// 驱动回复数据中包含了各个控制环路的 PI 参数。
// 数据域 说明 数据
// DATA[0] 命令字节 0x30
// DATA[1] NULL 0x00
// DATA[2] 位置环 P 参数 DATA[2] = anglePidKp
// DATA[3] 位置环 I 参数 DATA[3] = anglePidKi
// DATA[4] 速度环 P 参数 DATA[4] = speedPidKp
// DATA[5] 速度环 I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩环 P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩环 I 参数 DATA[7] = iqPidKi

void read_pid(uint8_t id){
    uint8_t buf[LEN] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}

//  写入 PID 参数到 RAM 命令 0x31
// DATA[0] 命令字节 0x31
// DATA[1] NULL 0x00
// DATA[2] 位置环 P 参数 DATA[2] = anglePidKp
// DATA[3] 位置环 I 参数 DATA[3] = anglePidKi
// DATA[4] 速度环 P 参数 DATA[4] = speedPidKp
// DATA[5] 速度环 I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩环 P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩环 I 参数 DATA[7] = iqPidKi

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_pid(uint8_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi){
    uint8_t buf[LEN] = {0x31, 0x00, anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi};
    can_send(buf, id);
}

//  写入 PID 参数到 ROM 命令 0x32
// DATA[0] 命令字节 0x32
// DATA[1] NULL 0x00
// DATA[2] 位置环 P 参数 DATA[2] = anglePidKp
// DATA[3] 位置环 I 参数 DATA[3] = anglePidKi
// DATA[4] 速度环 P 参数 DATA[4] = speedPidKp
// DATA[5] 速度环 I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩环 P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩环 I 参数 DATA[7] = iqPidKi

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致
void write_pid_to_rom(uint8_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi){
    uint8_t buf[LEN] = {0x32, 0x00, anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi};
    can_send(buf, id);
}





//  读取加速度命令 0x33
// 数据域 说明 数据
// DATA[0] 命令字节 0x33
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复数据中包含了加速度参数。加速度数据 Accel 为 int32_t 类型，单位 1dps/s
// 数据域 说明 数据
// DATA[0] 命令字节 0x33
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 加速度低字节 1 DATA[4] = *(uint8_t *)(&Accel)
// DATA[5] 加速度字节 2 DATA[5] = *((uint8_t *)(&Accel)+1)
// DATA[6] 加速度字节 3 DATA[6] = *((uint8_t *)(&Accel)+2)
// DATA[7] 加速度字节 4 DATA[7] = *((uint8_t *)(&Accel)+3)

void read_acc(uint8_t id){
    uint8_t buf[LEN] = {0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}




//  写入加速度到 RAM 命令 0x34
// DATA[0] 命令字节 0x34
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 加速度低字节 1 DATA[4] = *(uint8_t *)(&Accel)
// DATA[5] 加速度字节 2 DATA[5] = *((uint8_t *)(&Accel)+1)
// DATA[6] 加速度字节 3 DATA[6] = *((uint8_t *)(&Accel)+2)
// DATA[7] 加速度字节 4 DATA[7] = *((uint8_t *)(&Accel)+3)

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_acc(uint8_t id, float Accel){
    uint8_t buf[LEN] = {0x34, 0x00, 0x00, 0x00, *(uint8_t *)(&Accel), *((uint8_t *)(&Accel)+1), *((uint8_t *)(&Accel)+2), *((uint8_t *)(&Accel)+3)};
    can_send(buf, id);
}




//  读取编码器命令 0x90
// 数据域 说明 数据
// DATA[0] 命令字节 0x90
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 编码器位置 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383），为编码器原始位置减
// 去编码器零偏后的值。
// 2. 编码器原始位置 encoderRaw（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 3. 编码器零偏 encoderOffset（uint16_t 类型，14bit 编码器的数值范围 0~16383），该点作为电机角
// 度的 0 点。
// 数据域 说明 数据
// DATA[0] 命令字节 0x90
// DATA[1] NULL 0x00
// DATA[2] 编码器位置低字节 DATA[2] = *(uint8_t *)(&encoder)
// DATA[3] 编码器位置高字节 DATA[3] = *((uint8_t *)(&encoder)+1)
// DATA[4] 编码器原始位置低字节 DATA[4] = *(uint8_t *)(&encoderRaw)
// DATA[5] 编码器原始位置高字节 DATA[5] = *((uint8_t *)(&encoderRaw)+1)
// DATA[6] 编码器零偏低字节 DATA[6] = *(uint8_t *)(&encoderOffset)
// DATA[7] 编码器零偏高字节 DATA[7] = *((uint8_t *)(&encoderOffset)+1)

void read_encoder(uint8_t id){
    uint8_t buf[LEN] = {0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}






//  写入编码器值到 ROM 作为电机零点命令 0x91
// 主机发送该命令以设置编码器的零偏，其中，需要写入的编码器值 encoderOffset 为 uint16_t 类型，
// 14bit 编码器的数值范围 0~16383。
// 数据域 说明 数据
// DATA[0] 命令字节 0x91
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] 编码器零偏低字节 DATA[6] = *(uint8_t *)(&encoderOffset)
// DATA[7] 编码器零偏高字节 DATA[7] = *((uint8_t *)(&encoderOffset)+1)

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_encoder_offset(uint8_t id, uint16_t encoderOffset){
    uint8_t buf[LEN] = {0x91, 0x00, 0x00, 0x00, 0x00, 0x00, *(uint8_t *)(&encoderOffset), *((uint8_t *)(&encoderOffset)+1)};
    can_send(buf, id);
}



//  写入当前位置到 ROM 作为电机零点命令 0x19
// 数据域 说明 数据
// DATA[0] 命令字节 0x19
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_current_position_to_rom(uint8_t id){
    uint8_t buf[LEN] = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}



//  读取多圈角度命令 0x92
// 主机发送该命令以读取当前电机的多圈绝对角度值
// 数据域 说明 数据
// DATA[0] 命令字节 0x92
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机角度 motorAngle，为 int64_t 类型数据，正值表示顺时针累计角度，负值表示逆时针累计角
// 度，单位 0.01°/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0x92
// DATA[1] 角度低字节 1 DATA[1] = *(uint8_t *)(&motorAngle)
// DATA[2] 角度字节 2 DATA[2] = *((uint8_t *)(& motorAngle)+1)
// DATA[3] 角度字节 3 DATA[3] = *((uint8_t *)(& motorAngle)+2)
// DATA[4] 角度字节 4 DATA[4] = *((uint8_t *)(& motorAngle)+3)
// DATA[5] 角度字节 5 DATA[5] = *((uint8_t *)(& motorAngle)+4)
// DATA[6] 角度字节 6 DATA[6] = *((uint8_t *)(& motorAngle)+5)
// DATA[7] 角度字节 7 DATA[7] = *((uint8_t *)(& motorAngle)+6)



void read_angle(uint8_t id){
    uint8_t buf[LEN] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}



//  读取单圈角度命令 0x94
// 数据域 说明 数据
// DATA[0] 命令字节 0x94
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机单圈角度 circleAngle，为 uint16_t 类型数据，以编码器零点为起始点，顺时针增加，再次到
// 达零点时数值回 0，单位 0.01°/LSB，数值范围 0~35999。
// 数据域 说明 数据
// DATA[0] 命令字节 0x94
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] 单圈角度低字节 DATA[6] = *(uint8_t *)(& circleAngle)
// DATA[7] 单圈角度高字节 DATA[7] = *((uint8_t *)(& circleAngle)+1)

void read_angle_single(uint8_t id){
    uint8_t buf[LEN] = {0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}



//  清除电机角度命令（设置电机初始位置） 0x95 厂家暂未实现
// 回复 电机在收到命令后回复主机，回复命令和接收命令一致


//  读取电机状态 1 和错误标志命令 0x9A
// 该命令读取当前电机的温度、电压和错误状态标志
// 数据域 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下参数：
// 1. 电机温度 temperature（int8_t 类型，单位 1℃/LSB）。
// 2. 电压 voltage（uint16_t 类型，单位 0.1V/LSB）。
// 3. 错误标志 errorState（为 uint8_t 类型，各个位代表不同的电机状态）
// 数据域 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] NULL 0x00
// DATA[3] 电压低字节 DATA[3] = *(uint8_t *)(&voltage)
// DATA[4] 电压高字节 DATA[4] = *((uint8_t *)(& voltage)+1)
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] 错误状态字节 DATA[7]=errorState
// 备注：
// 1. errorState 各个位具体状态表如下
// errorState 位 状态说明 0 1
// 0 电压状态 电压正常 低压保护
// 1 无效
// 2 无效
// 3 温度状态 温度正常 过温保护
// 4 无效
// 5 无效
// 6 无效
// 7 无效


void read_status(uint8_t id){
    uint8_t buf[LEN] = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}

//  清除电机错误标志命令 0x9B

// 数据域 说明 数据
// DATA[0] 命令字节 0x9B
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

//驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下参数：
// 1. 电机温度 temperature（int8_t 类型，单位 1℃/LSB）。
// 2. 电压 voltage（uint16_t 类型，单位 0.1V/LSB）。
// 3. 错误标志 errorState（为 uint8_t 类型，各个位代表不同的电机状态）。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] NULL 0x00
// DATA[3] 电压低字节 DATA[3] = *(uint8_t *)(&voltage)
// DATA[4] 电压高字节 DATA[4] = *((uint8_t *)(& voltage)+1)
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] 错误状态字节 DATA[7]=errorState
// 备注：
// 1. 电机状态没有恢复正常时，错误标志无法清除。
// 2. errorState 各个位具体状态参考读取电机状态 1 和错误标志命令。

void clear_error(uint8_t id){
    uint8_t buf[LEN] = {0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}

//  读取电机状态 2 命令 0x9C


// 该命令读取当前电机的温度、电压、转速、编码器位置。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9C
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9C
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)



void read_status2(uint8_t id){
    uint8_t buf[LEN] = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}



//  读取电机状态 3 命令 0x9D
// 该命令读取当前电机的温度和相电流数据
// 数据域 说明 数据
// DATA[0] 命令字节 0x9D
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下数据：
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）
// 2. A 相电流数据，数据类型为 int16_t 类型，对应实际相电流为 1A/64LSB。
// 3. B 相电流数据，数据类型为 int16_t 类型，对应实际相电流为 1A/64LSB。
// 4. C 相电流数据，数据类型为 int16_t 类型，对应实际相电流为 1A/64LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9D
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] A 相电流低字节 DATA[2] = *(uint8_t *)(&iA)
// DATA[3] A 相电流高字节 DATA[3] = *((uint8_t *)(& iA)+1)
// DATA[4] B 相电流低字节 DATA[4] = *(uint8_t *)(&iB)
// DATA[5] B 相电流高字节 DATA[5] = *((uint8_t *)(& iB)+1)
// DATA[6] C 相电流低字节 DATA[6] = *(uint8_t *)(&iC)
// DATA[7] C 相电流高字节 DATA[7] = *((uint8_t *)(& iC)+1)


void read_status3(uint8_t id){
    uint8_t buf[LEN] = {0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}






//  电机关闭命令 0x80

// 关闭电机，同时清除电机运行状态和之前接收的控制指令
// 数据域 说明 数据
// DATA[0] 命令字节 0x80
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void motor_close(uint8_t id){
    uint8_t buf[LEN] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}


//  电机停止命令 0x81
// 停止电机，但不清除电机运行状态和之前接收的控制指令
// 数据域 说明 数据
// DATA[0] 命令字节 0x81
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void motor_stop(uint8_t id){
    uint8_t buf[LEN] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}


//  电机运行命令 0x88
// 从电机停止命令中恢复电机运行（恢复停止前的控制方式）
// 数据域 说明 数据
// DATA[0] 命令字节 0x88
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void motor_run(uint8_t id){
    uint8_t buf[LEN] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}


//  转矩开环控制命令 0xA0
// 主机发送该命令以控制电机的开环输出功率，控制值 powerControl 为 int16_t 类型，数值范围-1000~
// 1000，（电机母线电流和扭矩因不同电机而异）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA0
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 输出功率控制值低字节 DATA[4] = *(uint8_t *)(& powerControl)
// DATA[5] 输出功率控制值高字节 DATA[5] = *((uint8_t *)(& powerControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 1. 该命令中的控制值 powerControl 不受上位机(LK-Motor Tool)中的 Max Power 值限制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的输出功率值（int16_t 类型，范围-1000~1000）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA0
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 输出功率值低字节 DATA[2] = *(uint8_t *)(& power)
// DATA[3] 输出功率值高字节 DATA[3] = *((uint8_t *)(&power)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void torque_open_loop(uint8_t id, int16_t powerControl){
    uint8_t buf[LEN] = {0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& powerControl);
    buf[5] = *((uint8_t *)(& powerControl)+1);
    can_send(buf, id);
}



//  转矩闭环控制命令 0xA1
// 主机发送该命令以控制电机的转矩电流输出，控制值 iqControl 为 int16_t 类型，数值范围-2000~ 2000，
// 对应实际转矩电流范围-32A~32A（母线电流和电机的实际扭矩因不同电机而异）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA1
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 转矩电流控制值低字节 DATA[4] = *(uint8_t *)(&iqControl)
// DATA[5] 转矩电流控制值高字节 DATA[5] = *((uint8_t *)(&iqControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA1
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)



void torque_close_loop(uint8_t id, int16_t iqControl){
    uint8_t buf[LEN] = {0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& iqControl);
    buf[5] = *((uint8_t *)(& iqControl)+1);
    can_send(buf, id);
}



//  速度闭环控制命令 0xA2
// 主机发送该命令以控制电机的速度，控制值 speedControl 为 int32_t 类型，对应实际转速为 0.01dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA2
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 速度控制低字节 DATA[4] = *(uint8_t *)(&speedControl)
// DATA[5] 速度控制 DATA[5] = *((uint8_t *)(&speedControl)+1)
// DATA[6] 速度控制 DATA[6] = *((uint8_t *)(&speedControl)+2)
// DATA[7] 速度控制高字节 DATA[7] = *((uint8_t *)(&speedControl)+3)

// 1. 该命令下电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA2
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void speed_close_loop(uint8_t id, int32_t speedControl){
    uint8_t buf[LEN] = {0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& speedControl);
    buf[5] = *((uint8_t *)(& speedControl)+1);
    buf[6] = *((uint8_t *)(& speedControl)+2);
    buf[7] = *((uint8_t *)(& speedControl)+3);
    can_send(buf, id);
}




//  位置闭环控制命令 1 0xA3
// 主机发送该命令以控制电机的位置（多圈角度）， 控制值 angleControl 为 int32_t 类型，对应实际位
// 置为 0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA3
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该命令下的控制值 angleControl 受上位机(LK-Motor Tool)中的 Max Angle 值限制。
// 2. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制。
// 3. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 4. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA3
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop(uint8_t id, int32_t angleControl){
    uint8_t buf[LEN] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    buf[6] = *((uint8_t *)(& angleControl)+2);
    buf[7] = *((uint8_t *)(& angleControl)+3);
    can_send(buf, id);
}



//  位置闭环控制命令 2 0xA4
// 主机发送该命令以控制电机的位置（多圈角度）， 控制值 angleControl 为 int32_t 类型，对应实际位
// 置为 0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定。
// 控制值 maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转速 1dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA4
// DATA[1] NULL 0x00
// DATA[2] 速度限制低字节 DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字节 DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该命令下的控制值 angleControl 受上位机(LK-Motor Tool)中的 Max Angle 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 3. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA4
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)



void angle_close_loop_with_speed(uint8_t id, int32_t angleControl, uint16_t maxSpeed){
    uint8_t buf[LEN] = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // angleControl = angleControl * 8;
    buf[2] = *(uint8_t *)(& maxSpeed);
    buf[3] = *((uint8_t *)(& maxSpeed)+1);
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    buf[6] = *((uint8_t *)(& angleControl)+2);
    buf[7] = *((uint8_t *)(& angleControl)+3);
    can_send(buf, id);
}


//  位置闭环控制命令 3 0xA5
// 主机发送该命令以控制电机的位置（单圈角度）， 控制值 angleControl 为 uint16_t 类型，数值范围
// 0~35999，对应实际位置为 0.01degree/LSB，即实际角度范围 0°~359.99°。
// 控制值 spinDirection 设置电机转动的方向，为 uint8_t 类型，0x00 代表顺时针，0x01 代表逆时针。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA5
// DATA[1] 转动方向字节 DATA[1] = spinDirection
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制高字节 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 备注：
// 1. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 3. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA5
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction(uint8_t id, int32_t angleControl, uint8_t spinDirection){
    uint8_t buf[LEN] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[1] = spinDirection;
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    can_send(buf, id);
}


//  位置闭环控制命令 4 0xA6
// 主机发送该命令以控制电机的位置（单圈角度）。
// 1. 角度控制值 angleControl 为 uint16_t 类型，数值范围 0~35999，对应实际位置为 0.01degree/LSB，
// 即实际角度范围 0°~359.99°。
// 2. spinDirection 设置电机转动的方向，为 uint8_t 类型，0x00 代表顺时针，0x01 代表逆时针。
// 3. maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转速 1dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA6
// DATA[1] 转动方向字节 DATA[1] = spinDirection
// DATA[2] 速度限制低字节 DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字节 DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制高字节 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 备注：
// 1. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 2. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA6
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction_and_speed(uint8_t id, int32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed){
    uint8_t buf[LEN] = {0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[1] = spinDirection;
    buf[2] = *(uint8_t *)(& maxSpeed);
    buf[3] = *((uint8_t *)(& maxSpeed)+1);
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    can_send(buf, id);
}


//  增量位置闭环控制命令 5 0xA7
// 主机发送该命令以控制电机的位置增量， 控制值 angleControl 为 int32_t 类型，对应实际位置为
// 0.01degree/LSB。电机的转动方向由控制量的符号确定。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA7
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 3. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA7
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop4(uint8_t id, int32_t angleControl){
    uint8_t buf[LEN] = {0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    buf[6] = *((uint8_t *)(& angleControl)+2);
    buf[7] = *((uint8_t *)(& angleControl)+3);
    can_send(buf, id);
}





//  位置闭环控制命令 6 0xA8
// 主机发送该命令以控制电机的位置增量， 控制值 angleControl 为 int32_t 类型，对应实际位置为
// 0.01degree/LSB。电机的转动方向由控制量的符号确定。
// 控制值 maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转速 1dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA8
// DATA[1] NULL 0x00
// DATA[2] 速度限制低字节 DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字节 DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 2. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA8
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction_and_speed_and_max_speed(uint8_t id, int32_t angleControl, uint16_t maxSpeed){
    uint8_t buf[LEN] = {0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[2] = *(uint8_t *)(& maxSpeed);
    buf[3] = *((uint8_t *)(& maxSpeed)+1);
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    buf[6] = *((uint8_t *)(& angleControl)+2);
    buf[7] = *((uint8_t *)(& angleControl)+3);
    can_send(buf, id);
}


// 名称命令数据读取PID参数命令 0x30写入PID参数到RAM命令 0x31写入PID参数到ROM命令 0x32读取加速度命令 0x33写入加速度到RAM命令 0x34读取编码器命令 0x90写入编码器值到ROM作为电机零点命令 0x91写入当前位置到ROM作为电机零点命令 0x19读取多圈角度命令 0x92读取单圈角度命令 0x94清除电机角度命令（设置电机初始位置） 0x95读取电机状态1和错误标志命令 0x9A清除电机错误标志命令 Ox9B读取电机状态2命令 0x9C读取电机状态3命令 0x9D电机关闭命令 0x80电机停止命令 0x81电机运行命令 0x88转矩开环控制命令 0xAO转矩闭环控制命令 OxA1速度闭环控制命令 OxA2位置闭环控制命令1 OxA3位置闭环控制命令2 0xA4位置闭环控制命令3 OxA5位置闭环控制命令4 OxA6位置闭环控制命令5 OxA7位置闭环控制命令6 0xA8


// 根据之前的注释，判断接收到的数据是哪个命令，并执行相应的操作

void receive_process(uint8_t *data, uint8_t id){
    // uint8_t id = 0x140;
    uint8_t cmd = data[0];
    switch(cmd){
        //
        case 0x30:  // 读取PID参数命令
            motor[id].pi[0].p = data[2];
            motor[id].pi[0].i = data[3];
            motor[id].pi[1].p = data[4];
            motor[id].pi[1].i = data[5];
            motor[id].pi[2].p = data[6];
            motor[id].pi[2].i = data[7];
            break;
        case 0x31:  // 写入PID参数到RAM命令
            motor[id].pi[0].p = data[2];
            motor[id].pi[0].i = data[3];
            motor[id].pi[1].p = data[4];
            motor[id].pi[1].i = data[5];
            motor[id].pi[2].p = data[6];
            motor[id].pi[2].i = data[7];
            break;
        case 0x32:  // 写入PID参数到ROM命令
            motor[id].pi[0].p = data[2];
            motor[id].pi[0].i = data[3];
            motor[id].pi[1].p = data[4];
            motor[id].pi[1].i = data[5];
            motor[id].pi[2].p = data[6];
            motor[id].pi[2].i = data[7];
            break;
        case 0x33:  // 读取加速度命令
            // 32bit  7 6 5 4
            motor[id].acc = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
            break;
        case 0x34:  // 写入加速度到RAM命令
            motor[id].acc = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
            break;
        case 0x90:  // 读取编码器命令
        
            motor[id].encoder = data[2] | (data[3] << 8);  // 当前编码器值 （原始 - 电机零点）
            motor[id].encoder_i = data[4] | (data[5] << 8); //原始编码器值
            motor[id].encoder_zero = data[6] | (data[7] << 8); // 电机零点
            break;
        case 0x91:  // 写入编码器值到ROM作为电机零点命令
            // motor[id].encoder = data[2] | (data[3] << 8);
            // motor[id].encoder_i = data[4] | (data[5] << 8);
            motor[id].encoder_zero = data[6] | (data[7] << 8);
            break;
        case 0x19:  // 写入当前位置到ROM作为电机零点命令
            // motor[id].encoder = data[2] | (data[3] << 8);
            // motor[id].encoder_i = data[4] | (data[5] << 8);
            motor[id].encoder_zero = data[6] | (data[7] << 8);
            break;
        case 0x92:  // 读取多圈角度命令
        //int64
            motor[id].angle2_0 = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24) ;
            motor[id].angle2_1 = data[5] | (data[6] << 8) | (data[7] << 16) ;

            break;
        case 0x94:  // 读取单圈角度命令
            //int 16
            motor[id].angle = data[6] | (data[7] << 8);
            break;
        case 0x95:  // 清除电机角度命令（设置电机初始位置）//官方暂未实现
            // motor[id].encoder = data[2] | (data[3] << 8);
            // motor[id].encoder_i = data[4] | (data[5] << 8);
            // motor[id].encoder_zero = data[6] | (data[7] << 8);
            break;
        case 0x96:  // 读取电机温度命令
            motor[id].temp = data[1];
            // 电压
            motor[id].voltage = data[3] | (data[4] << 8);
            //状态 7
            motor[id].status = data[7];
            break;
        case 0x9B:  // 清除错误命令
            motor[id].temp = data[1];
            // 电压
            motor[id].voltage = data[3] | (data[4] << 8);
            //状态 7
            motor[id].status = data[7];
            break;
        case 0x9C:  // 读取电机状态2命令
            // 温度 1 电流 23 速度 45 编码器位置 67
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 电压
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0x9D:  // 读取电机状态3命令
            break;  //命令复杂，不需要
        case 0x80: //电机关闭
            break;
        case 0x81: //电机停止
            break;
        case 0x82: //电机运行
            break;
        case 0xA1: //转矩闭环
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0xA2: //速度闭环
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0xA3: //位置闭环1
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0xA4: //位置闭环2
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0xA5: //位置闭环3
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0xA6: //位置闭环4
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0xA7: //位置闭环5
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;
        case 0xA8: //位置闭环6
            // temp
            motor[id].temp = data[1];
            // 电流
            motor[id].current = data[2] | (data[3] << 8);
            // 速度
            motor[id].speed = data[4] | (data[5] << 8);
            // 编码器
            motor[id].encoder = data[6] | (data[7] << 8);
            break;

        default:
            break;
    }
}