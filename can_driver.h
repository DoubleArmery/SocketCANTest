#ifndef __CAN_DRIVER_H
#define __CAN_DRIVER_H
/*
struct can_frame {
	canid_t can_id;//CAN 标识符 
	__u8 can_dlc;//数据场的长度 
	__u8 data[8];//数据 
};
*/
 
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
 
/*定义can缓存区大小*/
#define CAN_FRAME_BUFFER_SIZE 1024
/*定义缓冲区结构体*/
typedef struct CAN_BUFFER{
    struct can_frame frame_buf[CAN_FRAME_BUFFER_SIZE];
    unsigned int out;
    unsigned int in;
}can_buf_struct;
 
/*can 0发送函数 填充数据到发送缓存区*/
int can0_tx(struct can_frame *fram);
/*can 0接受函数 从接收缓存区获取接收到的CAN一帧数据*/
int can0_rx(struct can_frame *fram);
/*打印 can_frame  帧数据内容*/
void print_can_frame(struct can_frame *frame);
int can_rx_tx_init(void);
 
#endif