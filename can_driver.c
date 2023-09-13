#include "can_driver.h"
#define  CAN_ZERO "can0"
 
#define debug_print printf
 
static volatile can_buf_struct can0_rx_buf;
static volatile can_buf_struct can0_tx_buf;
static pthread_mutex_t can0_lock;
static pthread_t thread_read_id=0;
static pthread_t thread_write_id=0;
static int sockfd = -1;
 
static void * can0_send(void *v);
static void * can0_recive(void *v);
static void pthread_quit_slot(int sig);
 
#define ip_cmd_set_can0_params "ip link set can0 type can bitrate 1000000 triple-sampling on"
#define ip_cmd_can0_up         "ifconfig can0 up"
#define ip_cmd_can0_down       "ifconfig can0 down"






/*can 0发送函数 填充数据到发送缓存区*/
int can0_tx(struct can_frame *fram)
{
    int ret=0;
//    debug_print("can0_tx in = %d out = %d\r\n",can0_tx_buf.in,can0_tx_buf.out);
    memcpy(&can0_tx_buf.frame_buf[can0_tx_buf.in],
            fram,sizeof(struct can_frame));
    can0_tx_buf.in++;
    if(can0_tx_buf.in>=CAN_FRAME_BUFFER_SIZE){
        can0_tx_buf.in = 0;
    }
    if(can0_tx_buf.in==can0_tx_buf.out){
        debug_print("can0_tx_buf fill \r\n");
    }
 
//    pthread_mutex_lock(&can0_lock);
    // send frame to can 0
//    ret = write(sockfd, fram, sizeof(struct can_frame));
//    pthread_mutex_unlock(&can0_lock);
    return 0;
}
 
/*can 0接受函数 从接收缓存区获取接收到的CAN一帧数据*/
int can0_rx(struct can_frame *fram)
{
    if(can0_rx_buf.in==can0_rx_buf.out){
        return -1;
    }else{
        memcpy(fram,&can0_rx_buf.frame_buf[can0_rx_buf.out],sizeof(struct can_frame));
 
        can0_rx_buf.out++;
        if(can0_rx_buf.out>=CAN_FRAME_BUFFER_SIZE){
            can0_rx_buf.out = 0;
        }
    }
	return 0;
}
 
/*can0 初始化函数*/
 
 
static void can_rx_tx_release(int sig)
{
    (void)sig;
    if(thread_read_id>0)pthread_kill(thread_read_id,SIGQUIT);
    if(thread_write_id>0)pthread_kill(thread_write_id,SIGQUIT);
    if(thread_read_id)pthread_join(thread_read_id,NULL);
    if(thread_write_id)pthread_join(thread_write_id,NULL);
    if(sockfd>0)close(sockfd);
}
 
int can_rx_tx_init(void)
{

    // 使用系统调用函数运行以上命令，也可以自行在终端中运行
    system(ip_cmd_set_can0_params); // 设置参数
    system(ip_cmd_can0_up);  // 开启can0接口


	struct ifreq ifr = {0};
    struct sockaddr_can can_addr = {0};
 
    int ret;
 
    can0_rx_buf.in = 0;
    can0_rx_buf.out = 0;
    can0_tx_buf.in = 0;
    can0_tx_buf.out = 0;
    pthread_mutex_init(&can0_lock,NULL);
    /* 打开套接字 */
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(0 > sockfd) {
        perror("socket error");
		debug_print("socket error");
        exit(EXIT_FAILURE);
    }
	
    /* 指定can0设备 */
    strcpy(ifr.ifr_name, CAN_ZERO);
    ioctl(sockfd, SIOCGIFINDEX, &ifr);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;
 
    /* 将can0与套接字进行绑定 */
    ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr));
    if (0 > ret) {
        perror("bind error");
		debug_print("bind error");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    ret = pthread_create(&thread_read_id, NULL,can0_recive, &sockfd);
	if (0 > ret) {
		debug_print(" read pthread_create fail\r\n");	
	}
    ret = pthread_create(&thread_write_id, NULL,can0_send, &sockfd);
	if (0 > ret) {
		debug_print(" write pthread_create fail\r\n");	
	}
 
    signal(SIGQUIT,can_rx_tx_release);
	return 0;
}
 
 
/*私有函数 can0发送缓冲区数据*/
static void * can0_send(void *v)
{
    int *p = v;
    int sockfd = *p;
    int ret=0;
    //= (int)(*v);
    struct can_frame *frame;
    signal(SIGQUIT,pthread_quit_slot);
    while(1){
//        debug_print("%s,%d\r\n",__FUNCTION__,__LINE__);
 
        if(can0_tx_buf.in==can0_tx_buf.out){
//            debug_print("can0_tx_buf empty \r\n");
        }else{
            debug_print("%s,%d\r\n",__FUNCTION__,__LINE__);
            frame = &can0_tx_buf.frame_buf[can0_tx_buf.out];
            can0_tx_buf.out++;
            if(can0_tx_buf.out >= CAN_FRAME_BUFFER_SIZE){
                can0_tx_buf.out = 0;
            }
            pthread_mutex_lock(&can0_lock);
            // send frame to can 0
            ret = write(sockfd, frame, sizeof(struct can_frame));
            pthread_mutex_unlock(&can0_lock);
            if(ret!=sizeof(struct can_frame)){
                debug_print("send can frame error\r\n");
            }
        }
        usleep(0);
    }
    return NULL;
}
 
/*私有函数 can0接收数据到缓冲区*/
static void * can0_recive(void *v)
{
    int *p = v;
    int sockfd = *p;
    int ret=0;
    struct can_frame frame;
    signal(SIGQUIT,pthread_quit_slot);
    while(1){
        pthread_mutex_lock(&can0_lock);
        ret = read(sockfd, &frame, sizeof(struct can_frame));
        pthread_mutex_unlock(&can0_lock);
 
        if (0 > ret) {
            debug_print("read can frame error %s,%d",__FUNCTION__,__LINE__);
//            break;
        }else{
            debug_print("%s,%d\r\n",__FUNCTION__,__LINE__);
            memcpy(&can0_rx_buf.frame_buf[can0_rx_buf.in],&frame,sizeof(struct can_frame));
            can0_rx_buf.in++;
            if(can0_rx_buf.in >= CAN_FRAME_BUFFER_SIZE){
              can0_rx_buf.in = 0;
            }
            if(can0_rx_buf.in==can0_rx_buf.out){
                debug_print("can0_rx_buf fill \r\n");
            }
            usleep(0);
//            print_can_frame(&frame);
        }
    }
    return NULL;
}
 
static void pthread_quit_slot(int sig)
{
    (void)sig;
    pthread_exit(0);
}
 
/*打印 can_frame  帧数据内容*/
void print_can_frame(struct can_frame *frame)
{
    /* 校验帧格式 */
    if (frame->can_id & CAN_EFF_FLAG)	//扩展帧
        printf("扩展帧 <0x%08x> ",frame->can_id & CAN_EFF_MASK);
	else		//标准帧
        printf("标准帧 <0x%03x> ",frame->can_id & CAN_SFF_MASK);
 
 	/* 校验帧类型：数据帧还是远程帧 */
    if (frame->can_id & CAN_RTR_FLAG) {
        debug_print("remote request\n");
        return ;
    }
	/* 打印数据长度 */
    printf("[%d] ", frame->can_dlc);
	/* 打印数据 */
    for (int i = 0; i < frame->can_dlc; i++)
        printf("%02x ", frame->data[i]);
    printf("\n");
}
 
 
 
//static void pthread_write_quit_slot(int sig)
//{
//    pthread_kill(thread_read_id,SIG_QUIT);
//    pthread_join(thread_read_id);
//    pthread_exit(0);
//}
 
 
 
 
 
 
 