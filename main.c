#include"can_driver.h"
#include<stdio.h>

int main(){
    struct can_frame frame;
    can_rx_tx_init();
    while(1){
        if(can0_rx(&frame)==0){
            print_can_frame(&frame);
        }
    }
    

    return 0;
}