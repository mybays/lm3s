//*****************************************************************************
//   a example of tcp test for uip
//*****************************************************************************

#include "uip.h"
#include "tcp.h"
#include "string.h"

///unsigned char databuf[10] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99};
///void *SENDDATA = &databuf[0]; 

//tcp init function
void tcp_init(void) 
{
	uip_listen(HTONS(8000));
}

//tcp appcall function
void tcp_appcall(void) 
{
    char *hello = "Hi, This is a TCP test!\n";
	
    if(uip_connected())
    {
        uip_send(hello,strlen(hello)+1);
    }
    if(uip_newdata()) 
    {
		uip_send(uip_appdata,uip_len);
    }
}

