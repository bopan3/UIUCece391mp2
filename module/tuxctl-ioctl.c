/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */
#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)



static unsigned int ack;
static unsigned long led_store;
static unsigned long button_store;
int init(struct tty_struct* tty);
int button(struct tty_struct* tty, unsigned long arg);
int set_led (struct tty_struct* tty, unsigned long arg);
int renew_button_by_irq(unsigned b, unsigned c);


/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
	unsigned a, b, c;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

    //printk("packet : %x %x %x\n", a, b, c);
    switch(a)
    {

     	case MTCP_ACK:
     		ack = 1;
     		break;

     	case MTCP_BIOC_EVENT:
     		renew_button_by_irq(b, c);
     		break;

     	case MTCP_RESET:
     		init(tty);	
     		if(!ack)
      			break;
		 	set_led(tty, led_store);	
     		break;
		 default:
     		break;
     }
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int tuxctl_ioctl (struct tty_struct* tty, struct file* file, unsigned cmd, unsigned long arg)
{
    switch (cmd) 
    {
		case TUX_INIT:
			return  init(tty);
		case TUX_BUTTONS:
			return  button(tty, arg);
		case TUX_SET_LED:
			return  set_led (tty, arg);
		case TUX_LED_ACK:
			return -EINVAL;
		case TUX_LED_REQUEST:
			return -EINVAL;
		case TUX_READ_LED:
			return -EINVAL;
		default:
	    	return -EINVAL;
    }
}

/*********************implementation of local functions*************************/

/*
 *init
 *DESCRIPTION: initialize tux
 *Input: tty - pointer to a tty_struct for ldisc functions
 *Output: None
 *Return Value: 0
 *Side Effects: 
 */
 int init(struct tty_struct* tty)
 {
 	unsigned char buf[2]; //need send 2 bytes for initial
	//set device to be busy
 	ack = 0;

 	buf[0] = MTCP_BIOC_ON;
 	buf[1] = MTCP_LED_USR;
 	tuxctl_ldisc_put(tty, buf, 2);

 	//initialize led_store and buttons
 	led_store = 0x00000000;
 	button_store=0x00FF;
 	return 0;
 }

/*
 *set_led
 *DESCRIPTION: set the LED according to arg
 *Input: tty - pointer to a tty_struct for ldisc functions
 *       arg - a 32-bit integer of the following form: The low 
 * 16-bits specify a number whose hexadecimal value is to be displayed
 *  on the 7-segment displays. The low 4 bits of the third byte specifies 
 * which LED’s should be turned on. The low 4 bits of the highest byte (bits 27:24)
 *  specify whether the corresponding decimal points should be turned on. 
 *Output: None
 *Return Value: 0 on suc, -1 on fail
 *Side Effects: 
 */
 int set_led (struct tty_struct* tty, unsigned long arg)
 {
	unsigned char seven_segment_information [16] = {0xE7, 0x06, 0xCB, 0x8F, 0x2E, 0xAD, 0xED, 0x86, 0xEF, 0xAF, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8};
 	unsigned char num_on;
 	unsigned char dot_on;		
 	unsigned long mask_1bit[4]={0x01,0x02,0x04,0x08}; 
 	unsigned long mask_4bit[4]={0x000F,0x00F0,0x0F00,0xF000}; 
 	unsigned char buf[6];  //no matter how many led to display, just send 6 bytes
	unsigned int  i;

 	if(ack==0)
 		return -1;
 	ack = 0;
 	led_store = arg; //save the current led of tux


 	num_on = (arg & (0x0F << 16)) >> 16;
 	dot_on = (arg & (0x0F << 24)) >> 24;
	 
	//no matter how many led to display, just send 6 bytes
 	buf[0] = MTCP_LED_SET;
 	buf[1] = 0x0F;

 	for(i = 0; i < 4; ++i){
 		if(num_on & mask_1bit[i]){
			// 2 + i because we the first two byte of buf has been filled
 			buf[2 + i] = seven_segment_information[(mask_4bit[i] & arg) >> (4*i)]; //  4 bit as a group for processing hex representation
 		}
 		else{
 			buf[2 + i] = 0x0;
 		}
 		if(dot_on & mask_1bit[i]){
 			buf[2 + i] |= 0x10;
		}
 	}
 	tuxctl_ldisc_put(tty, buf, 6);

	return 0;
 }
/*
 *renew_button_by_irq
 *DESCRIPTION: renew the value of button_store 
 *INPUT: b - packet data1
 *		 c - packet data2
 *OUPUT: None
 *Return Value: 0
 *Side Effects: renew the value of button_store 
 */
int renew_button_by_irq(unsigned b, unsigned c)
{
	unsigned int status_R;
	unsigned int status_L;
	unsigned int status_D;
	unsigned int status_U;
	unsigned int status_C;
	unsigned int status_B;
	unsigned int status_A;
	unsigned int status_S;

			// 		Byte 0 - MTCP_BIOC_EVENT
			// 		byte 1  +-7-----4-+-3-+-2-+-1-+---0---+
			// 		     	| 1 X X X | C | B | A | START |
			// 			    +---------+---+---+---+-------+
			// 		byte 2  +-7-----4-+---3---+--2---+--1---+-0--+
			// 			    | 1 X X X | right | down | left | up |
			// 			    +---------+-------+------+------+----+
	//the magic number below are the correspoding bit position in the table above
	status_R = (c & 0x01<<3) >> 3;
	status_L = (c & 0x01<<1) >> 1;
	status_D = (c & 0x01<<2) >> 2;
	status_U = (c & 0x01   )     ;
	status_C = (b & 0x01<<3) >> 3;
	status_B = (b & 0x01<<2) >> 2;
	status_A = (b & 0x01<<1) >> 1;
	status_S = (b & 0x01   )     ;

	button_store= (status_R<<7) | (status_L<<6) | (status_D<<5) | (status_U<<4) | (status_C<<3) | (status_B<<2) | (status_A<<1) | status_S;
	// the number 7,6,5,4,3,2,1 are the bit positions for the status_X in the required return format
	return 0;
}


/*
 *button
 *DESCRIPTION: put the status of button to arg
 *INPUT: pointer to a tty_struct for ldisc functions
 		 arg - where to put the status of button
 *OUPUT: None
 *Return Value: 0 if success
 *Side Effects: None
 */
int button(struct tty_struct* tty, unsigned long arg)
{
	unsigned long *buttons_ptr;
	int ret;
	buttons_ptr = &button_store;

	//copy to user space
	ret = copy_to_user((void *)arg, (void *)buttons_ptr, sizeof(unsigned long));

	if (ret > 0)
		return -EFAULT;
	else
		return 0;
}
