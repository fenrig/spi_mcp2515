/* http://www.linuxchix.org/content/courses/kernel_hacking/lesson5
 * http://www.linuxjournal.com/article/8110?page=0,1
 * http://tuomasnylund.fi/drupal6/content/making-embedded-linux-kernel-device-driver
 * http://www.linuxjournal.com/article/6930
 */

// TODO: replace 16mhz osc with 10mhz osc calculations (can baud)
// TODO: make kernelspace/userspace interactions for setting baud and mode

#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/moduleparam.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/mutex.h>

#define DEBUG 1
#define LOOPBACK 1

/* ------------------------
 * Module information
 */
#define MODULE_NAME "spi_mcp2515"
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION(MODULE_NAME);
MODULE_VERSION("0.1");
MODULE_AUTHOR("Matthias Van Gestel");

/* ------------------------
 * SPI Part
 */

// Buffer size for the largest SPI transfer (CAN FRAME)
#define CAN_FRAME_MAX_DATA_LEN 	8
#define SPI_TRANSFER_BUF_LEN 	(6 + CAN_FRAME_MAX_DATA_LEN)
#define CAN_FRAME_MAX_BITS		128


// SPI Data pointers
 typedef struct{
 	struct spi_device *spidev;
 	__u8 *tx;
 	__u8 *rx;
	// __u8 rx_size; 
 } spi_data;

 spi_data spidata = {
	.spidev = NULL,
	.tx = NULL,
	.rx = NULL,
	//.rx_size = 0,
 };

static int spi_transfer(struct spi_device *spi, int len){
	struct spi_transfer transfer = {
		.tx_buf = spidata.tx,
		.rx_buf = spidata.rx,
		.len = len,
		.cs_change = 0,
	};

	struct spi_message message;
	int ret;

	spi_message_init(&message);
	spi_message_add_tail(&transfer, &message);
	ret = spi_sync(spi, &message);
	if(ret)
		dev_err(&spi->dev, "spi transfer failed: spi_sync() = %d\n", ret);
	return ret;
}

static int spidriver_resume(struct spi_device *spi){
	// No implementation
	return 0;
}

static int spidriver_suspend(struct spi_device *spi, pm_message_t state){
	// No implementation
	return 0;
}

static int spidriver_remove(struct spi_device *spi){
	// No implementation
	return 0;
}

static int mcp2515_probe(struct spi_device *spi);

static int spidriver_probe(struct spi_device *spi){
	int ret = 0;

	if(DEBUG)
		printk(KERN_INFO "SPI: Probe started\n");

	spidata.spidev = spi;
	dev_set_drvdata(&spi->dev, &spidata); // gebruikt om data in te laden en te gebruiken in de spi struct functies
	dev_info(&spi->dev, "spi registered");

	// Register SPI Buffer space (size = spi + max can frame)
	spidata.tx = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if(! spidata.tx){
		ret = -ENOMEM;
		goto error_tx;
	}
	spidata.rx = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if(! spidata.rx){
		ret = -ENOMEM;
		goto error_rx;
	}

	// Configure SPI
	spi->mode = spi->mode ? : SPI_MODE_0;
	spi->max_speed_hz = spi->max_speed_hz ? : (10 * 1000 * 1000);
	spi->bits_per_word = 8;
	spi_setup(spi);
	
	/* CAN */
	//	Probe for MCP
	if(! mcp2515_probe(spi) ){
		ret = -ENODEV;
		goto error_probe;
	}
	
	if(DEBUG)
		printk(KERN_INFO "MCP2515: Succesfull probe\n");
	dev_info(&spi->dev, "Succesfull probe\n");

	return ret;

error_probe:
	kfree(spidata.rx);
error_rx:
	kfree(spidata.tx);
error_tx:
	printk(KERN_ALERT "MCP2515: Failed SPI probe\n");
	return ret;
} 

static struct spi_driver spi_mcp2515_driver = {
	.driver = {
		.name = "mcp2515",
		//.bus = &spi_bus_type, // line 87
		.owner = THIS_MODULE,
	},
	.probe = spidriver_probe,
	.remove = spidriver_remove,
	.suspend = spidriver_suspend,
	.resume = spidriver_resume,
};

static int __init initSPI(void){
	int ret = 0;
	ret = spi_register_driver(&spi_mcp2515_driver);
	if(ret)
		printk(KERN_ALERT "SPI: Problem with spi_register_driver() = %d\n" , ret);
	else if(DEBUG)
		printk(KERN_INFO "SPI: Succeeded spi_register_driver()\n");
	return ret;
}

int __exit exitSPI(void){
	if(spidata.tx != NULL)
		kfree(spidata.tx);
	if(spidata.rx != NULL)
		kfree(spidata.rx);
	return 0;
}

/* ------------------------
 * Message Queue definitions
 */
struct can_message{
	struct list_head list;
	__u16 id;
	__u8  data[CAN_FRAME_MAX_DATA_LEN];
	__u8  data_len;
};

struct transmit_buffer{
	struct can_message *msg;
} transmit_buffer[3] = {{NULL}, {NULL}, {NULL}};

struct can_message in_message_queue;
struct can_message out_message_queue;
struct mutex in_message_mutex;
struct mutex interrupt_mutex;
struct mutex out_message_mutex;

/* ------------------------ 
 * CAN
 */

// Registers (mcp2515)
#define EXIDE 3
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define CANSTATUS 0x0E
#define CANCONTROL 0x0F
  #define CANCTRL_REQOP_MASK 0xe0
  #define CANCTRL_REQOP_CONF 0x80
#define TXB0EID0 0x34
#define RXF0SIDH 0x00
#define TXB0EID8 0x3
#define CNF1 0x2A
#define CNF2 0x29
#define CNF3 0x28
#define CANINTF 0x2c
  #define CANINTF_MERRF 0x80
  #define CANINTF_WAKIF 0x40
  #define CANINTF_ERRIF 0x20
  #define CANINTF_TX2IF 0x10
  #define CANINTF_TX1IF 0x08
  #define CANINTF_TX0IF 0x04
  #define CANINTF_RX1IF 0x02
  #define CANINTF_RX0IF 0x01
  #define CANINTF_RX (CANINTF_RX0IF | CANINTF_RX1IF)
  #define CANINTF_TX (CANINTF_TX2IF | CANINTF_TX1IF | CANINTF_TX0IF)
  #define CANINTF_ERR (CANINTF_ERRIF)
#define CANINTE 0x2b
  #define CANINTE_MERRE 0x80
  #define CANINTE_WAKIE 0x40
  #define CANINTE_ERRIE 0x20
  #define CANINTE_TX2IE 0x10
  #define CANINTE_TX1IE 0x08
  #define CANINTE_TX0IE 0x04
  #define CANINTE_RX1IE 0x02
  #define CANINTE_RX0IE 0x01

// Instructions (spi)
#define CAN_RESET 0b11000000
#define CAN_WRITE 0x02
#define CAN_READ 0x03
#define CAN_BITMODIFY 0x05
#define GARBAGE 0x00

#define LOAD_TX_BUF_0_ID 0x40
#define LOAD_TX_BUF_0_DATA 0x41
#define LOAD_TX_BUF_1_ID 0x42
#define LOAD_TX_BUF_1_DATA 0x43
#define LOAD_TX_BUF_2_ID 0x44
#define LOAD_TX_BUF_2_DATA 0x45

#define SEND_TX_BUF_0 0x81
#define SEND_TX_BUF_1 0x82
#define SEND_TX_BUF_2 0x83

#define READ_RX_BUF_0_ID 0x90
#define READ_RX_BUF_0_DATA 0x92
#define READ_RX_BUF_1_ID 0x94
#define READ_RX_BUF_1_DATA 0x96
#define READ_STATUS 0xA0

// Modes
#define MCP2515_CONFIGURATION 0x80;
#define MCP2515_NORMAL 0x00;
#define MCP2515_SLEEP 0x20;
#define MCP2515_LISTEN 0x60;
#define MCP2515_LOOPBACK 0x40;

// Baudrates
#define CAN_BAUD_10K 1
#define CAN_BAUD_50K 2
#define CAN_BAUD_100K 3
#define CAN_BAUD_125K 4
#define CAN_BAUD_250K 5
#define CAN_BAUD_500K 6

// Specifications
#define TRANSMIT_BUFFER_SIZE 3

static __u8 mcp2515_read_register(struct spi_device *spi, __u8 reg){
	__u8 val;
	spidata.tx[0] = CAN_READ;
	spidata.tx[1] = reg;

	spi_transfer(spi, 3);
	val = spidata.rx[2];
	return val;
}

static void mcp2515_read_2_registers(struct spi_device *spi, __u8 reg, __u8 *value1, __u8 *value2){
	spidata.tx[0] = CAN_READ;
	spidata.tx[1] = reg;
	spidata.tx[2] = GARBAGE;
	spidata.tx[3] = GARBAGE;

	spi_transfer(spi, 4);
	*value1 = spidata.rx[2];
	*value2 = spidata.rx[3];
	return;
}

static void mcp2515_write_register(struct spi_device *spi, __u8 reg, __u8 val){
	spidata.tx[0] = CAN_WRITE;
	spidata.tx[1] = reg;
	spidata.tx[2] = val;

	spi_transfer(spi, 3);
	return;
}

static void mcp2515_write_registerbit(struct spi_device *spi, __u8 reg, __u8 bit ,__u8 val){
	spidata.tx[0] = CAN_BITMODIFY;
	spidata.tx[1] = reg;
	spidata.tx[2] = bit; // (1 << bit);
	/*
	if(val)
		spidata.tx[3] = 0xff;
	else
		spidata.tx[3] = 0x00;
	*/
	spidata.tx[3] = val;

	spi_transfer(spi, 4);
	return;
}

static void mcp2515_load_0(struct spi_device *spi, __u8 id, __u8 data){
	spidata.tx[0] = LOAD_TX_BUF_0_ID;
	spidata.tx[1] = id;
	spi_transfer(spi, 2);

	spidata.tx[0] = LOAD_TX_BUF_0_DATA;
	spidata.tx[1] = data;
	spi_transfer(spi, 2);
	return;
}

static void mcp2515_load_1(struct spi_device *spi, __u8 id, __u8 data){
	spidata.tx[0] = LOAD_TX_BUF_1_ID;
	spidata.tx[1] = id;
	spi_transfer(spi, 2);

	spidata.tx[0] = LOAD_TX_BUF_1_DATA;
	spidata.tx[1] = data;
	spi_transfer(spi, 2);
	return;
}

static void mcp2515_load_2(struct spi_device *spi, __u8 id, __u8 data){
	spidata.tx[0] = LOAD_TX_BUF_2_ID;
	spidata.tx[1] = id;
	spi_transfer(spi, 2);

	spidata.tx[0] = LOAD_TX_BUF_2_DATA;
	spidata.tx[1] = data;
	spi_transfer(spi, 2);
	return;
}

static void mcp2515_send_0(struct spi_device *spi){
	spidata.tx[0] = SEND_TX_BUF_0;
	spi_transfer(spi, 1);
	return;
}

static void mcp2515_send_1(struct spi_device *spi){
	spidata.tx[0] = SEND_TX_BUF_1;
	spi_transfer(spi, 1);
	return;
}

static void mcp2515_send_2(struct spi_device *spi){
	spidata.tx[0] = SEND_TX_BUF_2;
	spi_transfer(spi, 1);
	return;
}

static void mcp2515_load_frame_0(struct spi_device *spi, __u16 id, __u8 *data, __u8 length){
	__u8 id_high, id_low;
	__u8 i;

	id_high = (__u8) (id >> 3);
	id_low  = (__u8) ((id << 5) & 0x00E0);

	spidata.tx[0] = LOAD_TX_BUF_0_ID;
	spidata.tx[1] = id_high;
	spidata.tx[2] = id_low;
	spidata.tx[3] = 0x00; // extended id registers (unused)
	spidata.tx[4] = 0x00;
	spidata.tx[5] = length;
	for(i = 0; i < length; i++)
		spidata.tx[6 + i] = data[i];
	spi_transfer(spi, length + 6);

	return;
}

static void mcp2515_load_frame_1(struct spi_device *spi, __u16 id, __u8 *data, __u8 length){
	__u8 id_high, id_low;
	__u8 i;

	id_high = (__u8) (id >> 3);
	id_low  = (__u8) ((id << 5) & 0x00E0);

	spidata.tx[0] = LOAD_TX_BUF_1_ID;
	spidata.tx[1] = id_high;
	spidata.tx[2] = id_low;
	spidata.tx[3] = 0x00; // extended id registers (unused)
	spidata.tx[4] = 0x00;
	spidata.tx[5] = length;
	for(i = 0; i < length; i++)
		spidata.tx[6 + i] = data[i];
	spi_transfer(spi, length + 6);

	return;
}

static void mcp2515_load_frame_2(struct spi_device *spi, __u16 id, __u8 *data, __u8 length){
	__u8 id_high, id_low;
	__u8 i;

	id_high = (__u8) (id >> 3);
	id_low  = (__u8) ((id << 5) & 0x00E0);

	spidata.tx[0] = LOAD_TX_BUF_2_ID;
	spidata.tx[1] = id_high;
	spidata.tx[2] = id_low;
	spidata.tx[3] = 0x00; // extended id registers (unused)
	spidata.tx[4] = 0x00;
	spidata.tx[5] = length;
	for(i = 0; i < length; i++)
		spidata.tx[6 + i] = data[i];
	spi_transfer(spi, length + 6);

	return;
}

static __u8 mcp2515_read_id_0(struct spi_device *spi){
	__u8 ret;

	spidata.tx[0] = READ_RX_BUF_0_ID;
	spidata.tx[1] = GARBAGE;
	spi_transfer(spi, 2);

	ret = spidata.rx[1];
	return ret;
}

static __u8 mcp2515_read_id_1(struct spi_device *spi){
	__u8 ret;

	spidata.tx[0] = READ_RX_BUF_1_ID;
	spidata.tx[1] = GARBAGE;
	spi_transfer(spi, 2);

	ret = spidata.rx[1];
	return ret;
}

static __u8 mcp2515_read_data_0(struct spi_device *spi){
	__u8 ret;

	spidata.tx[0] = READ_RX_BUF_0_DATA;
	spidata.tx[1] = GARBAGE;
	spi_transfer(spi, 2);

	ret = spidata.rx[1];
	return ret;
}

static __u8 mcp2515_read_data_1(struct spi_device *spi){
	__u8 ret;

	spidata.tx[0] = READ_RX_BUF_1_DATA;
	spidata.tx[1] = GARBAGE;
	spi_transfer(spi, 2);

	ret = spidata.rx[1];
	return ret;
}

static __u8 mcp2515_read_status(struct spi_device *spi){
	__u8 ret;

	spidata.tx[0] = READ_STATUS;
	spidata.tx[1] = GARBAGE;
	spi_transfer(spi, 2);

	ret = spidata.rx[1];
	return ret;
}

static void mcp2515_read_frame_0(struct spi_device *spi, __u16 *id, __u8 *data, __u8 *length){
	__u16 id_h, id_l;
	__u8 i, len = 8;	

	spidata.tx[0] = READ_RX_BUF_0_ID;
	spidata.tx[1] = GARBAGE; // id_high
	spidata.tx[2] = GARBAGE; // id_low
	spidata.tx[3] = GARBAGE; // unused id
	spidata.tx[4] = GARBAGE; // unused id
	spidata.tx[5] = GARBAGE; // data length
	for(i = 0; i < len; i++)
		spidata.tx[6 + i] = GARBAGE;
	spi_transfer(spi, len + 6);

	len = spidata.rx[5] & 0x0F;
	id_h = (__u16) spidata.rx[1];
	id_l = (__u16) spidata.rx[2];

	for(i = 0; i < len; i++)
		data[i] = spidata.rx[6 + i];

	*length = len;
	*id = ((id_h << 3) + ((id_l & 0xE0) >> 5));
	return;
}

static void mcp2515_read_frame_1(struct spi_device *spi, __u16 *id, __u8 *data, __u8 *length){
	__u16 id_h, id_l;
	__u8 i, len = 8;	

	spidata.tx[0] = READ_RX_BUF_1_ID;
	spidata.tx[1] = GARBAGE; // id_high
	spidata.tx[2] = GARBAGE; // id_low
	spidata.tx[3] = GARBAGE; // unused id
	spidata.tx[4] = GARBAGE; // unused id
	spidata.tx[5] = GARBAGE; // data length
	for(i = 0; i < len; i++)
		spidata.tx[6 + i] = GARBAGE;
	spi_transfer(spi, len + 6);

	len = spidata.rx[5] & 0x0F;
	id_h = (__u16) spidata.rx[1];
	id_l = (__u16) spidata.rx[2];

	for(i = 0; i < len; i++)
		data[i] = spidata.rx[6 + i];

	*length = len;
	*id = ((id_h << 3) + ((id_l & 0xE0) >> 5));
	return;
}

static int MCP2515_set_CANBAUD(int baudConst){
	__u8 brp;

	//BRP<5:0> = 00h, so divisor (0+1)*2 for 125ns per quantum at 16MHz for 500K
	//SJW<1:0> = 00h, Sync jump width = 1
	switch(baudConst){
   		case CAN_BAUD_500K: brp = 0; break;
    	case CAN_BAUD_250K: brp = 1; break;
    	case CAN_BAUD_125K: brp = 3; break;
    	case CAN_BAUD_100K: brp = 4; break;
    	default: return 1;
  	}

  	if(DEBUG)
  		printk("MCP2515: Changing CAN Baudrate\n");

  	mcp2515_write_register(spidata.spidev, CNF1, (brp & 0b00111111));
	
	//PRSEG<2:0> = 0x01, 2 time quantum for prop
	//PHSEG<2:0> = 0x06, 7 time constants to PS1 sample
	//SAM = 0, just 1 sampling
	//BTLMODE = 1, PS2 determined by CNF3
  	mcp2515_write_register(spidata.spidev, CNF2, 0b10110001);

  	//PHSEG2<2:0> = 5 for 6 time constants after sample
  	mcp2515_write_register(spidata.spidev, CNF3, 0x05);

  	//SyncSeg + PropSeg + PS1 + PS2 = 1 + 2 + 7 + 6 = 16

  	return 0;
}

static void mcp2515_set_CAN_mode(struct spi_device *spi, __u8 can_mode){
	/*
	__u8 settings = 0b00000111;
	__u8 mode;

	singleShot &= 0b00000001;

	settings |= (singleShot << 3);

	mcp2515_write_register(spidata.spidev, CANCONTROL, settings);

	// Read mode and make sure it is normal
	mode = mcp2515_read_register(spidata.spidev, CANSTATUS) >> 5;
	if(mode != 0)
		return 1;

	return 0;
	*/
	mcp2515_write_registerbit(spi, CANCONTROL, 0xE0, can_mode);
	return;
}

static int mcp2515_reset(struct spi_device *spi){
	int ret;
	unsigned long timeout;

	if(DEBUG)
		printk(KERN_INFO "MCP2515: Send RESET 0x%02x\n", CAN_RESET);

	spidata.tx[0] = CAN_RESET;
	ret = spi_write(spi, spidata.tx, 1);
	if(ret) {
		printk(KERN_ALERT "MCP2515: RESET failed: spi_write() = %d\n", ret);
		dev_err(&spi->dev, "MCP2515 RESET failed: spi_write() = %d\n", ret);
		return -EIO;
	}
	timeout = jiffies + HZ;
	mdelay(10);
	while(((mcp2515_read_register(spi, CANSTATUS)) & (CANCTRL_REQOP_MASK) != (CANCTRL_REQOP_CONF))){
		if(DEBUG)
			printk("MCP2515: Waiting for MCP2515 to enter in conf mode after RESET\n");
		schedule();
		if(time_after(jiffies, timeout)){
			printk(KERN_ALERT "MCP2515: did not enter conf mode after RESET\n");
			dev_err(&spi->dev, "MCP2515 did not enter conf mode after RESET\n");
			return -EBUSY;
		}
	}

	if(DEBUG)
		printk("MCP2515: MCP2515 in conf mode after RESET\n");
	return 0;
}

static int mcp2515_probe(struct spi_device *spi){
	int st1, st2;

	if(DEBUG)
		printk(KERN_INFO "MCP2515: probing mcp2515\n");

	mcp2515_reset(spi);

	st1 = mcp2515_read_register(spi, CANSTATUS) & 0xEE;
	st2 = mcp2515_read_register(spi, CANCONTROL) & 0x17;

	if(DEBUG)
		printk(KERN_INFO "MCP2515: CANSTATUS= 0x%02x | CANCONTROL= 0x%02x\n", st1, st2);
	dev_dbg(&spi->dev, "CANSTATUS= 0x%02x | CANCONTROL= 0x%02x\n", st1, st2);

	// Check power up default values
	return (st1 == 0x80 && st2 == 0x07) ? 1 : 0;
}

/* ------------------------
 * Char device
 */

#define SCULL_MAJOR 0 // Dynamic major
#define SCULL_MINOR 0

// http://www.makelinux.net/ldd3/chp-2-sect-8
// http://os.csie.ncku.edu.tw/drupal/sites/default/files/23/IODD_lab5.pdf
int scull_major = SCULL_MAJOR;
int scull_minor = SCULL_MINOR;

module_param(scull_major, int, S_IRUGO);
module_param(scull_minor, int, S_IRUGO);

static loff_t mcp2515_llseek(struct file *filp, loff_t off, int whence);
static ssize_t mcp2515_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t mcp2515_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static int mcp2515_open(struct inode *inode, struct file *filp);
static int mcp2515_release(struct inode *inode, struct file *filp);

static struct file_operations mcp2515_fops = {
	.owner  = THIS_MODULE,
	.llseek = mcp2515_llseek,
	.read   = mcp2515_read,
	.write  = mcp2515_write,
	.open   = mcp2515_open,
	.release= mcp2515_release,
};

struct mcp2515_device{
	struct cdev *cdev;
};

static struct mcp2515_device device;

static loff_t mcp2515_llseek(struct file *filp, loff_t off, int whence){
	printk("MCP2515: SEEK\n");
	return 0;
}

// TODO: continue reading page 24 of LDD3 chapter 03
// TODO: implement read, write and after that llseek
static ssize_t mcp2515_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos){
	struct list_head *pos,*q;
	struct can_message *tmp;
	int i, k, j = 0;
	char *kbuf, *help;
	long length;
	int ret;
	
	count /= (2 + 1 + CAN_FRAME_MAX_DATA_LEN);
	if(! count){
		printk(KERN_INFO "MCP2515: mcp2515_read() - COUNT NOT BIG ENOUGH\n");
		ret = -EFAULT;
		goto error;
	}
	
	help = kbuf = (char*) kmalloc(count * (2 + 1 + CAN_FRAME_MAX_DATA_LEN), GFP_KERNEL);
	if(! kbuf){
		printk(KERN_ALERT "MCP2515: mcp2515_read() - Cannot allocate memory\n");
		ret = -ENOMEM;
		goto error;
	}

	mutex_lock(&in_message_mutex);

	list_for_each_safe(pos, q, &in_message_queue.list){
		tmp = list_entry(pos, struct can_message, list);

		if(tmp == NULL) break;

		help[0] = (tmp->id >> 8) & 0xff;
		help[1] = (tmp->id) & 0xff;
		help[2] = tmp->data_len;

		for(i = 3, k = 0; k < help[2]; i++, k++)
			help[i] = tmp->data[k];
		for(; k < CAN_FRAME_MAX_DATA_LEN; k++, i++)
			help[i] = 0x00;

		help += (2 + 1 + CAN_FRAME_MAX_DATA_LEN);
		
		list_del(pos);
		kfree(tmp);

		j++;
		
		if(j >= count)
			break;
			
	}

	mutex_unlock(&in_message_mutex);

	length = help - kbuf;
	if(copy_to_user(buf, kbuf, length)){
		printk("MCP2515: mcp2515_read() - Can't copy from kernelspace to userspace\n");
		ret = -EFAULT;
		goto free_error;
	}
	ret = length;

free_error:
	kfree(kbuf);
error:
	return ret;
}

static ssize_t mcp2515_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos){
	int i, j;
	ssize_t ret;
	struct can_message *tmp;
	__u8 kbuf[2 + 1 + CAN_FRAME_MAX_DATA_LEN];
	__u8 data[CAN_FRAME_MAX_DATA_LEN];

	if(count < 4){
		printk(KERN_ALERT "MCP2515: mcp2515_write() - count nog big enough (< 4) = 0x%x\n", count);
		ret = -1; // TODO: correct error, make more informative;
		goto error;
	}else if(count > 11){
		printk(KERN_ALERT "MCP2515: mcp2515_write() - count too big (> 11) = 0x%x\n", count);
		ret = -1;
		goto error;
	}

	j = copy_from_user(kbuf, buf, count);
	if(j){
		printk(KERN_ALERT "MCP2515: mcp2515_write() - couldn't copy full buffer from userspace [0x%x/0x%x]\n", count, i);
		ret = -1; // TODO: correct error, make more informative;
		goto error;
	}

	tmp = (struct can_message*) kmalloc(sizeof(struct can_message), GFP_KERNEL);
	tmp->id = (kbuf[0] << 8) |  (kbuf[1]);
	tmp->data_len = kbuf[2];
	for(i = 0, j = 3; i < tmp->data_len; i++, j++){
		tmp->data[i] = kbuf[j];
	}
	for(; i < CAN_FRAME_MAX_DATA_LEN; i++){
		tmp->data[i] = 0x00;
	}

	for(i = 0;i < 3; i++){
		if(! transmit_buffer[i].msg ) break;
	}

	if(i == 3){
		// Adding to out_message_queue
		mutex_lock(&out_message_mutex);
		list_add_tail(&(tmp->list),&(out_message_queue.list));
		mutex_unlock(&out_message_mutex);
		if(DEBUG) 
			printk(KERN_INFO "MCP2515: Added message to out_message_queue list 0x%x\n", tmp);
	}else{
		if(LOOPBACK) mutex_lock(&interrupt_mutex);
		transmit_buffer[i].msg = tmp;
		if(DEBUG)
			printk(KERN_INFO "MCP2515: Not adding to out_message_queue list sending with buffer 0x%x\n", i);
		switch(i){
			case 0: mcp2515_load_frame_0(spidata.spidev, tmp->id, tmp->data, tmp->data_len);
					mcp2515_send_0(spidata.spidev);
					break;
			case 1: mcp2515_load_frame_1(spidata.spidev, tmp->id, tmp->data, tmp->data_len);
					mcp2515_send_1(spidata.spidev);
					break;
			case 2: mcp2515_load_frame_2(spidata.spidev, tmp->id, tmp->data, tmp->data_len);
					mcp2515_send_2(spidata.spidev);
					break;
		}
		if(LOOPBACK){
			msleep(25);
			mutex_unlock(&interrupt_mutex);
		}
	}
	
	ret = count;
error:
	return ret;
}

static int mcp2515_release(struct inode *inode, struct file *filp){
	// nothing todo -> no implementation
	
	return 0;
}

static int mcp2515_open(struct inode *inode, struct file *filp){
	struct mcp2515_device *dev;
	printk(KERN_INFO "MCP2515: OPEN\n");
	// Eigenlijk weten we dit staan, maar toch zullen we dynamisch maar onze
	// mcp2515_device structuur zoeken aan de hand van de inode. 
	//dev = container_of(inode->i_cdev, struct mcp2515_device, cdev);
	// 
	filp->private_data = &device; 
	return 0;
}

/* ------------------------
 * KERNEL
 */
// Interrupt data
struct interrupt_data{
	int irq_number;

} interrupt_data;

struct interrupt_data interrupt_data = {
	.irq_number = 0,
};

static void add_message(struct list_head *addtolist, struct mutex *mutex, __u8 id, const __u8 *data, __u8 data_len){
	__u8 i;

	struct can_message *tmp;

	tmp = (struct can_message*) kmalloc(sizeof(struct can_message), GFP_KERNEL);
	tmp->id = id;
	tmp->data_len = data_len;

	for(i = 0; i < data_len; i++)
		tmp->data[i] = data[i];
	for(; i < CAN_FRAME_MAX_DATA_LEN; i++)
		tmp->data[i] = 0;

	mutex_lock(mutex);

	list_add_tail(&(tmp->list), addtolist);

	mutex_unlock(mutex);

	return;
}

static int check_queue_send_message(int teller, struct mutex *mutex){
	struct list_head *pos, *n;
	struct can_message *tmp = NULL;

	if(DEBUG) printk("MCP2515: transmit_buffer[0x%x].msg = 0x%x", teller, transmit_buffer[teller].msg);
	kfree(transmit_buffer[teller].msg);
	
	mutex_lock(mutex);
	list_for_each_safe(pos, n, &(out_message_queue.list)){
		tmp = list_entry(pos, struct can_message, list);
		list_del(pos);
		break;
	}
	mutex_unlock(mutex);

	transmit_buffer[teller].msg = tmp;
	if(! tmp) return 0;

	if(DEBUG) printk(KERN_INFO "MCP2515: Interrupt sending message from the out_message_queue\n");
	
	switch(teller){
		case 0: mcp2515_load_frame_0(spidata.spidev, tmp->id, tmp->data, tmp->data_len);
				mcp2515_send_0(spidata.spidev);
				break;
		case 1: mcp2515_load_frame_1(spidata.spidev, tmp->id, tmp->data, tmp->data_len);
				mcp2515_send_1(spidata.spidev);
				break;
		case 2: mcp2515_load_frame_2(spidata.spidev, tmp->id, tmp->data, tmp->data_len);
				mcp2515_send_2(spidata.spidev);
				break;
	}
	return 1;
}

static irqreturn_t buffer1_intterupt(int irq, void *mutex){
	__u8 intf, iflags = 0;
	__u16 id;
	__u8 length;
	__u8 data[CAN_FRAME_MAX_DATA_LEN];
	irqreturn_t ret;
	//__u8 v1, v2;
	struct mutex *int_mutex = (struct mutex*) mutex;

	mutex_lock(int_mutex);
	if(DEBUG)
		printk(KERN_ALERT "MCP2515: GPIO IRQ %d event\n", interrupt_data.irq_number);
	/*
	// NAAR VOORBEELD VAN MCP251X
	mcp2515_read_2_registers(spidata.spidev, CANINTF, &v1, &v2);

	// mask out flags we don't care about
	v1 &= CANINTF_RX | CANINTF_TX | CANINTF_ERR;

	// Buffer 0
	if(v1 & CANINTF_RX0IF){
		;
	}
	*/
	//status = mcp2515_read_status(spidata.spidev);
	intf = mcp2515_read_register(spidata.spidev, CANINTF);

	if(DEBUG)
		printk(KERN_INFO "MCP2515: Status = 0x%02x\n", intf);

	// Borked or boring
	if(intf == 0x00){
		printk(KERN_ALERT "MCP2515: Empty Status\n");
		ret = IRQ_NONE;
		goto error;
	}
	if(intf == 0xFF){
		printk(KERN_ALERT "MCP2515: Faulty Status\n");
		ret = IRQ_NONE;
		goto error;
	}

	if(intf){
		if(intf & 0x01){
			// Packet in RxBuf0
			if(DEBUG)
				printk(KERN_INFO "MCP2515: Packet Frame in RxBuf0\n"); 
			mcp2515_read_frame_0(spidata.spidev, &id, &data, &length);
			add_message(&(in_message_queue.list), &in_message_mutex, id, &data, length);
		}
		if(intf & 0x02){
			// Packet in RxBuf1
			if(DEBUG)
				printk(KERN_INFO "MCP2515: Packet Frame in RxBuf1\n");
			mcp2515_read_frame_1(spidata.spidev, &id, &data, &length);
			add_message(&(in_message_queue.list), &in_message_mutex, id, &data, length);
		}
		if(intf & 0x04){
			// Packet Sent from TxBuf0
			if(DEBUG)
				printk(KERN_INFO "MCP2515: Packet Sent from TxBuf0\n");
			check_queue_send_message(0, &out_message_mutex);
			mcp2515_write_registerbit(spidata.spidev, CANINTF, 0x04, 0x00);
		}
		if(intf & 0x08){
			// Packet Sent from TxBuf1
			if(DEBUG)
				printk(KERN_INFO "MCP2515: Packet Sent from TxBuf1\n");
			check_queue_send_message(1, &out_message_mutex);
			mcp2515_write_registerbit(spidata.spidev, CANINTF, 0x08, 0x00);
		}
		if(intf & 0x10){
			// Packet Sent from TxBuf2
			if(DEBUG)
				printk(KERN_INFO "MCP2515: Packet Sent from TxBuf2\n");
			check_queue_send_message(2, &out_message_mutex);
			mcp2515_write_registerbit(spidata.spidev, CANINTF, 0x10, 0x00);
		}
		ret = IRQ_HANDLED;
	}

error:
	mutex_unlock(int_mutex);
	return IRQ_HANDLED;
}

static int __init init(void){
	dev_t dev = 0;
	int result;

	int i,j;

	if(DEBUG)
		printk(KERN_INFO "MCP2515: Init module\n");

	if( initSPI() ){
		result = -ENODEV;
		goto error;
	}
		
	// Registering device number
	if (scull_major) { 
		// Static major - minor (insmod with parameters)
		dev = MKDEV(scull_major, scull_minor);
		result = register_chrdev_region(dev, 1, MODULE_NAME);
	}else{
		// Dynamic major - minor (default)
		result = alloc_chrdev_region(&dev, scull_minor, 1, MODULE_NAME);
		scull_major = MAJOR(dev);
	}
	if (result < 0 ) {
		printk(KERN_ALERT "MCP2515: Can't get MAJOR(dev) %d\n", result);
		goto error;
	}else if(DEBUG)
		printk(KERN_INFO "MCP2515: Succeeded MAJOR(dev) = %d\n", result);

	// Initialise CharacterDEV (CDEV) structure
	device.cdev = cdev_alloc();
	device.cdev->owner = THIS_MODULE;
	device.cdev->ops   = &mcp2515_fops;
	result = cdev_add(device.cdev, dev, 1);
	if(result){
		printk(KERN_ALERT "MCP2515: Error cdev_add() = %d\n", result);
		goto error;
	}else if(DEBUG)
		printk(KERN_INFO "MCP2515: cdev_add() succeeded.\n");

	// ----------------------
	printk(KERN_INFO "MCP2515: setting baud\n");
	MCP2515_set_CANBAUD(CAN_BAUD_500K);

	__u8 test;
	if(LOOPBACK){
		printk(KERN_INFO "MCP2515: setting to loopback mode\n");
		test = MCP2515_LOOPBACK;
	}else{
		printk(KERN_INFO "MCP2515: setting to normal mode\n");
		test = MCP2515_NORMAL;
	}
	mcp2515_set_CAN_mode(spidata.spidev, test); 
	// -------------

	// Set up messageQueue
		// init List
	INIT_LIST_HEAD(&(in_message_queue.list));
	INIT_LIST_HEAD(&(out_message_queue.list));
		// Init Mutex
	mutex_init(&in_message_mutex);
	mutex_init(&interrupt_mutex);
	mutex_init(&out_message_mutex);

	// Enable interrupt
	mcp2515_write_register(spidata.spidev, CANINTE, (CANINTE_RX0IE & CANINTE_RX1IE & CANINTE_TX0IE & CANINTE_TX1IE & CANINTE_TX2IE));
	
	// Set up interrupt
	interrupt_data.irq_number = gpio_to_irq(25);

	result = request_threaded_irq(interrupt_data.irq_number, 
		NULL, buffer1_intterupt, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
		MODULE_NAME, &interrupt_mutex);

	if(result){
		dev_err(spidata.spidev, "MCP2515: Failed to acquire irq %d\n", interrupt_data.irq_number);
		goto error;
	}
	/*mutex_lock(&interrupt_mutex);
	// ------------------------
	
	printk(KERN_INFO "MCP2515: loading id and frame into txbuf 0\n");
	// mcp2515_load_0(spidata.spidev, 0x64, 0x34);
	__u8 data[8] = {'1','2','3','4','5','6','7','8'};
	mcp2515_load_frame_0(spidata.spidev, 0x0140, data, 8);

	printk(KERN_INFO "MCP2515: sending txbuf 0\n");
	mcp2515_send_0(spidata.spidev);

	printk(KERN_INFO "MCP2515: loading id and frame into txbuf 1\n");
	__u8 data2[8] = {'8','7','6','5','4','3','2','1'};
	mcp2515_load_frame_1(spidata.spidev, 0x0140, data2, 8);

	printk(KERN_INFO "MCP2515: sending txbuf 1\n");
	mcp2515_send_1(spidata.spidev);
	msleep(250);

	// ----------------------
	mutex_unlock(&interrupt_mutex);

	buffer1_intterupt(0, &interrupt_mutex);
	*/
	//buffer1_intterupt(0, &interrupt_mutex);
	/*
	__u8 data2[9] = {0};
	__u16 id = 0;
	__u8 length = 0;
	

	mcp2515_read_frame_1(spidata.spidev, &id, data2, &length);
	printk(KERN_INFO "MCP2515: Received ' %s ' ( 0x%x bytes ) from ID = 0x%x\n", data2, length, id);
	*//* i = mcp2515_read_id_1(spidata.spidev);
	printk(KERN_INFO "MCP2515: Receive ID rx buf 1 = %x\n", i);
	j = mcp2515_read_data_1(spidata.spidev);
	printk(KERN_INFO "MCP2515: Receive DATA rx buf 1 = %x\n", j);

	i = mcp2515_read_id_0(spidata.spidev);
	printk(KERN_INFO "MCP2515: Receive ID rx buf 0 = %x\n", i);
	j = mcp2515_read_data_0(spidata.spidev);
	printk(KERN_INFO "MCP2515: Receive DATA rx buf 0 = %x\n", j);
	*/

	return 0;

error:
	printk(KERN_ALERT "MCP2515: Aborting init\n");
	return result;
}

void exit(void){
	printk(KERN_INFO "MCP2515: Remove module spi_mcp2515\n");

	exitSPI();

	// Deleting Character Device (CDEV)
	cdev_del(device.cdev);
	// Unregistering device number
	dev_t devno = MKDEV(scull_major, scull_minor);
	unregister_chrdev_region(devno, 1);

	return;
}

module_init(init);
module_exit(exit);