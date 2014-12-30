#include <linux/module.h>   
#include <linux/init.h>		
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/sc16is750_i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fcntl.h>
#include <linux/i2c.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <asm/current.h>
#include <linux/string.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

static int irq;

struct task_struct *intr_thread, *current_process;

struct driver_data
{
	struct semaphore i2c_lock;
	struct cdev *i2c_cdev;
	bool s_flag;
	int READ_RX_FLAG;
	int WRITE_TX_FLAG;
	int READ_COUNT_FLAG;
	int WRITE_COUNT_FLAG;
	int READ_COUNT;
	int WRITE_COUNT;
	struct i2c_client *dev_client;
};

struct driver_data *driverdata;

char *received_data, *sent_data;
char *k_buffer;

/***************************** FUNCTION DECLARATIONS **************************************/
int allocatebuffer_mem(void);
void freebuffer_mem(void);

//Interrupt handled via a thread by this function
int handle_interrupt(void *data);
//Interrupt handler to trigger threading
irqreturn_t executethread(int irq, void *dev_id);


dev_t createdev(int, int);

static int __init i2c_sc16is750_init(void);
static void __exit i2c_sc16is750_exit(void);

static int i2c_irda_open(struct inode *inode, struct file *filp);
static int i2c_irda_release(struct inode *inode, struct file *filp);

//static long i2c_irda_ioctl(struct file *filp,unsigned int cmd, unsigned long arg);
static ssize_t i2c_irda_read_data(struct file *filp, char * ch, size_t count, loff_t *offp);
static ssize_t i2c_irda_read_register(struct file *filp, char * ch, size_t count, loff_t *offp);
static ssize_t i2c_irda_read_size(struct file *filp, char * ch, size_t count, loff_t *offp);
static ssize_t i2c_irda_write(struct file *filp, const char * ch, size_t count, loff_t *offp);
static int sc16is750_probe(struct i2c_client *, const struct i2c_device_id *);
static int sc16is750_remove(struct i2c_client *);

static const struct i2c_device_id sc16is750_id[] =
{
	{
		"sc16is750_i2c", 0
	},
	{ }
};

dev_t createdev(int MAJOR, int MINOR)
{
	dev_t dev;
	dev = MKDEV(MAJOR, MINOR);
	return dev;
}



static struct i2c_driver i2cdev_driver =
{
	.driver =
		{
                     .owner	= THIS_MODULE,
			.name	= "sc16is750_i2c",
		},
	.probe = sc16is750_probe,
	.remove = sc16is750_remove,
	.id_table = sc16is750_id,
};

static struct file_operations i2c_fops =
{
	owner: THIS_MODULE,
	open: i2c_irda_open,
	release: i2c_irda_release,
	read: i2c_irda_read_data,
	write: i2c_irda_write,
};

int allocatebuffer_mem()
{
	k_buffer = kmalloc(MAX_FIFO, GFP_KERNEL);
	if (k_buffer==NULL)
		return -ENOMEM;
	/*2KB of buffer to save the received I2C data from the slave. Avoid an excess size of the buffer due to possible
	  limitation of the memory */
	received_data = kzalloc(MAX_RECEIVED, GFP_KERNEL);
	if(received_data == NULL)
		return -ENOMEM;

	/*2KB of buffer to copy the data from user to kernel space when data is being transmitted by the master (sc6is750).
	 The buffer size can always to changed but care should be taken with the size of data transmitted at once.*/
	sent_data = kzalloc(MAX_SENT, GFP_KERNEL);
	if(sent_data == NULL)
		return -ENOMEM;

	driverdata = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if(driverdata == NULL)
		return -ENOMEM;

	return 0;
}

void freebuffer_mem()
{
	kfree(k_buffer);
	kfree (sent_data);
	kfree (received_data);
}

/* The function is invoked when ever 'intr_thread' is waken up, at the time of interrupt arrival. 
   It reads out the Interrupt Identification Register (IIR) to identify the interrupt source. It
   handles 5 different interrupts. 
   @Line Status Interrupt 	: On interrupt due to any error in received UART frame, the actual error
                          	  condition is found by LSR and the user is informed.
   @Receive Holding Interrupt	:
   @Receiver Time-Out Interrupt	: Both these interrupts suggests the arrival of a new data in the Rx
                                  FIFO. The data is read out and is stored in a buffer of size 2KB (2048)
                                  characater. If there is an overflow, the previous data is all lost and the
				  new data is stored at the start location. So it is adviced to read out the 
                                  buffer before any overflow occurs. See ioctl() and read() command.
   @Transmit Holding Interrupt	: THR interrupt is used to send the data to Tx FIFO of sc16is750. The data is 
                                  sent in chunks of MAX_TX_FIFO. 
   @Input pin (GPIO) Interrupt	: If the interrupt is enabled for the acknowledgement of change of state of an
                                  input pin, then this interrupt will occur. It informs the user about the present
                                  state of all pins.
*/

int handle_interrupt(void *data)
{
	u_char buf[2], iir_value[2];
	int size, i;

	driverdata->READ_COUNT = 0;

	while(1){

		//Read out the Interrupt Identification Register
		*k_buffer = IIR;
		i2c_master_send(driverdata->dev_client,k_buffer,1);
		i2c_master_recv(driverdata->dev_client, iir_value, 1);
cases:
		//Receiver Line Status Interrupt, RHR interrupt and Receiver time-out interrupt
		if(GET_BIT(iir_value[0], 2))
		{
			down(&driverdata->i2c_lock);
			if(GET_BIT(iir_value[0], 1))
			{

				buf[0] = LSR;
				i2c_master_send(driverdata->dev_client, buf, 1);
				i2c_master_recv(driverdata->dev_client, buf, 1);
				if(GET_BIT(buf[0], 0))
					printk(KERN_ALERT "LSR Interrupt: No data in receiver\n");
				if(GET_BIT(buf[0], 1))
					printk(KERN_ALERT "LSR Interrupt: Overrun error\n");
				if(GET_BIT(buf[0], 2))
					printk(KERN_ALERT "LSR Interrupt: Parity error\n");
				if(GET_BIT(buf[0], 3))
					printk(KERN_ALERT "LSR Interrupt: Framing error\n");
				if(GET_BIT(buf[0], 4))
					printk(KERN_ALERT "LSR Interrupt: Break error\n");
				if(GET_BIT(buf[0], 5))
					printk(KERN_ALERT "LSR Interrupt: THR empty\n");
				if(GET_BIT(buf[0], 6))
					printk(KERN_ALERT "LSR Interrupt: THR and TSR empty\n");
			}

			buf[0] = RXLVL;
			i2c_master_send(driverdata->dev_client, buf, 1);
			i2c_master_recv(driverdata->dev_client, buf, 1);
			*k_buffer = RHR;
			i2c_master_send(driverdata->dev_client, k_buffer, 1);
			size = (int)buf[0];

			/* Received data in RHR is read one byte at a time due to code error of inbuilt I2C driver in reading
			   multiple bytes. There is a lost of one byte when muliple bytes are tried to read out.*/
			for( i = 0; i<size; i++)
		    		i2c_master_recv(driverdata->dev_client, k_buffer+i, 1);

			if((driverdata->READ_COUNT = driverdata->READ_COUNT + size) > MAX_RECEIVED)
			{
		    		printk(KERN_ALERT "Received buffer overflow... over writing now\n");
				/*After the end of the received_data buffer, the next incoming data clears and overwrites
				  the received_data buffer. So always read this buffer before it reaches 2KB mark*/
		    		driverdata->READ_COUNT = size;
				*received_data = '\0';
		         }
		    	strncat(received_data , k_buffer, size);	

			up(&driverdata->i2c_lock);
		}

		//Input pin change of state interrupt
		if((GET_BIT(iir_value[0], 4)) && (GET_BIT(iir_value[0], 5)))
		{
			buf[0] = IOState;
			i2c_master_send(driverdata->dev_client, buf, 1);
			i2c_master_recv(driverdata->dev_client, buf, 1);
			printk(KERN_INFO "Input pin change of state. State of 8 I/O pins (in hex) %x\n", buf[0]);
		}

		*k_buffer = IIR;
		i2c_master_send(driverdata->dev_client,k_buffer,1);
		i2c_master_recv(driverdata->dev_client, iir_value, 1);
		//Put this thread to sleep if there is no interrupt
		if(iir_value[0]& 0x01)
		{
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule();
		}
		else
			goto cases;
	}

	return 0;
}

/* The sc16is750 is initialized when the driver is opened. Initialization sets the driver 
   ready to start the data tranmission.
*/

static int i2c_irda_open(struct inode *inode, struct file *filp)
{
	u_char write_to_reg[2];
	
/* The different configuration registers are set here. By default the FIFO is enabled and the UART data frame is set to 8N1
   format. Also the Tx/Rx is interrupt driven where interrupt occurs according to the trigger level set in TLR. By default,
   the hardware flow control is not set.
   Note: Please do not try to modify anyy of the register values directly from user end. The driver takes care of them and any 
   change may result in some serious errors*/

	//Set LCR[7] = 1 to allow access of DLL/DLH and set DLL = 0x0E (UART baud rate : 115200 by default)
	write_to_reg[0] = LCR;
	write_to_reg[1] = SET_BIT(7);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;
	
	write_to_reg[0] = DLL;
	write_to_reg[1] = SET_BIT(1)|SET_BIT(2)|SET_BIT(3);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = DLH;
	write_to_reg[1] = 0x0;
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//Setting EFR[4] = 1 and MCR[6] = 1 to enable use of enchanced functions
	write_to_reg[0] = LCR;
	write_to_reg[1] = 0xBF; //LCR should be 0xBF to allow access of EFR register
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = EFR;
	write_to_reg[1] = SET_BIT(4);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//LCR[7] is set to '0' and UART data frame is set to 8N1 (default)
	write_to_reg[0] = LCR;
	write_to_reg[1] = SET_BIT(0) | SET_BIT(1); 
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = MCR;
	write_to_reg[1] = SET_BIT(6);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//To exploit the usefulness of FIFO, the Tx/Rx is done using interrupt with FIFO enabled.
	write_to_reg[0] = FCR;
	write_to_reg[1] = SET_BIT(0) | SET_BIT(1) | SET_BIT(2); //FIFO-Reset & Enabled
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//Enabling RHR//
	write_to_reg[0] = IER;
	write_to_reg[1] = SET_BIT(0) | SET_BIT(2);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	return 0;
}

/* When an interrupt occurs, the thread called 'intr_thread' is waken up. The thread then
   read out the interrupt identification register to decide which operation to perform.
   For details on the operations, see the 'handle_interrupt' function'  
*/

irqreturn_t executethread(int irq, void *dev_id)
{

	wake_up_process(intr_thread);
	return IRQ_HANDLED;
}

static int i2c_irda_release(struct inode *inode, struct file *filp)
{
       u_char write_to_reg[2];

       //Setting EFR[4] = 1  enable use of enchanced functions
	write_to_reg[0] = LCR;
	write_to_reg[1] = 0xBF; //LCR should be 0xBF to allow access of EFR register
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = EFR;
	write_to_reg[1] = SET_BIT(4);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = MCR;
	write_to_reg[1] = 0x0;
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//disabling RHR//
	write_to_reg[0] = IER;
	write_to_reg[1] = 0x0;
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	return 0;
}


/* The write() call will write to the registers of the sc16is750. In case the data has to be written
   to the Tx FIFO, the data is to copied into a buffer (limited by size MAX_SENT) and THR interrupt
   is enabled. This will wake up the 'intr_thread' and the data transfer will start in chunks of 
   MAX_TX_FIFO.
*/

static ssize_t i2c_irda_write(struct file *filp, const char  __user *ch, size_t count, loff_t *offp)
{
	u_char  write_to_reg[2];
	u_char  buffer[2];
	u_char txbuffer[MAX_TX_FIFO + 1];

	down(&driverdata->i2c_lock);
	if (count > MAX_SENT)
		count = MAX_SENT;
	if (copy_from_user(sent_data, ch,count))
		return -EFAULT;

	//Writing to THR  Registers

	driverdata->WRITE_COUNT = count;
	driverdata->WRITE_COUNT_FLAG = 0;

	write_to_reg[0] = EFCR;
       i2c_master_send(driverdata->dev_client, write_to_reg, 1);
       i2c_master_recv(driverdata->dev_client, write_to_reg, 1);
	write_to_reg[1] = write_to_reg[0] | SET_BIT(1); //set EFCR[1] = 1
	write_to_reg[0] = EFCR;
	i2c_master_send(driverdata->dev_client, write_to_reg, 2);

until_write:
	txbuffer[0] = THR;
	if ((driverdata->WRITE_COUNT <= MAX_TX_FIFO) && (driverdata->WRITE_COUNT != 0))
	{

	        printk("---------doing write data to THR\n");
		 strncpy(txbuffer + 1, sent_data + driverdata->WRITE_COUNT_FLAG , driverdata->WRITE_COUNT);
		 i2c_master_send(driverdata->dev_client, txbuffer , driverdata->WRITE_COUNT + 1);
		 driverdata->WRITE_COUNT_FLAG += driverdata->WRITE_COUNT;
		 driverdata->WRITE_COUNT = 0;

		 up(&driverdata->i2c_lock);

	 }
	 else if(driverdata->WRITE_COUNT > MAX_TX_FIFO)
	 {
		 strncpy(txbuffer + 1, sent_data + driverdata->WRITE_COUNT_FLAG , MAX_TX_FIFO);
		 driverdata->WRITE_COUNT_FLAG += MAX_TX_FIFO;
		 driverdata->WRITE_COUNT -= MAX_TX_FIFO;
		 i2c_master_send(driverdata->dev_client, txbuffer , MAX_TX_FIFO + 1);
		 goto until_write;
	 }
read_ISR:
	buffer[0] = LSR;
	i2c_master_send(driverdata->dev_client, buffer, 1);
	i2c_master_recv(driverdata->dev_client, buffer, 1);
       if(GET_BIT(buffer[0], 6))
       {
	      write_to_reg[0] = EFCR;
             i2c_master_send(driverdata->dev_client, write_to_reg, 1);
             i2c_master_recv(driverdata->dev_client, write_to_reg, 1);
	      write_to_reg[1] = write_to_reg[0]& CLEAR_BIT(1); //set EFCR[1] = 0  
	      write_to_reg[0] = EFCR;	
	      i2c_master_send(driverdata->dev_client, write_to_reg, 2);
       }
	else
		goto read_ISR;

	return 0;
}

static ssize_t i2c_irda_write_loop(const char  __user *ch, size_t count)
{
	u_char txbuffer[MAX_TX_FIFO + 1];
	if (count > MAX_SENT)
		count = MAX_SENT;

	strncpy(sent_data, ch, count);

	//Writing to THR  Registers

	driverdata->WRITE_COUNT = count-1;
	driverdata->WRITE_COUNT_FLAG = 0;

until_write:
	txbuffer[0] = THR;
	if ((driverdata->WRITE_COUNT <= MAX_TX_FIFO) && (driverdata->WRITE_COUNT != 0))
	{

		 strncpy(txbuffer + 1, sent_data + driverdata->WRITE_COUNT_FLAG , driverdata->WRITE_COUNT);
		 i2c_master_send(driverdata->dev_client, txbuffer , driverdata->WRITE_COUNT + 1);
		 driverdata->WRITE_COUNT_FLAG += driverdata->WRITE_COUNT;
		 driverdata->WRITE_COUNT = 0;

	 }
	 else if(driverdata->WRITE_COUNT > MAX_TX_FIFO)
	 {
		 strncpy(txbuffer + 1, sent_data + driverdata->WRITE_COUNT_FLAG , MAX_TX_FIFO);
		 driverdata->WRITE_COUNT_FLAG += MAX_TX_FIFO;
		 driverdata->WRITE_COUNT -= MAX_TX_FIFO;
		 i2c_master_send(driverdata->dev_client, txbuffer , MAX_TX_FIFO + 1);
		 goto until_write;
	 }

	return count;
}


/* There are three different read operations. 1) Reads out the receievd data
   stored in a buffer (size limited to MAX_RECEIVED). 2) Reads out the register value of sc16is750
   3) Reads out the size of the data received by the driver. 
*/

static ssize_t i2c_irda_read_data(struct file *filp, char * ch, size_t count, loff_t *offp)
{
	int ret;
	
	if(!count)
	{
		ret = copy_to_user(ch,received_data,driverdata->READ_COUNT);
		if(!ret)
			return -EFAULT;
		*received_data = '\0';
		driverdata->READ_COUNT = 0;
		driverdata->READ_RX_FLAG = 0;
	}
	else
	{
		if(count > driverdata->READ_COUNT)
			return -ENOTTY;
		ret = copy_to_user(ch,received_data, count);
		if(!ret)
			return -EFAULT;
		strncpy(received_data, received_data + count, driverdata->READ_COUNT - count);
		*(received_data +  driverdata->READ_COUNT - count) = '\0';
		driverdata->READ_COUNT = driverdata->READ_COUNT - count;
		driverdata->READ_RX_FLAG = 0;
	}

	return ret;
}

static ssize_t i2c_irda_read_register(struct file *filp, char  __user *ch, size_t count, loff_t *offp)
{
       int ret;
       if (count > MAX_FIFO)
		count = MAX_FIFO;
	ret = i2c_master_recv(driverdata->dev_client, k_buffer,count);
	if (ret >= 0)
	{
		ret = copy_to_user(ch,k_buffer,count);
		if(ret)
			return -EFAULT;
	}
      return 0;
}

static ssize_t i2c_irda_read_size(struct file *filp, char  __user *ch, size_t count, loff_t *offp)
{
             *ch = driverdata->READ_COUNT % 100;
		ch++;
		*ch = (driverdata->READ_COUNT - (driverdata->READ_COUNT % 100))/100;
		driverdata->READ_COUNT_FLAG = 0;
		return 0;
}

static ssize_t attr_test_sc16is750_open(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t count;
	u_char write_to_reg[2];

	count = sprintf(buf, "--->Lintar i2c_open");
/* The different configuration registers are set here. By default the FIFO is enabled and the UART data frame is set to 8N1
   format. Also the Tx/Rx is interrupt driven where interrupt occurs according to the trigger level set in TLR. By default,
   the hardware flow control is not set.
   Note: Please do not try to modify anyy of the register values directly from user end. The driver takes care of them and any 
   change may result in some serious errors*/

	//Set LCR[7] = 1 to allow access of DLL/DLH and set DLL = 0x0E (UART baud rate : 115200 by default)
	write_to_reg[0] = LCR;
	write_to_reg[1] = SET_BIT(7);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = DLL;
	write_to_reg[1] = SET_BIT(1)|SET_BIT(2)|SET_BIT(3);
       //write_to_reg[1] = SET_BIT(0)|SET_BIT(3)|SET_BIT(5)|SET_BIT(7);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = DLH;
	write_to_reg[1] = 0x0;
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//Setting EFR[4] = 1 and MCR[2] = 1 to enable use of enchanced functions
	write_to_reg[0] = LCR;
	write_to_reg[1] = 0xBF; //LCR should be 0xBF to allow access of EFR register
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = EFR;
	write_to_reg[1] = SET_BIT(4);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//LCR[7] is set to '0' and UART data frame is set to 8N1 (default)
	write_to_reg[0] = LCR;
	write_to_reg[1] = SET_BIT(0) | SET_BIT(1);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	write_to_reg[0] = MCR;
	write_to_reg[1] = SET_BIT(6);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//To exploit the usefulness of FIFO, the Tx/Rx is done using interrupt with FIFO enabled.
	write_to_reg[0] = FCR;
	write_to_reg[1] = SET_BIT(0) | SET_BIT(1) | SET_BIT(2); //FIFO-Reset & Enabled
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;

	//Enabling RHR and Receive Line Status interrupt//
	write_to_reg[0] = IER;
	write_to_reg[1] = SET_BIT(0) | SET_BIT(2);
	if (i2c_master_send(driverdata->dev_client, write_to_reg, 2) != 2)
		return -EIO;
	return count;
}

static ssize_t attr_test_sc16is750_write(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
       u_char  write_to_reg[2];
	u_char  buffer[2];
	u_char txbuffer[MAX_TX_FIFO + 1];
	ssize_t count;
	count = sprintf(buf, "--->Lintar  sc16is750_write");

       printk("--->Lintar sc16is750_write\n");
       down(&driverdata->i2c_lock);

	sent_data[0] = 'a';
	sent_data[1] = 0x65;
	sent_data[2] = 0x35;
	sent_data[3] = 0x99;

	driverdata->WRITE_COUNT = 4;
	driverdata->WRITE_COUNT_FLAG = 0;


       write_to_reg[0] = EFCR;
       i2c_master_send(driverdata->dev_client, write_to_reg, 1);
       i2c_master_recv(driverdata->dev_client, write_to_reg, 1);
	write_to_reg[1] = write_to_reg[0] | SET_BIT(1); //set EFCR[1] = 1 
	write_to_reg[0] = EFCR;
	i2c_master_send(driverdata->dev_client, write_to_reg, 2);


until_write:
	txbuffer[0] = THR;
	if ((driverdata->WRITE_COUNT <= MAX_TX_FIFO) && (driverdata->WRITE_COUNT != 0))
	{

	       printk("---------doing write data to THR\n");
		strncpy(txbuffer + 1, sent_data + driverdata->WRITE_COUNT_FLAG , driverdata->WRITE_COUNT);
		i2c_master_send(driverdata->dev_client, txbuffer , driverdata->WRITE_COUNT + 1);
		driverdata->WRITE_COUNT_FLAG += driverdata->WRITE_COUNT;
		driverdata->WRITE_COUNT = 0;

		printk("txbuffer = %x, %x, %x, %x, %x \n", txbuffer[0], txbuffer[1],txbuffer[2],txbuffer[3],txbuffer[4]);

		up(&driverdata->i2c_lock);

	}
	else if(driverdata->WRITE_COUNT > MAX_TX_FIFO)
	{
		strncpy(txbuffer + 1, sent_data + driverdata->WRITE_COUNT_FLAG , MAX_TX_FIFO);
		driverdata->WRITE_COUNT_FLAG += MAX_TX_FIFO;
		driverdata->WRITE_COUNT -= MAX_TX_FIFO;
		i2c_master_send(driverdata->dev_client, txbuffer , MAX_TX_FIFO + 1);
		goto until_write;
	}


read_ISR:
	buffer[0] = LSR;
	i2c_master_send(driverdata->dev_client, buffer, 1);
	i2c_master_recv(driverdata->dev_client, buffer, 1);
       if(GET_BIT(buffer[0], 6))
       {
	      write_to_reg[0] = EFCR;
             i2c_master_send(driverdata->dev_client, write_to_reg, 1);
             i2c_master_recv(driverdata->dev_client, write_to_reg, 1);
	      write_to_reg[1] = write_to_reg[0]& CLEAR_BIT(1); //set EFCR[1] = 0
	      write_to_reg[0] = EFCR;
	      i2c_master_send(driverdata->dev_client, write_to_reg, 2);
       }
	else
		goto read_ISR;

     return count;
}

static ssize_t attr_irda_mmi_write(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
        int ret;
	 struct inode *inode = NULL;
        struct file *filp = NULL;
	 loff_t *offp = NULL;

	 ret = i2c_irda_open(inode, filp);
	 if(ret){
               printk("%s: Failed to open irda\n", __func__);
               return ret;
	 }
        printk("%s:write %s size:%d\n",__func__,buf,strlen(buf)+1);
        ret = i2c_irda_write_loop(buf, strlen(buf)+1);
	 if(!ret){
               printk("%s: Failed to write data to irda\n", __func__);
               return ret;
	 }
	 return ret;
}

static ssize_t attr_irda_mmi_read(struct device *dev, struct device_attribute *attr,
				char *buf)
{
        int ret, ret0;
	 struct inode *inode = NULL;
        struct file *filp = NULL;
	 loff_t *offp = NULL;

         printk("%s:read %s\n",__func__,received_data);
	 ret = sprintf(buf, "%s\n", received_data);
	 memset(received_data,0,strlen(received_data)); 
	
	 ret0 = i2c_irda_release(inode, filp);
	 if(ret0){
               printk("%s: Failed to release irda\n", __func__);
               return ret0;
	 }
	 return ret;
}


static struct device_attribute irda_attr[] ={

	__ATTR(sc16is750_open_test, 0664, NULL, attr_test_sc16is750_open),

	__ATTR(sc16is750_write_test, 0664,NULL,attr_test_sc16is750_write),

	__ATTR(irda_mmi_test, 0664,attr_irda_mmi_read,attr_irda_mmi_write),

};

static int create_sysfs_interfaces(struct device *dev)
{
       int i;
       for (i = 0; i < ARRAY_SIZE(irda_attr); i++)
	          if (device_create_file(dev, irda_attr + i))
		       goto error;
	          return 0;
error:
	for ( ; i >= 0; i--)
		   device_remove_file(dev, irda_attr + i);
       dev_err(dev, "%s:Unable to create interface\n", __func__);
       return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
       int i;
       for (i = 0; i < ARRAY_SIZE(irda_attr); i++)
	           device_remove_file(dev, irda_attr + i);
       return 0;
}

static int sc16is750_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
       int err = 0;

	driverdata->dev_client = client;

      err = create_sysfs_interfaces(&client->dev);
      if (err < 0) {
	  	    dev_err(&client->dev, " irda device  sysfs register failed\n");
		    return err;
	}

	struct irda_device_platorm_data *pdata = client->dev.platform_data;

       if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	/*enable power config*/
	  ret = gpio_request(IRDA_2V85EN, "pw_enable");
        if (ret) {
            pr_err("%s: Failed to get pw_enable gpio %d. Code: %d.",
                   __func__, IRDA_2V85EN, ret);
            return ret;
        }
	 ret = gpio_direction_output(IRDA_2V85EN,1);
        if (ret) {
            pr_err("%s: Failed to setup pw_enable_gpio %d. Code: %d.",
                   __func__, IRDA_2V85EN, ret);
            gpio_free(IRDA_2V85EN);
        }

	/*irda reset*/
	if (pdata->gpio_config) {
		dev_info(&client->dev, "Configuring GPIOs.\n");
		ret = pdata->gpio_config(pdata->gpio_data, true);
		if (ret< 0) {
			dev_err(&client->dev, "Failed to configure GPIOs, code: %d.\n",ret);
			return ret;
		}
		dev_info(&client->dev, "Done with GPIO configuration.\n");
	}

	sema_init(&driverdata->i2c_lock,1);

	/*Driver data initialization */
	driverdata->READ_COUNT = 0;
	driverdata->READ_RX_FLAG = 0;
	driverdata->WRITE_TX_FLAG = 0;
	driverdata->READ_COUNT_FLAG = 0;

	//If all successful, create a thread to control the sc16is750 interrupt
	intr_thread = kthread_create(handle_interrupt, NULL, "IntrThread");
	intr_thread->state = TASK_UNINTERRUPTIBLE;

	//Requesting IRQ line for GPIO_022
	irq = gpio_to_irq(IRQ_GPIO_022);
	ret = request_irq(irq, executethread, IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING, "intr_handler", NULL);

	if(ret)
	{
		printk(KERN_ERR "Can not acquire interrupt line at GPIO_022\n");
		driverdata->s_flag = false;
		return ret;
	}

	printk(KERN_INFO "Interrupt line acquired successfully\n");


	return 0;
}


static int sc16is750_remove(struct i2c_client *client)
{
	free_irq(irq, NULL);
	remove_sysfs_interfaces(&client->dev);
	return 0;
}

static int __init i2c_sc16is750_init()
{
	int ret;
	dev_t dev;

	dev = createdev(I2C_MAJOR_NUM,I2C_MINOR_NUM);
	ret = register_chrdev_region(dev, 1, "sc750i2c");
	if(ret)
	{
		printk(KERN_ERR "Error allocating dev num\n");
		return ret;
	}
	printk(KERN_INFO "Dev num allocation...done\n");

	if(allocatebuffer_mem())
	{
		printk(KERN_ERR "Memory allocation for buffer failed\n");
		goto unregister;
	}
	driverdata->s_flag= true;
	driverdata->i2c_cdev = cdev_alloc();
	driverdata->i2c_cdev->owner = THIS_MODULE;
	driverdata->i2c_cdev->ops = &i2c_fops;
	if((ret = cdev_add(driverdata->i2c_cdev, dev, 1)))
	{
		printk(KERN_ERR "Device registration failure\n");
		goto free_mem;
	}

	printk(KERN_INFO "Device registration...done\n");

	if((ret = i2c_add_driver(&i2cdev_driver)))
		goto dev_del;

	if(driverdata->s_flag == false)
	{
		ret = -1;
		goto remove_driver;
	}
	printk("init successfully\n");

	return 0;

remove_driver:
	i2c_del_driver(&i2cdev_driver);
dev_del:
	cdev_del(driverdata->i2c_cdev);
free_mem:
	freebuffer_mem();
unregister:
	unregister_chrdev_region(createdev(I2C_MAJOR_NUM, I2C_MINOR_NUM), 1);
	return ret;
}


static void __exit i2c_sc16is750_exit()
{
	i2c_del_driver(&i2cdev_driver);
	freebuffer_mem();
	cdev_del(driverdata->i2c_cdev);
	printk (KERN_INFO "Unregistering the device...\n");
	unregister_chrdev_region(createdev(I2C_MAJOR_NUM, I2C_MINOR_NUM), 1);
	printk (KERN_INFO "Deallocating the dev no..\n");
}

module_init(i2c_sc16is750_init);
module_exit(i2c_sc16is750_exit);

/****************************** MODULE DESCRIPTION ****************************************/

MODULE_AUTHOR("Malay Jajodia - Nxp Semiconductors");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C-UART Bridge driver for SC16IS750");

