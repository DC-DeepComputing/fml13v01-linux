/*
 * USB Skeleton driver - 2.2
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * but has been rewritten to be easier to read and use.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>


/* Define these values to match your devices */
#define USB_SKEL_VENDOR_ID	0x174c
#define USB_SKEL_VENDOR_ID_OEM	0x1c05
#define USB_SKEL_PRODUCT_ID	0x2074
#define USB_SKEL_PRODUCT_ID2	0x2084
#define USB_SKEL_PRODUCT_ID3	0x3412
/* table of devices that work with this driver */
static const struct usb_device_id skel_table[] = {
	{ USB_DEVICE(USB_SKEL_VENDOR_ID, USB_SKEL_PRODUCT_ID) },
	{ USB_DEVICE(USB_SKEL_VENDOR_ID, USB_SKEL_PRODUCT_ID2) },	
	{ USB_DEVICE(USB_SKEL_VENDOR_ID, USB_SKEL_PRODUCT_ID3) },
	{ USB_DEVICE(USB_SKEL_VENDOR_ID_OEM, USB_SKEL_PRODUCT_ID) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, skel_table);


/* Get a minor range for your devices from the usb maintainer */
#define USB_SKEL_MINOR_BASE	192

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */
#define WRITES_IN_FLIGHT	8
/* arbitrarily chosen */

/* Structure to hold all of our device specific stuff */
struct usb_skel {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct urb		*bulk_in_urb;		/* the urb to read data with */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	size_t			bulk_in_copied;		/* already copied to user space */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	int			errors;			/* the last request tanked */
	int			open_count;		/* count the number of openers */
	bool			ongoing_read;		/* a read is going on */
	bool			processed_urb;		/* indicates we haven't processed the urb */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	struct completion	bulk_in_completion;	/* to wait for an ongoing read */
};
#define to_skel_dev(d) container_of(d, struct usb_skel, kref)

static struct usb_driver skel_driver;
static void skel_draw_down(struct usb_skel *dev);

static void skel_delete(struct kref *kref)
{
	struct usb_skel *dev = to_skel_dev(kref);

	usb_free_urb(dev->bulk_in_urb);
	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static int skel_open(struct inode *inode, struct file *file)
{
	struct usb_skel *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;


	printk(KERN_ALERT "Entering skel_open\n"); // 
	subminor = iminor(inode);

	interface = usb_find_interface(&skel_driver, subminor);
	if (!interface) {
		//err("%s - error, can't find device for minor %d",
		//     __func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* lock the device to allow correctly handling errors
	 * in resumption */
	mutex_lock(&dev->io_mutex);

	if (!dev->open_count++) {
		retval = usb_autopm_get_interface(interface);
			if (retval) {
				dev->open_count--;
				mutex_unlock(&dev->io_mutex);
				kref_put(&dev->kref, skel_delete);
				goto exit;
			}
	} /* else { //uncomment this block if you want exclusive open
		retval = -EBUSY;
		dev->open_count--;
		mutex_unlock(&dev->io_mutex);
		kref_put(&dev->kref, skel_delete);
		goto exit;
	} */
	/* prevent the device from being autosuspended */

	/* save our object in the file's private structure */
	file->private_data = dev;
	mutex_unlock(&dev->io_mutex);

exit:
	//printk(KERN_ALERT "Exit skel_open:%d \n",retval); // 

	return retval;
}

static int skel_release(struct inode *inode, struct file *file)
{
	struct usb_skel *dev;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (!--dev->open_count && dev->interface)
		usb_autopm_put_interface(dev->interface);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, skel_delete);
	return 0;
}


#define DATA_BUFF_LEN 32
#define CMD_BUFF_LEN	8

#define FW_WRITE_CMD  	0x50
#define CFG_WRITE_CMD  	0x51
#define REG_Read	0x52
#define REG_Write	0x53
#define CFG_READ_CMD	0x55
#define RESET_CPU_CMD	0x54
#define SPI_ERASE_BLK	0x57



static unsigned char SetupPacket[CMD_BUFF_LEN];
static long skel_ioctrl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct usb_skel *dev;
	int retval;
	int VCMD,ICMD,LCMD;
	int debugC;

	unsigned char *user_buffer,*CodeBuff;	

	printk("\n\nEnter skel_ioctrl !!\n");

	CodeBuff = kmalloc(DATA_BUFF_LEN, GFP_KERNEL);//32 Byte data
   
	user_buffer=(unsigned char *)arg;
	dev = (struct usb_skel *)file->private_data;


	copy_from_user(SetupPacket, user_buffer, CMD_BUFF_LEN);//copy command 

#if 1
	printk("\nSetupPacket:");
	printk("%2.2X ",SetupPacket[0]);
	printk("%2.2X ",SetupPacket[1]);
	printk("%2.2X ",SetupPacket[2]);
	printk("%2.2X ",SetupPacket[3]);
	printk("%2.2X ",SetupPacket[4]);
	printk("%2.2X ",SetupPacket[5]);
	printk("%2.2X ",SetupPacket[6]);
	printk("%2.2X ",SetupPacket[7]);
	printk("\n");
#endif


VCMD=(SetupPacket[3]<<8) + SetupPacket[2] ;
ICMD=(SetupPacket[5]<<8) + SetupPacket[4];
LCMD=(SetupPacket[7]<<8)+SetupPacket[6];

	printk("\n ==== AAAAAAAAAAAAAAAAAAAA ==== \n");
	printk("<0>" "\n ==== YYYYYYYYYYYYYYYYYYYY ==== \n");

	printk("\nSetupPacket:");
	printk("%04X ",VCMD);
	printk("%04X ",ICMD);
	printk("%04X ",LCMD);


	if(SetupPacket[1]==FW_WRITE_CMD || SetupPacket[1]==CFG_WRITE_CMD || SetupPacket[1]==RESET_CPU_CMD || SetupPacket[1]==SPI_ERASE_BLK || SetupPacket[1]==0x03)
	{
			copy_from_user(CodeBuff, (user_buffer+8),0x20);//copy FW 

			retval =  usb_control_msg(
					dev->udev, 	//@dev: pointer to the usb device to send the message to
					usb_sndctrlpipe(dev->udev, 0),	//@pipe: endpoint "pipe" to send the message to
					SetupPacket[1],	//@request: USB message request value
		    	SetupPacket[0],  //@requesttype: USB message request type value
		    	VCMD  ,  //@value: USB message value
		    	ICMD  , 	//@index: USB message index value
		    	CodeBuff,					//@data: pointer to the data to send
		    	LCMD, 	//@size: length in bytes of the data to send
		    	USB_CTRL_GET_TIMEOUT			//@timeout: time in msecs to wait for the message to complete before timing
		    	);

					printk("\n\nWRITE_CMD ret=%d\n",retval);
			#if 1
			printk("\nDATA:");
			for(debugC=0;debugC<32 ; debugC++)
			{
			
				printk("%02X",*(CodeBuff+debugC));

			}
			printk("\n");
			#endif



	}
	else	if (SetupPacket[1]==CFG_READ_CMD||SetupPacket[1]==REG_Read)
	{
			retval =  usb_control_msg(
					dev->udev, 	//@dev: pointer to the usb device to send the message to
					usb_rcvctrlpipe(dev->udev, 0),	//@pipe: endpoint "pipe" to send the message to
					SetupPacket[1],	//@request: USB message request value
		    	SetupPacket[0],  //@requesttype: USB message request type value
		    	VCMD  ,  //@value: USB message value
		    	ICMD  , 	//@index: USB message index value
		    	CodeBuff,					//@data: pointer to the data to send
		    	LCMD, 	//@size: length in bytes of the data to send
		    	USB_CTRL_GET_TIMEOUT			//@timeout: time in msecs to wait for the message to complete before timing
		    	);


				//copy_to_user((user_buffer+CMD_BUFF_LEN),CodeBuff, DATA_BUFF_LEN);
				copy_to_user((void __user *)arg+8,CodeBuff, DATA_BUFF_LEN);


				//printk("\n\nREAD ret=%d\n",retval);
			#if 1
			printk("\nDATA:");
			for(debugC=0;debugC<32 ; debugC++)
			{
			
				printk("%02X",*(CodeBuff+debugC));

			}
			printk("\n");
			#endif

	}
	else
	{
		retval=(-1);
	}

	return retval;
//return 0;
}


static const struct file_operations skel_fops = {
	.owner =	THIS_MODULE,
	//.read =		skel_read,
	//.write =	skel_write,
	.open =		skel_open,
	.release =	skel_release,
	//.flush =	skel_flush,
	//.unlocked_ioctl=skel_ioctl,//Joe 20130607
	.unlocked_ioctl = skel_ioctrl,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver skel_class = {
	.name =		"ASMT_HUB%d",
	.fops =		&skel_fops,
	.minor_base =	USB_SKEL_MINOR_BASE,
};

static int skel_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_skel *dev;
	//struct usb_host_interface *iface_desc;
	//struct usb_endpoint_descriptor *endpoint;
	//size_t buffer_size;
	//int i;
	int retval = -ENOMEM;
	printk(KERN_ALERT "Entering skel_probe\n"); // 
	
	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		//err("Out of memory");
		goto error;
	}
 
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);
	init_completion(&dev->bulk_in_completion);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
#if 0
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr &&
		    usb_endpoint_is_bulk_in(endpoint)) {
			/* we found a bulk in endpoint */
			buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				//err("Could not allocate bulk_in_buffer");
				goto error;
			}
			dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!dev->bulk_in_urb) {
				//err("Could not allocate bulk_in_urb");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    usb_endpoint_is_bulk_out(endpoint)) {
			/* we found a bulk out endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}
 
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		//err("Could not find both bulk-in and bulk-out endpoints");
		goto error;
	}
 #endif
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &skel_class);
 
	if (retval) {
		/* something prevented us from registering this driver */
		//err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB Skeleton device now attached to USBSkel-%d",
		 interface->minor);

	//printk(KERN_ALERT "\nProbe exit\n");	



	return 0;

error:
	if (dev)
		/* this frees allocated memory */
		kref_put(&dev->kref, skel_delete);
	return retval;
}

static void skel_disconnect(struct usb_interface *interface)
{
	struct usb_skel *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &skel_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
	kref_put(&dev->kref, skel_delete);

	dev_info(&interface->dev, "USB Skeleton #%d now disconnected", minor);
}

static void skel_draw_down(struct usb_skel *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
	usb_kill_urb(dev->bulk_in_urb);
}

static int skel_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	skel_draw_down(dev);
	return 0;
}

static int skel_resume(struct usb_interface *intf)
{
	return 0;
}

static int skel_pre_reset(struct usb_interface *intf)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	skel_draw_down(dev);

	return 0;
}

static int skel_post_reset(struct usb_interface *intf)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}

static struct usb_driver skel_driver = {
	.name =		"skeleton",
	.probe =	skel_probe,
	.disconnect =	skel_disconnect,
	.suspend =	skel_suspend,
	.resume =	skel_resume,
	.pre_reset =	skel_pre_reset,
	.post_reset =	skel_post_reset,
	.id_table =	skel_table,
	.supports_autosuspend = 1,
};

static int __init usb_skel_init(void)
{
	int result;
	printk(KERN_ALERT "Entering myusb init\n"); // 加入usb_skel_init函數中
	/* register this driver with the USB subsystem */
	result = usb_register(&skel_driver);
	//if (result)
		//err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit usb_skel_exit(void)
{
	/* deregister this driver with the USB subsystem */
	printk(KERN_ALERT "Entering myusb exit\n"); // 加入usb_skel_exit函數中
	usb_deregister(&skel_driver);
}

module_init(usb_skel_init);
module_exit(usb_skel_exit);

MODULE_LICENSE("GPL");
