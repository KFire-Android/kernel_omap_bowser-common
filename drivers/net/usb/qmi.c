/*
 * QMI USB driver
 *
 * Copyright 2011  Lab126, Inc.  All rights reserved.
 *
 */
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/qmi.h>
#include <linux/wakelock.h>

#define MAX_RMNET_DEVS  (2)

static int qmi_rmnet_bind(struct usbnet *, struct usb_interface *);
static int qmi_remote_wakeup_init(struct qmi_private *qmi);
static int qmi_remote_wakeup_destroy(struct qmi_private *qmi);
static struct wake_lock	wlock;
static spinlock_t	wakeup_lock;
static int wakeup_irq_enabled = 0;

static int qmi_manage_power(struct usbnet *dev, int on)
{
	return 0;
}

static const struct driver_info	rmnet_info = {
	.bind =		qmi_rmnet_bind,
	.description =	"RmNet Ethernet Device",
	.manage_power = qmi_manage_power,
};

static struct usb_device_id id_table [] = {
	/* MDM9600 FFA - 1 RMNET */
	{ USB_DEVICE(HSU_QC_VENDOR_ID, HSU_QC_PRODUCT_9001),
		.driver_info = (unsigned long)&rmnet_info},
	/* MDM9600 FFA - 2 RMNET */
	{ USB_DEVICE_AND_INTERFACE_INFO(HSU_QC_VENDOR_ID,
		HSU_QC_PRODUCT_9011, 0xff, 0x06, 0x00),
		.driver_info = (unsigned long)&rmnet_info},
	/* ELMO */
	{ USB_DEVICE_AND_INTERFACE_INFO(LAB126_VENDOR_ID,
		ELMO_PRODUCT_ID, 0xff, 0x06, 0x00),
		.driver_info = (unsigned long)&rmnet_info},
	/* Grover */
	{ USB_DEVICE_AND_INTERFACE_INFO(LAB126_VENDOR_ID,
		GROVER_PRODUCT_ID, 0xff, 0x06, 0x00),
		.driver_info = (unsigned long)&rmnet_info},
	/* Ernie */
	{ USB_DEVICE_AND_INTERFACE_INFO(LAB126_VENDOR_ID,
		ERNIE_PRODUCT_ID, 0xff, 0x06, 0x00),
		.driver_info = (unsigned long)&rmnet_info},
	/* Pantech LTE usb dongle */
	{ USB_DEVICE_AND_INTERFACE_INFO(PANTECH_VENDOR_ID,
		PANTECH_PRODUCT_3718, 0xff, 0xf0, 0xff),
		.driver_info = (unsigned long)&rmnet_info },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct qmi_private *qmi_devs[MAX_RMNET_DEVS];
static int debug = 0;

static void print_hex(uint8_t *buf, int length)
{
#define LINE_LENGTH 16
	int i;
	for (i=0; i < length;) {
		printk ("0x%02x", buf[i++]);
		if ((i % LINE_LENGTH) == 0 || i == length) {
			printk ("\n");
		} else {
			printk (" ");
		}
	}
	printk ("\n");
}

/* Called in atomic context - can't use sync calls */
static void qmi_msg_timeout(unsigned long data)
{
	int cnt, i = 0;
	struct qmi_private *qmi = (struct qmi_private*) data;

	if (!qmi) {
		return;
	}

	cnt = atomic_read(&qmi->qmi_msg_cnt);
	for (i = 0; i < cnt; i++) {
		usb_autopm_put_interface_async(qmi->usb_intf);
	}
	atomic_set(&qmi->qmi_msg_cnt, 0);
}

/* Decrease the reference count on the USB interface */
static inline int qmi_autopm_put_interface_async(struct qmi_private *qmi)
{
	if (!qmi) {
		return 0;
	}

	atomic_dec(&qmi->qmi_msg_cnt);
	usb_autopm_put_interface_async(qmi->usb_intf);
	return 0;
}

/* Increase the reference count on the interface */
static inline int qmi_autopm_get_interface_async(struct qmi_private *qmi)
{
	int err = 0;

	if (!qmi) {
		return 0;
	}

	err = usb_autopm_get_interface_async(qmi->usb_intf);
	if (!err) {
		atomic_inc(&qmi->qmi_msg_cnt);
	}

	return err;
}

/* Should be called from process context */
static inline int qmi_autopm_put_interface(struct qmi_private *qmi)
{
	if (!qmi) {
		return 0;
	}

	atomic_dec(&qmi->qmi_msg_cnt);
	usb_autopm_put_interface(qmi->usb_intf);
	return 0;
}

static inline int qmi_autopm_get_interface(struct qmi_private *qmi)
{
	int err = 0;

	if (!qmi) {
		return 0;
	}

	err = usb_autopm_get_interface(qmi->usb_intf);
	if (!err) {
		atomic_inc(&qmi->qmi_msg_cnt);
	}

	return err;
}

static void do_get_encap_rsp(struct work_struct *dummy)
{
	struct usb_device *dev;
	struct qmi_private *qmi;
	int if_num;
	int rsp_len, err = 0;
	unsigned int ridx;
	unsigned int widx;

	qmi = container_of(dummy,struct qmi_private, get_encap_work);
	if_num = qmi->usb_intf->cur_altsetting->desc.bInterfaceNumber;
	dev = interface_to_usbdev(qmi->usb_intf);

	/*
	 * Runs in kthread context. Use sync call.
	 * Block until the bus is resumed.
	 */
	err = qmi_autopm_get_interface(qmi);
	if (err) {
		return;
	}

	if ((rsp_len = usb_control_msg( dev, usb_rcvctrlpipe(dev, 0),
		0x01, 0xa1, 0, if_num, qmi->encap_rsp_buf, PAGE_SIZE,
		USB_CTRL_SET_TIMEOUT)) > 0) {
		if (debug) print_hex(qmi->encap_rsp_buf,rsp_len);
		if (likely(kfifo_avail(&qmi->qmi_rcv_kfifo) >= rsp_len)) {
			ridx = qmi->rcv_pkt_ridx;
			widx = qmi->rcv_pkt_widx;
			if ((widx + 1) % MAX_RCV_PKT_INDX != ridx) {
				kfifo_in(&qmi->qmi_rcv_kfifo,
					qmi->encap_rsp_buf,rsp_len);
				qmi->rcv_pkt_sizes[widx] = rsp_len;
				qmi->rcv_pkt_widx = (widx + 1) % MAX_RCV_PKT_INDX;
				qmi->read_wake_cond = 1;
				wake_up(&qmi->read_wait_q);
				if (waitqueue_active(&qmi->read_poll_q))
					wake_up_interruptible(&qmi->read_poll_q);
			} else {
				pr_err("%s: Ran out of pkt descritors."
					"Dropping packet\n",__func__);
			}
		} else {
			pr_err("%s: Receive buffer overrun."
				"Dropping packet.\n",__func__);
		}
	} else {
		pr_err("%s: Get encap rsp timed out\n",__func__);
		pr_err("%s: usb_control_msg() timed out. err = %d\n",
			__func__, rsp_len);
	}

	if (!qmi->intr_urb->hcpriv) {
		err = usb_submit_urb(qmi->intr_urb, GFP_ATOMIC);

		if (err) {
			pr_err("%s: Re-submit intr URB failed. err = %d\n",
				__func__, err);
			goto done;
		}

		/*
		 * Don't reduce the reference count on the interface here.
		 * Start the timer. We will decrease the ref count if
		 * the timer expires.
		 */
		mod_timer(&qmi->qmi_timer, jiffies +
			msecs_to_jiffies(QMI_CONTROL_MSG_TIMEOUT_MS));
	}
	return;
done:
	qmi_autopm_put_interface(qmi);
}

static int qmi_rmnet_bind(struct usbnet *dev, struct usb_interface *udev)
{
	int status;
	strcpy(dev->net->name,"rmnet%d");
	status = usbnet_get_endpoints (dev, udev);

	if (status != 0)
		pr_err("%s: can not get usbnet ep\n",__func__);

	return status;
}

static int qmi_misc_open(struct inode *ino, struct file *filep)
{
	int err = 0;
	struct miscdevice *misc_dev = NULL;
	struct qmi_private *qmi = NULL;
	int i;

	for (i=0 ; i<MAX_RMNET_DEVS ; i++){
		if (qmi_devs[i]->qmi_misc_dev.minor == MINOR(ino->i_rdev)) {
			/*
			 * sanity check: earlier kernel did not set private_data.
			 * Do not proceed if private data is set and it's not
			 * miscdevice */
			if (filep->private_data && filep->private_data
				!= &qmi_devs[i]->qmi_misc_dev) {
				pr_err("%s: private data mismach %p - %p\n",
					__func__, filep->private_data,
					&qmi_devs[i]->qmi_misc_dev);
				return -EINVAL;
			}
			misc_dev = filep->private_data =
					&qmi_devs[i]->qmi_misc_dev;
			break;
		}
	}

	if (!misc_dev) {
		pr_err("%s: could not find qmi device\n",__func__);
		return -EINVAL;
	}
	qmi = container_of(misc_dev, struct qmi_private, qmi_misc_dev);

	/* only one process can open qmi device */
	if (atomic_cmpxchg(&qmi->opened, 0, 1) == 1) {
		/* it's already open */
		pr_err("%s: qmi device busy (%d)\n",
			__func__, atomic_read(&qmi->opened));
		return -EBUSY;
	}

	/* submit urb when usb is connected */
	if (atomic_read(&qmi->usb_connected)) {
		if (qmi->intr_urb->hcpriv) {
			return 0;
		}
		err = qmi_autopm_get_interface(qmi);
		if (err) {
			atomic_set(&qmi->opened, 0);
			return err;
		}
		err = usb_submit_urb(qmi->intr_urb, GFP_ATOMIC);
		if (err) {
			dbg("%s: resubmit intr urb failed. (%d)",
				__func__, err);
			atomic_set(&qmi->opened, 0);
		}
		qmi_autopm_put_interface(qmi);
	} else {
		pr_info("%s: usb not connected yet\n", __func__);
	}
	return err;
}

static int qmi_misc_release(struct inode *ino, struct file *filep)
{
	struct miscdevice *misc_dev;
	struct qmi_private *qmi;

	misc_dev = filep->private_data;
	qmi = container_of(misc_dev, struct qmi_private, qmi_misc_dev);

	/* kill urb only usb is still connected */
	if( qmi->qmi_intr_ep )
		usb_kill_urb(qmi->intr_urb);
	atomic_set(&qmi->opened,0);
	pr_info("%s: released qmi file (%d)\n", __func__,
		atomic_read(&qmi->opened));
	return 0;
}

static ssize_t qmi_misc_read(struct file *filep, char __user *buf, size_t count,
				loff_t *ppos)
{
	int ret = 0;
	unsigned int remain_size = count;
	unsigned int num_pkts = 0;
	unsigned int ridx;
	unsigned int widx;
	struct miscdevice *misc_dev;
	struct qmi_private *qmi;

	misc_dev = filep->private_data;
	qmi = container_of(misc_dev,struct qmi_private,qmi_misc_dev);

	/* check if usb is connected. */
	if (!qmi->qmi_intr_ep) {
		return -ENODEV;
	}

	if (unlikely(count == 0))
		return 0;

	if (!kfifo_len(&qmi->qmi_rcv_kfifo)) {
		/* No data to be read. check for non-blocking mode */
		if( filep->f_flags & O_NONBLOCK )
			return -EAGAIN;

		qmi->read_wake_cond = 0;
		dbg("Wait for data");
		ret = wait_event_interruptible(
				qmi->read_wait_q,
				qmi->read_wake_cond);

		/* check if usb is connected during the wait */
		if (qmi->read_wake_cond == 2)
			return -ENODEV;
	}

	if (ret == 0) {
		int copy_size = 0;
		int kcopy_size;
		unsigned int pkt_size;
		ridx = qmi->rcv_pkt_ridx;
		widx = qmi->rcv_pkt_widx;
		/* we are giving entire packets to user space */
		pkt_size =
			qmi->rcv_pkt_sizes[(ridx+num_pkts)%MAX_RCV_PKT_INDX];
		while (pkt_size < remain_size &&
			(ridx+num_pkts)%MAX_RCV_PKT_INDX != widx) {
			remain_size -= qmi->rcv_pkt_sizes[(ridx+num_pkts)%MAX_RCV_PKT_INDX];
			num_pkts++;
			pkt_size = qmi->rcv_pkt_sizes[(ridx+num_pkts)%MAX_RCV_PKT_INDX];
		}
		if (num_pkts > 0) {
			copy_size = count - remain_size;
			ret = kfifo_to_user(&qmi->qmi_rcv_kfifo,buf,copy_size,&kcopy_size);
			if (ret) {
				pr_err("%s: Error while copying to user\n", __func__);
				return -EFAULT;
			}
			qmi->rcv_pkt_ridx = (ridx+num_pkts)%MAX_RCV_PKT_INDX;
			if ( unlikely(kcopy_size != copy_size) ) {
				pr_err("%s: Copy size does not match!\n",__func__);
			}
		} else {
			dbg("No data to be read?");
		}
		return copy_size;
	} else {
		/* we don't restart the system call to allow read() to be interrupted */
		if (ret == -ERESTARTSYS)
			ret = -EINTR;
		else
			pr_err("%s: Err waiting for event %d\n",__func__,ret);
	}
	return ret;
}

static ssize_t qmi_misc_write(struct file *filep, const char __user *buf,
				size_t len,
				loff_t *ppos)
{
	int if_num, err = 0;
	int write_len;
	struct usb_device *dev;
	struct miscdevice *misc_dev;
	struct qmi_private *qmi;

	misc_dev = filep->private_data;
	qmi = container_of(misc_dev, struct qmi_private, qmi_misc_dev);

	/* check if usb is connected */
	if (!atomic_read(&qmi->usb_connected)) {
		pr_warning("%s: Can't write to qmi device\n",__func__);
		return -ENODEV;
	}
	if_num = qmi->usb_intf->cur_altsetting->desc.bInterfaceNumber;
	write_len = (len > PAGE_SIZE)? PAGE_SIZE: len;

	dev = interface_to_usbdev(qmi->usb_intf);
	if (copy_from_user(qmi->write_buf,buf, write_len)) {
		pr_err("%s: Error while copying from user\n", __func__);
		return -EFAULT;
	}

	if (debug) print_hex(qmi->write_buf,write_len);

	/* Use sync call. We want to resume the bus before sending data */
	err = qmi_autopm_get_interface(qmi);

	if (err) {
		return err;
	}

	/* async write is probably better for performance.
	 * However, there is no way to notify if error occurs */
	write_len = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
		0x00, 0x21, 0, if_num, qmi->write_buf,
		write_len, USB_CTRL_SET_TIMEOUT);

	if (write_len < 0) {
		qmi_autopm_put_interface(qmi);
		pr_err("%s: Unable to write ctrl message."
			"error = %d\n", __func__, write_len);
		goto done;
	}

	/*
	 * The reference count will never reach 0 if the modem doesn't
	 * respond the control message. Use a timer to keep track of this
	 */
	mod_timer(&qmi->qmi_timer, jiffies +
		msecs_to_jiffies(QMI_CONTROL_MSG_TIMEOUT_MS));

done:
	return write_len;
}

static unsigned int qmi_misc_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct miscdevice *misc_dev;
	struct qmi_private *qmi;

	misc_dev = filep->private_data;
	qmi = container_of(misc_dev,struct qmi_private,qmi_misc_dev);

	poll_wait(filep, &qmi->read_poll_q, wait);
	if (kfifo_len(&qmi->qmi_rcv_kfifo))
		mask |= POLLIN | POLLRDNORM;
	if (! qmi->qmi_intr_ep)
		mask |= POLLHUP;
	else
		mask |= POLLOUT;

	return mask;
}

static const struct file_operations qmi_misc_fops = {
	.owner		= THIS_MODULE,
	.read		= qmi_misc_read,
	.write		= qmi_misc_write,
	.open		= qmi_misc_open,
	.poll		= qmi_misc_poll,
	.release	= qmi_misc_release,
};

static const char *qmi_misc_dev_name[MAX_RMNET_DEVS] = { "qmi0" , "qmi1" };

static struct usb_host_endpoint *qmi_get_intr_ep(struct usb_interface *intf)
{
	int				tmp;
	struct usb_host_interface	*alt = NULL;
	struct usb_host_endpoint	*intr_ep = NULL;

	for (tmp = 0; tmp < intf->num_altsetting; tmp++) {
		unsigned	ep;

		alt = intf->altsetting + tmp;

		for (ep = 0; ep < alt->desc.bNumEndpoints; ep++) {
			struct usb_host_endpoint	*e;
			int	intr = 0;

			e = alt->endpoint + ep;
			switch (e->desc.bmAttributes) {
			case USB_ENDPOINT_XFER_INT:
				if (!usb_endpoint_dir_in(&e->desc))
					continue;
				intr = 1;
				break;
			default:
				continue;
			}
			if (usb_endpoint_dir_in(&e->desc)) {
				intr_ep = e;
			} else {
				/* sth wrong here */
			}
		}
		if (intr_ep)
			break;
	}
	if (!intr_ep)
		return NULL;
	return intr_ep;

}

static void usb_intr_in_cb(struct urb *urb)
{
	int err;
	int status = urb->status;
	struct qmi_private *qmi = urb->context;

	if (!status) {
		struct usb_ctrlrequest *req_pkt =
				(struct usb_ctrlrequest *)urb->transfer_buffer;
		if (!req_pkt) {
			dbg("%s: NULL req_pkt", __func__);
			return;
		}
		dbg("\nbRequestType=%u,"
			"bRequest=%u,"
			"wValue=%u,"
			"wIndex=%u,"
			"wLength=%u\n",
			req_pkt->bRequestType,
			req_pkt->bRequest,
			req_pkt->wValue,
			req_pkt->wIndex,
			req_pkt->wLength
		);

		if (qmi) {
			qmi->intr_urb = urb;
			err = schedule_work(&qmi->get_encap_work);
		}
		return;
	}

	/* Recovery mechanism */
	if (status != ESHUTDOWN && status != -ENOENT && qmi) {

		/* Called in interrupt context. can't use sync calls */
		err = qmi_autopm_get_interface_async(qmi);

		if (err) {
			return;
		}

		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err) {
			pr_err("%s: Re-submit intr URB failed. err = %d\n",
				__func__, err);
		}
		qmi_autopm_put_interface_async(qmi);
	}

	return;
}

static int qmi_probe (struct usb_interface *intf,
			const struct usb_device_id *prod)
{
	struct usb_endpoint_descriptor *ep;
	struct usb_device *dev;
	struct qmi_private *qmi_priv = NULL;
	struct usb_host_endpoint *qmi_intr_ep;
	int if_num = intf->cur_altsetting->desc.bInterfaceNumber;
	int ret;
	int i;

	dbg("Probing qmi interface...");
	if (prod->idVendor == HSU_QC_VENDOR_ID &&
		prod->idProduct == HSU_QC_PRODUCT_9001 && if_num != 3) {
		/* there is no way to identify qmi device other than interface number
		 * for mdm9600 FFA */
		dbg("ignore other interface #%d",if_num);
		return -ENODEV;
	}

	/* qmi device must have a interrupt ep */
	if ((qmi_intr_ep = qmi_get_intr_ep(intf)) == NULL) {
		return -EINVAL;
	}

	/* associate qmi device to qmi driver data */
	for (i=0 ; i<MAX_RMNET_DEVS ; i++) {
		if (!atomic_read(&qmi_devs[i]->usb_connected)) {
			qmi_priv = qmi_devs[i];
			break;
		}
	}

	if (qmi_priv == NULL) {
		pr_warning("%s: More than %d QMI devices in the system?\n",
		__func__,MAX_RMNET_DEVS);
		return -EINVAL;
	}

	/* Autosuspend timer */
	setup_timer(&qmi_priv->qmi_timer, qmi_msg_timeout,
		(unsigned long) qmi_priv);

	/* remote wakeup has to be supported for autosuspend */
	intf->needs_remote_wakeup = 1;
	ret = qmi_remote_wakeup_init(qmi_priv);
	if (ret) {
		pr_err("%s: Error (%d) in remote wakeup init\n",
		__func__, ret);
		return ret;
	}

	qmi_priv->qmi_intr_ep = qmi_intr_ep;
	ep = &qmi_intr_ep->desc;
	qmi_priv->ep_addr = ep->bEndpointAddress;
	qmi_priv->intr_buf_size = le16_to_cpu(ep->wMaxPacketSize);

	if (qmi_priv->intr_buf_size > DEFAULT_INTR_SIZE) {
		/* we should fix this. */
		pr_err("%s: default size for interrupt ep mismatch - %d / %d \n",
			__func__,DEFAULT_INTR_SIZE,qmi_priv->intr_buf_size );
		kfree(qmi_priv->intr_buf);
		qmi_priv->intr_buf = kmalloc(qmi_priv->intr_buf_size, GFP_KERNEL);
	}

	qmi_priv->usb_intf = intf;
	dev = interface_to_usbdev(intf);
	usb_fill_int_urb (qmi_priv->intr_urb,dev,
			  usb_rcvintpipe(dev,ep->bEndpointAddress),
			  qmi_priv->intr_buf,
			  qmi_priv->intr_buf_size,
			  usb_intr_in_cb,
			  qmi_priv,
			  ep->bInterval);

	INIT_KFIFO(qmi_priv->qmi_rcv_kfifo);
	ret = usbnet_probe (intf,prod);

	if (ret < 0) {
		pr_err("%s: usbnet probe failed with %d\n",__func__,ret);
		goto failed;
	}

	qmi_priv->usbnet_data = usb_get_intfdata(intf);
	usb_set_intfdata(intf,qmi_priv);

	if (waitqueue_active(&qmi_priv->read_poll_q))
		wake_up_interruptible(&qmi_priv->read_poll_q);

	if (atomic_read(&qmi_priv->opened)) {
		/* it's already open, submit urb now */
		if (qmi_priv->intr_urb->hcpriv) {
			return 0;
		}

		ret = qmi_autopm_get_interface_async(qmi_priv);

		if (ret) {
			goto failed;
		}

		/* it's already open, submit urb now */
		ret = usb_submit_urb(qmi_priv->intr_urb, GFP_ATOMIC);

		if (ret) {
			pr_err("%s: submit intr urb failed. (%d)\n",
				__func__, ret);
			qmi_autopm_put_interface_async(qmi_priv);
			usb_set_intfdata(intf, qmi_priv->usbnet_data);
			usbnet_disconnect(intf);
			goto failed;
		}

		qmi_autopm_put_interface_async(qmi_priv);
	}
	spin_lock_init(&wakeup_lock);
	wake_lock_init(&wlock,WAKE_LOCK_SUSPEND,"");
	atomic_set(&qmi_priv->usb_connected, 1);
	return 0;

failed:
	qmi_remote_wakeup_destroy(qmi_priv);
	qmi_intr_ep = NULL;
	return ret;
}

static void qmi_disconnect (struct usb_interface *intf)
{
	struct qmi_private *qmi = usb_get_intfdata(intf);
	dbg("QMI disconnecting...");
	qmi->qmi_intr_ep = NULL;
	qmi->read_wake_cond = 2;
	atomic_set(&qmi->usb_connected, 0);
	wake_up(&qmi->read_wait_q);

	if (waitqueue_active(&qmi->read_poll_q)) {
		wake_up_interruptible(&qmi->read_poll_q);
	}

	usb_kill_urb(qmi->intr_urb);

	/* Reset the fifo and the indices */
	kfifo_reset(&qmi->qmi_rcv_kfifo);
	qmi->rcv_pkt_widx = qmi->rcv_pkt_ridx = 0;
	usb_set_intfdata(intf,qmi->usbnet_data);
	usbnet_disconnect(intf);
	del_timer(&qmi->qmi_timer);
	atomic_set(&qmi->qmi_msg_cnt, 0);
	cancel_delayed_work(&qmi->qmi_rw.work);
	qmi_remote_wakeup_destroy(qmi);
	wake_lock_destroy(&wlock);
	return;
}

static int qmi_suspend (struct usb_interface *intf, pm_message_t message)
{
	struct qmi_private *qmi = usb_get_intfdata(intf);
	int retval = 0;

	retval = usbnet_suspend(intf, message);

	if (qmi->qmi_intr_ep) {
		usb_kill_urb(qmi->intr_urb);
	}

	if (!wakeup_irq_enabled) {
		wakeup_irq_enabled = 1;
		enable_irq(gpio_to_irq(qmi->qmi_rw.gpio));
	}

	wake_unlock(&wlock);
	return retval;
}

static int qmi_resume(struct usb_interface *intf)
{
	struct qmi_private *qmi = usb_get_intfdata(intf);
	int err = 0;
	unsigned long flags;

	wake_lock(&wlock);

	spin_lock_irqsave(&wakeup_lock, flags);

	if (wakeup_irq_enabled) {
		disable_irq_nosync(gpio_to_irq(qmi->qmi_rw.gpio));
		wakeup_irq_enabled = 0;
	}
	spin_unlock_irqrestore(&wakeup_lock,flags);

	err = usbnet_resume(intf);

	if (qmi->qmi_intr_ep) {

		if (qmi->intr_urb->hcpriv) {
			return err;
		}

		err = qmi_autopm_get_interface_async(qmi);

		if (err) {
			return err;
		}

		err = usb_submit_urb(qmi->intr_urb, GFP_ATOMIC);

		if (err) {
			qmi_autopm_put_interface_async(qmi);
			return err;
		}

		/*
		 * Remote wake could have resumed the bus. Hence, don't
		 * reduce the ref count on the interface. If we did, the
		 * bus might get suspended again. To avoid this, use the
		 * timer.
		 */
		mod_timer(&qmi->qmi_timer, jiffies +
			msecs_to_jiffies(QMI_CONTROL_MSG_TIMEOUT_MS));
	}

	return err;
}

static struct usb_driver qmi_driver = {
	.name		=	"qmi",
	.probe		=	qmi_probe,
	.disconnect	=	qmi_disconnect,
	.suspend	=	qmi_suspend,
	.resume		=	qmi_resume,
	.reset_resume   =       qmi_resume,
	.id_table	=	id_table,
	.no_dynamic_id	=	1,
	.supports_autosuspend = 1,
};

static int qmi_remote_wakeup_destroy(struct qmi_private *qmi)
{
	free_irq(gpio_to_irq(qmi->qmi_rw.gpio), qmi);
	gpio_free(qmi->qmi_rw.gpio);
	return 0;
}

static void qmi_rw_delayed_work(struct work_struct *work)
{
	int err;
	struct qmi_remote_wakeup *qmi_rw = container_of(work,
			struct qmi_remote_wakeup, work.work);
	struct qmi_private *qmi = container_of(qmi_rw,
			struct qmi_private, qmi_rw);

	if (!qmi) return;

	if (!gpio_get_value(qmi_rw->gpio)) {
		pr_info("%s: Received spurious interrupt\n", __func__);
		return;
	}
	if ((err = qmi_autopm_get_interface(qmi))) {
		return;
	}
	mod_timer(&qmi->qmi_timer, jiffies +
		msecs_to_jiffies(QMI_CONTROL_MSG_TIMEOUT_MS));
}

extern int pm_suspend_in_progress;
static irqreturn_t qmi_remote_wakeup_handler(int irq, void *devid)
{
	struct qmi_private *qmi = (struct qmi_private *)devid;
	unsigned long flags;

	if (!qmi)
		return IRQ_HANDLED;

	if (qmi->qmi_rw.gpio <= 0) {
		pr_info("%s: Something wrong\n", __func__);
		return IRQ_HANDLED;
	}
	wake_lock(&wlock);

	spin_lock_irqsave(&wakeup_lock, flags);

	if (wakeup_irq_enabled){
		disable_irq_nosync(gpio_to_irq(qmi->qmi_rw.gpio));
		wakeup_irq_enabled = 0;
	}

	spin_unlock_irqrestore(&wakeup_lock, flags);

	if( !pm_suspend_in_progress )
	   schedule_delayed_work(&qmi->qmi_rw.work,
		msecs_to_jiffies(INTERRUPT_DEBOUNCE_TIME));

	return IRQ_HANDLED;
}

/*
 * request the remote wakeup GPIO and request an irq for
 * remote wakeup. Init a delayed work queue that is scheduled
 * when the interrupt fires.
 */
static int qmi_remote_wakeup_init(struct qmi_private *qmi)
{
	int err = 0;

	if (!qmi) {
		pr_err("%s: fatal error. data structure missing!\n", __func__);
		return -1;
	}

	if (qmi->qmi_rw.gpio <= 0) {
		/* Platform doesn't need GPIO for remote wake */
		return 0;
	}

	err = gpio_request(qmi->qmi_rw.gpio, "remote_wakeup");

	if (err) {
		return err;
	}

	gpio_direction_input(qmi->qmi_rw.gpio);
	err = request_irq(gpio_to_irq(qmi->qmi_rw.gpio),
		qmi_remote_wakeup_handler,
		IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING,
		"remote_wakeup", qmi);
	if (err) {
		return err;
	}
	disable_irq(gpio_to_irq(qmi->qmi_rw.gpio));
	enable_irq_wake(gpio_to_irq(qmi->qmi_rw.gpio));
	INIT_DELAYED_WORK(&qmi->qmi_rw.work, qmi_rw_delayed_work);
	return err;
}

/* Assign the remote wakeup GPIO */
static int __devinit qmi_omap_probe(struct platform_device *pdev)
{
	struct qmi_platform_data *pdata;
	int i = 0;

	pdata = pdev->dev.platform_data;

	if (!pdata)
		return 0;

	for (i = 0; i < MAX_RMNET_DEVS; i++) {
		qmi_devs[i]->qmi_rw.gpio = pdata->remote_wakeup_gpio;
	}

	return 0;
}

static int __devexit qmi_omap_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver qmi_omap_driver = {
	.driver = {
		.name = "qmi",
		.owner = THIS_MODULE,
	},
	.probe = qmi_omap_probe,
	.remove = qmi_omap_remove,
};

static int __init qmi_init(void)
{
	int retval;
	int i;
	dbg("Registering USB QMI driver");
	/* allocate all qmi_private data structures at init */
	for (i=0 ; i<MAX_RMNET_DEVS ; i++) {
		qmi_devs[i] = kzalloc(sizeof(struct qmi_private), GFP_KERNEL);
		qmi_devs[i]->intr_buf = kmalloc(DEFAULT_INTR_SIZE, GFP_KERNEL);
		qmi_devs[i]->encap_rsp_buf = kmalloc(PAGE_SIZE,GFP_KERNEL);
		qmi_devs[i]->write_buf = kmalloc(PAGE_SIZE,GFP_KERNEL);
		qmi_devs[i]->intr_urb = usb_alloc_urb(0, GFP_KERNEL);
		atomic_set(&qmi_devs[i]->qmi_msg_cnt, 0);
		atomic_set(&qmi_devs[i]->usb_connected, 0);
		init_waitqueue_head(&qmi_devs[i]->read_wait_q);
		init_waitqueue_head(&qmi_devs[i]->read_poll_q);
		INIT_WORK(&qmi_devs[i]->get_encap_work, do_get_encap_rsp);
		INIT_KFIFO(qmi_devs[i]->qmi_rcv_kfifo);
		/* register device file */
		qmi_devs[i]->qmi_misc_dev.name = qmi_misc_dev_name[i];
		qmi_devs[i]->qmi_misc_dev.minor = MISC_DYNAMIC_MINOR;
		qmi_devs[i]->qmi_misc_dev.fops = &qmi_misc_fops;
		if (misc_register(&qmi_devs[i]->qmi_misc_dev) < 0) {
			return -EINVAL;
		}
		qmi_devs[i]->qmi_rw.gpio = -1;
	}

	retval = usb_register(&qmi_driver);
	if (!retval) {
		retval = platform_driver_register(&qmi_omap_driver);
	}

	return retval;
}

static void __exit qmi_exit(void)
{
	int i;
	dbg("De-registering USB QMI driver");
	platform_driver_unregister(&qmi_omap_driver);
	usb_deregister(&qmi_driver);
	/* release all qmi_private data structures at init */
	for (i=0; i<MAX_RMNET_DEVS; i++) {
		kfree(qmi_devs[i]->write_buf);
		kfree(qmi_devs[i]->encap_rsp_buf);
		kfree(qmi_devs[i]->intr_buf);
		usb_free_urb(qmi_devs[i]->intr_urb);
		misc_deregister(&qmi_devs[i]->qmi_misc_dev);
		kfree(qmi_devs[i]);
	}
}

module_init(qmi_init);
module_exit(qmi_exit);

MODULE_DESCRIPTION("QMI driver");
MODULE_LICENSE("GPL");
