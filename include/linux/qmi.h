#ifndef _QMI_H
#define _QMI_H

#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>

#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/usb/cdc.h>
#include <linux/usb/usbnet.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <plat/usb.h>
#include <plat/cpu.h>

#define HSU_QC_VENDOR_ID			0x05C6
#define LAB126_VENDOR_ID			0x1949
#define PANTECH_VENDOR_ID			0x106c

#define HSU_QC_PRODUCT_9001			0x9001
#define HSU_QC_PRODUCT_9011			0x9011
#define ELMO_PRODUCT_ID				0x9001
#define GROVER_PRODUCT_ID			0x9002
#define ERNIE_PRODUCT_ID			0x9003
#define PANTECH_PRODUCT_3718			0x3718

#define QMI_CONTROL_MSG_TIMEOUT_MS		1000
#define INTERRUPT_DEBOUNCE_TIME			20

struct qmi_platform_data {
	void (*init) (void);
	int remote_wakeup_gpio;
};

struct qmi_remote_wakeup {
	struct delayed_work work;
	int gpio;
};

struct qmi_private {
	struct urb	*intr_urb;	/* Interrupt IN urb */
	u8		*intr_buf;	/* Interrupt IN buf */
#define DEFAULT_INTR_SIZE	(64)
	int		intr_buf_size;	/* size of Interrupt IN buf*/
	__u8		ep_addr;	/* ep address of Interrupt IN*/
	struct usb_interface    *usb_intf; /* USB interface of the QMI interface */
#define MAX_RCV_PKT_INDX	(16)
	/* Most 16 packets in the rcv fifo */
	unsigned int	rcv_pkt_sizes[MAX_RCV_PKT_INDX];
	unsigned int	rcv_pkt_widx;
	unsigned int	rcv_pkt_ridx;
#define QMI_RCV_KFIFO_SIZE	(4096*4)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
	DECLARE_KFIFO(qmi_rcv_kfifo, char, QMI_RCV_KFIFO_SIZE);
#else
	struct kfifo	qmi_rcv_kfifo;
	unsigned char	qmi_rcv_kfifokfifo_buffer[QMI_RCV_KFIFO_SIZE +
			sizeof(struct kfifo)];
#endif
	u8		*encap_rsp_buf;
	u8		*write_buf;
	int		read_wake_cond;
	wait_queue_head_t read_wait_q;
	wait_queue_head_t read_poll_q;
	struct work_struct get_encap_work;
	void		*usbnet_data;
	atomic_t	usb_connected;	/* usb connected */
	atomic_t	opened;		/* file opened for process*/
	struct miscdevice qmi_misc_dev;
	struct usb_host_endpoint *qmi_intr_ep;
	atomic_t	qmi_msg_cnt;
	struct timer_list	qmi_timer;
	struct qmi_remote_wakeup qmi_rw;
};

#endif
