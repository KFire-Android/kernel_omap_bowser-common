/*
 * trapz.c
 *
 * TRAPZ (TRAcing and Profiling for Zpeed) Log Driver
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Andy Prunicki (prunicki@lab126.com)
 * Martin Unsal (munsal@lab126.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * TODO replace all literal error return values with appropriate Linux error values
 * TODO pare down the included files - some are not necessary
 * TODO Remove dead, unused code
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/hash.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/rbtree.h>
#include <linux/wait.h>
#include <linux/eventpoll.h>
#include <linux/mount.h>
#include <linux/bitops.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/trapz.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/logger.h>
#include <linux/debugfs.h>
#include <linux/completion.h>

#define CREATE_TRACE_POINTS
#include <trace/events/trapz_tp.h>

static const char *g_trapz_version = "0.1";

static const int g_buffer_size = TRAPZ_DEFAULT_BUFFER_SIZE;
static const char *g_trapz_info_name = TRAPZ_PROC_INFO;
static const char *g_trapz_data_name = TRAPZ_PROC_DATA;
static const char *g_trapz_text_name = TRAPZ_PROC_TEXT;

/**
 * Internal data structure
 */

static struct _trapz_device_info {
	struct miscdevice trapz_device;
	int blocked;
	wait_queue_head_t wq;
} trapz_device_info;

typedef struct {
	struct completion comp;
	int blocked;
} trapz_event_info;

static struct trapz_data {
	trapz_entry_t *pBuffer;                // base address of internal entry buffer
	trapz_entry_t *pLimit;                 // first entry address past the end of the buffer
	trapz_entry_t *pHead;                  // entries are added at the head
	trapz_entry_t *pTail;                  // entries are removed from the tail
	int bufferSize;                        // the buffer will hold this many entries
	int count;                             // the buffer contains this many entries
	int total;                             // count of entries added since the last reset
	int overtaken;                         // count of entries overtaken due to buffer wrap
	struct proc_dir_entry *pProcInfo;      // /proc node pointers
	struct proc_dir_entry *pProcData;      // /proc node pointers
	struct proc_dir_entry *pProcText;      // /proc node pointers
	spinlock_t bufLock;                    // spin lock for SMP
	char *os_comp_loglevels;               // log levels for OS components
	char *app_comp_loglevels;              // log levels for app components
	int initialized;                       // indicates if driver has completed initialization
} g_data = {
	.initialized = 0                       // ==0 indicates uninitialized
};

static int trapz_retrieve_index(trapz_read_entry_t *pDst, int index);
static int trapz_control_reset(void);

static int trapz_proc_write(struct file *pFile, const char __user *pBuff,
				unsigned long len, void *pData);
static void trapz_get_g_data(struct trapz_data *pData);
static int trapz_proc_info_read(char *pPage, char **ppStart, off_t off,
				int count, int *pEof, void *pData);
static int trapz_proc_data_read(char *pPage, char **ppStart, off_t off,
				int count, int *pEof, void *pData);
static int trapz_proc_text_read(char *pPage, char **ppStart, off_t off,
				int count, int *pEof, void *pData);
static int trapz_probe(struct platform_device *p);

static struct platform_driver trapz_platform  = {
	.probe = trapz_probe,
	.driver = {
		.name = "trapz"
	}
};

struct trigger_list {
        struct list_head list;
        trapz_trigger_t trigger;
        struct timespec start_ts;
};

#ifdef CONFIG_TRAPZ_PERF
trapz_event_info trapz_trigger_start_perf, trapz_trigger_stop_perf;
static unsigned char trapz_perf_in_progress = 0;

static int trapz_debugfs_init(void);
static void trapz_init_event(trapz_event_info *event);
static void trapz_trigger_event(trapz_event_info *event);
static void trapz_wait_event(trapz_event_info *event);
#endif

static int trapz_open(struct inode *, struct file *);
static int trapz_release(struct inode *, struct file *);
static ssize_t trapz_read(struct file *, char *, size_t, loff_t *);
static ssize_t trapz_write(struct file *, const char *, size_t, loff_t *);
static loff_t trapz_llseek(struct file *filp, loff_t off, int whence);
static long trapz_ioctl(struct file *, unsigned int, unsigned long);
static int delete_trigger(const trapz_trigger_t *trigger, struct list_head *head);
static int find_trace_pt(const int trace_pt, struct trigger_list **trigger, struct list_head *head);
static void copy_timespec(const struct timespec *src, struct timespec *dest);
static void copy_trigger(const trapz_trigger_t *src, trapz_trigger_t *dest);
static void send_trigger_uevent(const trapz_trigger_event_t *trigger_event);

/* The trigger implementation is admittedly no performant for very many triggers.  We have
   weak requirements around triggers, and thus a simple, small link list is our current
   implementation until we have more concrete requirements. */
#define MAX_TRIGGERS 3
LIST_HEAD(trigger_head);
static int g_trigger_count = 0;

struct file_operations fops = {
   .llseek = trapz_llseek,
   .read = trapz_read,
   .write = trapz_write,
   .open = trapz_open,
   .release = trapz_release,
   .unlocked_ioctl = trapz_ioctl,
};

//=============================================================================

/**
 * Kernel API used to log scenario points.
 */
SYSCALL_DEFINE6(trapz, int, ctrl1, int, ctrl2, int, extra1, int, extra2, int, reserved, struct trapz_info __user *, ti)
{
	return systrapz(ctrl1, ctrl2, extra1, extra2, ti);
}

static inline void trapz_handle_flags(int ctrl1)
{
	int flags = TRAPZ_GET_LFLAGS(ctrl1);
	static unsigned char buf[27] = "\7TRAPZ";

//#ifdef CONFIG_TRAPZ_PERF
//	if (flags & TRAPZ_FLAG_PERFSTART) {
//		trapz_trigger_event(&trapz_trigger_start_perf);
//	}
//	if (flags & TRAPZ_FLAG_PERFSTOP) {
//		trapz_trigger_event(&trapz_trigger_stop_perf);
//	}
//#endif
	if (flags & TRAPZ_FLAG_LOG_KL)
		printk(KERN_WARNING "TRAPZ log: %08x\n", g_data.total - 1);
	if (flags & TRAPZ_FLAG_LOG_AL) {
		snprintf(&(buf[7]), 20, "Sequence = %08x", g_data.total - 1);
		__alog_main(buf, 27);
	}
	if (flags & TRAPZ_FLAG_LOG_KT)
		trace_trapz_tp(g_data.total - 1);
}

/**
 * Internal kernel API used to log scenario points.
 */
long systrapz(int ctrl1, int ctrl2, int extra1, int extra2, struct trapz_info __user *ti)
{
	unsigned long flags;
	trapz_entry_t *pEntry = NULL;
	struct timespec tv;
	trapz_trigger_event_t trigger_event;
	int filtered, trigger_rc;

	if (!g_data.initialized)
		return -98;

	int level = TRAPZ_GET_LLEVEL(ctrl1);
	int cat_id = TRAPZ_GET_CAT_ID(ctrl1);
	int comp_id = TRAPZ_GET_COMP_ID(ctrl1);

	trigger_event.trigger.start_trace_point = 0;

	filtered = 1;
	// Filter out events based on log level
	if (trapz_check_loglevel(level, cat_id, comp_id)) {
		// Not filtered by log level
		getnstimeofday(&tv);
		filtered = 0;
		spin_lock_irqsave(&g_data.bufLock, flags);
		{
			int trigger_rc;
			struct trigger_list *found_trig;

			trigger_rc = find_trace_pt(ctrl1 & TRAPZ_TRIGGER_MASK, &found_trig, &trigger_head);
			if (trigger_rc == 1) {
				found_trig->start_ts.tv_sec = tv.tv_sec;
				found_trig->start_ts.tv_nsec = tv.tv_nsec;
			} else if (trigger_rc == 2 && found_trig->start_ts.tv_sec != 0) {
				//TODO Load a local struct with the event info and send it out of crit. section
				//     Also, delete the trigger if it is a one-shot.
				//     This is not a high-priority to fix, as we have not even utilized
				//     the trigger mechanism yet.  Don't over-invest !!
				copy_trigger(&found_trig->trigger, &trigger_event.trigger);
				copy_timespec(&found_trig->start_ts, &trigger_event.start_ts);
				copy_timespec(&tv, &trigger_event.end_ts);
				if (found_trig->trigger.single_shot) {
					if (delete_trigger(&found_trig->trigger, &trigger_head) != 0) {
						printk(KERN_ERR "Trapz delete trigger failed.\n");
					}
					trigger_event.trigger_active = 0;
				} else {
					trigger_event.trigger_active = 1;
				}
			}

			if (g_data.count >= g_data.bufferSize) {
				g_data.pTail++;
				if (g_data.pTail >= g_data.pLimit)
					g_data.pTail = g_data.pBuffer;
				g_data.count--;
				g_data.overtaken++;
			}

			pEntry = g_data.pHead++;
			if (g_data.pHead >= g_data.pLimit)
				g_data.pHead = g_data.pBuffer;
			g_data.count++;

			pEntry->sequence = g_data.total++;
		}
		spin_unlock_irqrestore(&g_data.bufLock, flags);
	}
	if (filtered) {
		// Trapz call filtered out, return time if requested
		if (ti != NULL) {
			getnstimeofday(&tv);
			trapz_info_t kti;
			kti.tv = tv;
			kti.sequence = -1;

			if (copy_to_user(ti, &kti, sizeof(kti)))
				return -EFAULT;
		}

		return 0;
	}

	if (ctrl1 & TRAPZ_LFLAGS_OUT_MASK)
		trapz_handle_flags(ctrl1);

	pEntry->tv = tv;
	pEntry->ctrl1 = ctrl1;
	pEntry->ctrl2 = ctrl2;
	pEntry->extra1 = extra1;
	pEntry->extra2 = extra2;

	if (trigger_rc > 0) {
	}

#ifdef CONFIG_TRAPZ_DETAIL
	pEntry->cpu = smp_processor_id();

	if (!in_interrupt()) {
		pEntry->pid = current->tgid;
		pEntry->tid = current->pid;
	}
	else {
		pEntry->pid = 0;
		pEntry->tid = 0;
	}
#else
	pEntry->cpu = 0;
	pEntry->pid = 0;
	pEntry->tid = 0;
#endif

	if (trigger_event.trigger.start_trace_point != 0) {
		send_trigger_uevent(&trigger_event);
	}

	// Return time and sequence info if requested
	if (ti != NULL) {
		trapz_info_t kti;
		kti.tv = tv;
		kti.sequence = pEntry->sequence;

		if (copy_to_user(ti, &kti, sizeof(kti)))
			return -EFAULT;
	}

	// unblock if needed
	if (trapz_device_info.blocked) {
		trapz_device_info.blocked = 0;
		wake_up_interruptible(&(trapz_device_info.wq));
	}

	return 0;
}
EXPORT_SYMBOL(systrapz);

/**
 * Retrieve a single entry by index.
 */
static int trapz_retrieve_index(trapz_read_entry_t *pDst, int index)
{
	int rc = -1;
	unsigned long flags;

	spin_lock_irqsave(&g_data.bufLock, flags);
	{
		if (index < g_data.count) {
			trapz_entry_t *pEntry = g_data.pTail + index;
			if (pEntry >= g_data.pLimit)
				pEntry -= g_data.bufferSize;
			pDst->entry = *pEntry;
			pDst->overtaken = g_data.overtaken;
			pDst->remaining = g_data.count - index - 1;
			pDst->total = g_data.total;
			rc = 0;
		}
	}
	spin_unlock_irqrestore(&g_data.bufLock, flags);

	return rc;
}

/**
 * Reset the TRAPZ store.
 */
static int trapz_control_reset(void)
{
	unsigned long flags;

	if (!g_data.initialized)
		return -1;

	spin_lock_irqsave(&g_data.bufLock, flags);
	g_data.total = g_data.count = g_data.overtaken = 0;
	g_data.pHead = g_data.pTail = g_data.pBuffer;
	spin_unlock_irqrestore(&g_data.bufLock, flags);
	return 0;
}

#ifdef CONFIG_TRAPZ_PERF
static ssize_t trapz_dfs_char_read(struct file *file, char *buffer, size_t length, loff_t *offset)
{
	char buf[5] = {0};
	int size;
	trapz_event_info *event;

	event = (trapz_event_info *) file->private_data;
	if (event == NULL)
		return 0;

	size = snprintf(buf, sizeof(buf), "%u\n", event->blocked);
	if (size > 0)
		copy_to_user(buffer, buf, size);
	if (*offset == 0)
	{
		(*offset)++;
		return size;
	}
	else
	{
		return 0;
	}
}

static ssize_t trapz_dfs_char_write(struct file *file, const char *buffer, size_t length, loff_t *offset)
{
	char tmp;
	trapz_event_info *event;

	event = (trapz_event_info *) file->private_data;
	if (event == NULL)
		return 0;

	if (length == 0)
		return 0;
	get_user(tmp, buffer);
	if (tmp == '0') {
		if (event->blocked) {
			trapz_trigger_event(event);
		}
	} else {
		if (!event->blocked) {
			trapz_wait_event(event);
		}
	}
}

#define DEFINE_TRAPZ_DEBUGFS_FILE_OPS(__event)                            \
static int __event ## _dfs_open(struct inode *inode, struct file * file)  \
{                                                                         \
	file->private_data = &__event;                                          \
	return nonseekable_open(inode, file);                                   \
}                                                                         \
static struct file_operations __event ## _fops = {                        \
	.open = __event ## _dfs_open,                                           \
	.read = trapz_dfs_char_read,                                            \
	.write = trapz_dfs_char_write,                                          \
};

DEFINE_TRAPZ_DEBUGFS_FILE_OPS(trapz_trigger_start_perf)
DEFINE_TRAPZ_DEBUGFS_FILE_OPS(trapz_trigger_stop_perf)

static void trapz_init_event(trapz_event_info *event)
{
	event->blocked = 0;
	init_completion(&(event->comp));
}

static void trapz_trigger_event(trapz_event_info *event)
{
	if (event->blocked) {
		event->blocked = 0;
		complete_all(&(event->comp));
	}
}

static void trapz_wait_event(trapz_event_info *event)
{
	event->blocked = 1;
	INIT_COMPLETION(event->comp);
	wait_for_completion(&(event->comp));
}

/**
 * Initialize debugfs entries for trapz.
 */
static int __init trapz_debugfs_init(void)
{
	struct dentry *trapz_dfs_dir;
	// create the directory

	trapz_dfs_dir = debugfs_create_dir(TRAPZ_DEV_NAME, NULL);
	if (trapz_dfs_dir == NULL)
	{
		printk(KERN_ERR "trapz_init: cannot create debug fs entry\n");
	}
	else
	{
		if ((debugfs_create_file("perf_tool_wait_start", 0666, trapz_dfs_dir, NULL,
		     &trapz_trigger_start_perf_fops) == NULL) ||
		    (debugfs_create_file("perf_tool_wait_stop", 0666, trapz_dfs_dir, NULL,
		     &trapz_trigger_stop_perf_fops) == NULL) ||
		    (debugfs_create_u8("perf_tool_in_progress", 0666, trapz_dfs_dir,
		     &trapz_perf_in_progress) == NULL)) {
		  printk(KERN_ERR "trapz_init: cannot create debug fs entry\n");
		}
		else
		{
			trapz_init_event(&trapz_trigger_start_perf);
			trapz_init_event(&trapz_trigger_stop_perf);
		}
	}
	return 0;
}
#endif /* CONFIG_TRAPZ_PERF */

/**
 * Responds to writes on any TRAPZ /proc node.
 * Currently does nothing.
 * It's thought that this could be a simple control interface.
 */
static int trapz_proc_write(struct file *pFile, const char __user *pBuff, unsigned long len, void *pData)
{

	if (!g_data.initialized)
		return -1;

	//TODO
	//unsigned long copy_from_user( void *to, const void __user *from, unsigned long n );
	return 0;
}


/**
 * Retrieve a snapshot of g_data.
 */
static void trapz_get_g_data(struct trapz_data *pData)
{
	unsigned long flags;

	spin_lock_irqsave(&g_data.bufLock, flags);
	{
		*pData = g_data;
	}
	spin_unlock_irqrestore(&g_data.bufLock, flags);
}


/**
 * Responds to reads on the TRAPZ /proc info node.
 * Returns text block of TRAPZ infrastructure info.
 * (See kernel/fs/proc/generic.c "How to be a proc read function")
 */
static int trapz_proc_info_read(char *pPage, char **ppStart, off_t off, int count, int *pEof, void *pData)
{
	struct trapz_data data;
	int rc;

	trapz_get_g_data(&data);

	*pEof = 1;
	rc = sprintf(pPage, "TRAPZ v0.0\n       size: %8d\n      count: %8d\n      total: %8d\n  overtaken: %8d\n",
		data.bufferSize, data.count, data.total, data.overtaken);

	return rc;
}


/**
 * Responds to reads on the TRAPZ /proc data node.
 * Returns one entry (binary) and a count of remaining entries.
 * (See kernel/fs/proc/generic.c "How to be a proc read function")
 */
static int trapz_proc_data_read(char *pPage, char **ppStart, off_t off, int count, int *pEof, void *pData)
{
	int rc;

	if (count < sizeof(trapz_read_entry_t))
		return 0;

	if (trapz_retrieve_index((trapz_read_entry_t *) pPage, off / sizeof(trapz_read_entry_t)) == 0) {
		*ppStart = (char *)sizeof(trapz_read_entry_t);
		rc = sizeof(trapz_read_entry_t);
	} else {
		rc = 0;
	}
	return rc;
}


/**
 * Responds to reads on the TRAPZ /proc text node.
 * Returns one entry (as a line of formatted text) and a count of remaining entries.
 * (See kernel/fs/proc/generic.c "How to be a proc read function")
 */
static int trapz_proc_text_read(char *pPage, char **ppStart, off_t off, int count, int *pEof, void *pData)
{
	trapz_read_entry_t entry;
	int rc;

	if (trapz_retrieve_index(&entry, off / sizeof(trapz_read_entry_t)) == 0) {
		*ppStart = (char *)sizeof(trapz_read_entry_t);
		rc = snprintf(pPage, count, "%ld.%09ld: %08x %08x %08x %08x (%d,%d,%d)\n",
			entry.entry.tv.tv_sec, entry.entry.tv.tv_nsec,
			entry.entry.ctrl1, entry.entry.ctrl2, entry.entry.extra1,
			entry.entry.extra2,
			entry.overtaken, entry.remaining, entry.total);
	} else {
		rc = 0;
	}
	return rc;
}

int trapz_probe(struct platform_device *p)
{
#ifdef CONFIG_TRAPZ_LED
	debug_led1 = ((struct trapz_platform_data *)p->dev.platform_data)->leds->led1;
	debug_led2 = ((struct trapz_platform_data *)p->dev.platform_data)->leds->led2;
#endif
	return 0;
}

static int trapz_open(struct inode *inode, struct file *file)
{
	unsigned long flags;
	// set offset to be the current lowest sequence
	spin_lock_irqsave(&g_data.bufLock, flags);
	{
		file->f_pos = g_data.total - g_data.count;
	}
	spin_unlock_irqrestore(&g_data.bufLock, flags);
	return 0;
}

static int trapz_release(struct inode *inode, struct file *file)
{
	return 0;
}

static loff_t trapz_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;
	switch (whence) {
		case 0: /* SEEK_SET */
			newpos = off;
			break;

		case 1: /* SEEK_CUR */
			newpos = filp->f_pos + off;
			break;

		case 2: /* SEEK_END */
			newpos = g_data.total + off;
			break;

		default: /* can't happen */
			return -EINVAL;
	}
	if (newpos < 0) return -EINVAL;
	filp->f_pos = newpos;
	return newpos;
}

/*
 * reads out trapz entries from store
*/
static ssize_t trapz_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
	for (;;) {
		int base = g_data.total - g_data.count;
		int index;
		trapz_entry_t *pEnd, *pStart;
		int objcnt, objlim;
		int size;

		objlim = length / sizeof(trapz_entry_t);

		if (*offset < base) {
			*offset = base;
		}
		if (*offset >= g_data.total) {
			if(filp->f_flags & O_NONBLOCK)
				return 0;
			else {
				trapz_device_info.blocked = 1;
				if (wait_event_interruptible(trapz_device_info.wq, trapz_device_info.blocked == 0)
				    == -ERESTARTSYS)
				{
					return 0;
				}
				
			}
			continue;
		}
		index = *offset - g_data.total + g_data.count;
		if (index < 0) {
			index = 0;
		}
		pEnd = g_data.pHead;
		pStart = g_data.pTail + index;
		if (pStart >= g_data.pLimit)
			pStart -= g_data.bufferSize;
		if (pStart < pEnd) {
			/* We intentionally do not copy in the current entry as
			 * it is racy.
			 */
			objcnt = pEnd - pStart;
		} else {
			/* We slop up to the end of the buffer in the case that
			 * our entry point is before our tail, keeps it as one
			 * copy, ie simpler.
			 */
			objcnt = g_data.pLimit - pStart;
		}
		
		if (objcnt > objlim) {
			objcnt = objlim;
		}
		size = objcnt * sizeof(trapz_entry_t);
		if (copy_to_user(buffer, pStart, size))
			return -EFAULT;
		/* Now we figure out if we wrapped. */
		if (g_data.total - *offset > g_data.bufferSize) {
			/* Wrapped, start over :( */
			*offset = g_data.total;
			continue;
		}
		*offset += objcnt;
		return size;
	}
	return -EFAULT;
}

static ssize_t trapz_write(struct file *filp, const char *buffer, size_t length, loff_t *offset)
{
	// not allowing write for now
	return 0;
}

static inline int compute_loglevel_buffsize() {
	//The buffer size is the number of components * 2 bits, giving 3 values for each
	// component.  We have 4 log values, but log level WARN cannot be turned off.
	return (TRAPZ_MAX_COMP_ID + 1) >> 2;
}

int trapz_check_loglevel(const int level, const int cat_id, const int component_id) {
	int rc = 0;
	int byteIdx, bitIdx, bufflevel;
	char *buffer;

	if (g_data.initialized && component_id >= 0) {
		if (level >= TRAPZ_LOG_MIN && level <= TRAPZ_LOG_MAX) {
			if (level == TRAPZ_LOG_WARN) {
				//We always log WARN
				rc = 1;
			} else {
				if (cat_id == TRAPZ_CAT_APPS) {
					buffer = g_data.app_comp_loglevels;
				} else {
					buffer = g_data.os_comp_loglevels;
				}

				if (component_id <= TRAPZ_MAX_COMP_ID) {
					byteIdx = component_id / 4;
					bitIdx = (component_id % 4) << 1;
					bufflevel = ((buffer[byteIdx] & (TRAPZ_LOG_MAX << bitIdx)) >> bitIdx);
					if (bufflevel <= level) {
						rc = 1;
					}
				}
			}
		}
	}

	return rc;
}
EXPORT_SYMBOL(trapz_check_loglevel);

static void set_comp_loglevel(char *buffer, const int level, const int component_id) {
	int byteIdx, bitIdx, bufflevel;

	if (level >= TRAPZ_LOG_MIN && level <= TRAPZ_LOG_MAX &&
			component_id >= 0 && component_id <= TRAPZ_MAX_COMP_ID) {
		byteIdx = component_id / 4;
		bitIdx = (component_id % 4) << 1;
		bufflevel = level << bitIdx;
		buffer[byteIdx] = (buffer[byteIdx] & ((TRAPZ_LOG_MAX << bitIdx) ^ 0xFF)) | bufflevel;
	}
}

static void set_loglevel(const int level, const int cat_id, const int component_id) {
	const int buffsize = compute_loglevel_buffsize();
	int buffval = 0;

	if (!g_data.initialized)
		return;

	if (component_id >= 0) {
		// Setting the log level of just a component
		if (cat_id == TRAPZ_CAT_APPS) {
			set_comp_loglevel(g_data.app_comp_loglevels, level, component_id);
		} else {
			set_comp_loglevel(g_data.os_comp_loglevels, level, component_id);
		}
	} else {
		switch (level) {
		case TRAPZ_LOG_WARN:
			buffval = 0xFF;
			break;
		case TRAPZ_LOG_INFO:
			buffval = 0xAA;
			break;
		case TRAPZ_LOG_DEBUG:
			buffval = 0x55;
			break;
		case TRAPZ_LOG_VERBOSE:
			buffval = 0x00;
			break;
		}

		if (cat_id >= 0) {
			// Setting the log level of entire category
			if (cat_id == TRAPZ_CAT_APPS) {
				memset(g_data.app_comp_loglevels, buffval, buffsize);
			} else {
				memset(g_data.os_comp_loglevels, buffval, buffsize);
			}
		} else {
			// Setting the log level of the entire device
			memset(g_data.app_comp_loglevels, buffval, buffsize);
			memset(g_data.os_comp_loglevels, buffval, buffsize);
		}
	}
}

static void init_logbuffers(const int os_val, const int app_val) {
	const int buffsize = compute_loglevel_buffsize();

	memset(g_data.os_comp_loglevels, os_val, buffsize);
	memset(g_data.app_comp_loglevels, app_val, buffsize);
}

static void send_trigger_uevent(const trapz_trigger_event_t *trigger_event) {
	char *envp[8];
	int i = 0;

	for (i = 1; i < 6; i++) {
		envp[i] = kmalloc(sizeof(char) * 24, GFP_KERNEL);
	}

	envp[0] = "EVENT=TRIGGER";
	snprintf(envp[1], 24, "ID=%d", trigger_event->trigger.trigger_id);
	snprintf(envp[2], 24, "START_S=%ld", trigger_event->start_ts.tv_sec);
	snprintf(envp[3], 24, "START_NS=%ld", trigger_event->start_ts.tv_nsec);
	snprintf(envp[4], 24, "END_S=%ld", trigger_event->end_ts.tv_sec);
	snprintf(envp[5], 24, "END_NS=%ld", trigger_event->end_ts.tv_nsec);
	if (trigger_event->trigger_active) {
		envp[6] = "ACTIVE=TRUE";
	} else {
		envp[6] = "ACTIVE=FALSE";
	}
	envp[7] = NULL;

	kobject_uevent_env(&trapz_device_info.trapz_device.this_device->kobj, KOBJ_CHANGE, envp);
	for (i = 1; i < 6; i++) {
		kfree(envp[i]);
	}
}

static void copy_timespec(const struct timespec *src, struct timespec *dest) {
	dest->tv_sec = src->tv_sec;
	dest->tv_nsec = src->tv_nsec;
}

static void copy_trigger(const trapz_trigger_t *src, trapz_trigger_t *dest) {
	dest->start_trace_point = src->start_trace_point;
	dest->end_trace_point = src->end_trace_point;
	dest->trigger_id = src->trigger_id;
	dest->single_shot = src->single_shot;
}

static int compare_triggers(const trapz_trigger_t *trig1, const trapz_trigger_t *trig2) {
	int rc = 0;

	if (trig1->start_trace_point == trig2->start_trace_point &&
		trig1->end_trace_point == trig2->end_trace_point) {
		rc = 1;
	}

	return rc;
}

static int compare_trigger(const trapz_trigger_t *trig, const int trace_pt) {
	int rc = 0;

	if (trig->start_trace_point == trace_pt) {
		rc = 1;
	} else if (trig->end_trace_point == trace_pt) {
		rc = 2;
	}

	return rc;
}

static struct trigger_list * find_trigger(const trapz_trigger_t *trigger, struct list_head *head) {
	struct list_head *iter;
	struct trigger_list *curr_trig_list;

	list_for_each(iter, head) {
		curr_trig_list = list_entry(iter, struct trigger_list, list);
		if(compare_triggers(&curr_trig_list->trigger, trigger)) {
			return curr_trig_list;
		}
	}

	return NULL;
}

static int find_trace_pt(const int trace_pt, struct trigger_list **trigger, struct list_head *head) {
	struct list_head *iter;
	struct trigger_list *curr_trig_list;
	int rc = 0;

	list_for_each(iter, head) {
		curr_trig_list = list_entry(iter, struct trigger_list, list);
		rc = compare_trigger(&curr_trig_list->trigger, trace_pt);
		if (rc) {
			*trigger = curr_trig_list;
			break;
		}
	}

	return rc;
}

static int add_trigger(const trapz_trigger_t *trigger, struct list_head *head) {
	int rc = 0;
	struct trigger_list *trigger_list_ptr;
	//struct trigger_list *trig_list = find_trigger(trigger, head);

	if (g_trigger_count < MAX_TRIGGERS) {
		//Still more room for triggers
		if (find_trigger(trigger, head) == NULL) {
			//Trigger does not exist
			trigger_list_ptr = (struct trigger_list *) kzalloc(sizeof(struct trigger_list), GFP_KERNEL);
			if (trigger_list_ptr == NULL) {
				rc = -ENOMEM;
			} else {
				copy_trigger(trigger, &trigger_list_ptr->trigger);
				INIT_LIST_HEAD(&trigger_list_ptr->list);
				list_add(&trigger_list_ptr->list, head);
				g_trigger_count++;
			}
		} else {
			//Trigger already exists
			rc = -EEXIST;
		}
	} else {
		rc = -ENOMEM;
	}

	return rc;
}

static int delete_trigger(const trapz_trigger_t *trigger, struct list_head *head) {
	int rc = -EINVAL;
	struct trigger_list *trig_list = find_trigger(trigger, head);


	if (trig_list != NULL) {
		list_del(&trig_list->list);
		kfree(trig_list);
		g_trigger_count--;
		rc = 0;
	}

	return rc;
}

static void clear_triggers(struct list_head *head) {
	struct trigger_list *curr_trig_list;

	while (!list_empty(head)) {
		curr_trig_list = list_entry(head->next, struct trigger_list, list);
		list_del(head->next);
		kfree(curr_trig_list);
	}
	g_trigger_count = 0;
}

static long trapz_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	trapz_config config;
	struct trapz_data data;
	long rc = -EINVAL;
	trapz_trigger_t trigger;
	int level, cat_id, component_id;

	switch (cmd) {
		case TRAPZ_GET_CONFIG:
			trapz_get_g_data(&data);
			config.bufferSize = data.bufferSize;
			config.count = data.count;
			config.total = data.total;
			config.overtaken = data.overtaken;
			rc = 0;
			if (copy_to_user((void __user *) arg, &config,
					sizeof(trapz_config))) {
				rc = -EFAULT;
			}
			break;
		case TRAPZ_SET_BUFFER_SIZE:
			break;
		case TRAPZ_RESET_DEFAULTS:
			rc = 0;
			spin_lock_irqsave(&g_data.bufLock, flags);
			{
				init_logbuffers(0xFF, 0x55);
				clear_triggers(&trigger_head);
			}
			spin_unlock_irqrestore(&g_data.bufLock, flags);
			break;
		case TRAPZ_CLEAR_BUFFER:
			rc = trapz_control_reset();
			break;
		case TRAPZ_GET_VERSION:
			// copy entire string with terminating null
			rc = 0;
			if (copy_to_user((void __user *)arg, g_trapz_version, strlen(g_trapz_version)+1)) {
				rc = -EFAULT;
			}
			break;
		case TRAPZ_SET_LOG_LEVEL:
			rc = 0;
			level = (arg & (TRAPZ_LLEVEL_IN_MASK << (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE + 2)))
				>> (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE + 2);
			cat_id = -1;
			component_id = -1;
			if (arg & (1 << (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE + 1))) {
				cat_id = (arg & (TRAPZ_CAT_ID_IN_MASK << TRAPZ_COMP_ID_SIZE))
					>> TRAPZ_COMP_ID_SIZE;
			}
			if (arg & (1 << (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE))) {
				component_id = arg & TRAPZ_COMP_ID_IN_MASK;
			}
			set_loglevel(level, cat_id, component_id);
			break;
		case TRAPZ_CHK_LOG_LEVEL:
			rc = 0;
			level = (arg & (TRAPZ_LLEVEL_IN_MASK << (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE + 2)))
				>> (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE + 2);
			cat_id = -1;
			component_id = -1;
			if (arg & (1 << (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE + 1))) {
				cat_id = (arg & (TRAPZ_CAT_ID_IN_MASK << TRAPZ_COMP_ID_SIZE))
					>> TRAPZ_COMP_ID_SIZE;
			}
			if (arg & (1 << (TRAPZ_CAT_ID_SIZE + TRAPZ_COMP_ID_SIZE))) {
				component_id = arg & TRAPZ_COMP_ID_IN_MASK;
			}
			if (cat_id >= 0 && component_id >= 0) {
				//We only check for actual component ids - not general
				rc = trapz_check_loglevel(level, cat_id, component_id);
			}
			break;
		case TRAPZ_ADD_TRIGGER:
			if (copy_from_user(&trigger, (void __user *) arg, sizeof(trigger))) {
				rc = -EFAULT;
			} else {
				spin_lock_irqsave(&g_data.bufLock, flags);
				{
					rc = add_trigger(&trigger, &trigger_head);
				}
				spin_unlock_irqrestore(&g_data.bufLock, flags);
			}
			break;
		case TRAPZ_DEL_TRIGGER:
			if (copy_from_user(&trigger, (void __user *) arg, sizeof(trigger))) {
				rc = -EFAULT;
			} else {
				spin_lock_irqsave(&g_data.bufLock, flags);
				{
					rc = delete_trigger(&trigger, &trigger_head);
				}
				spin_unlock_irqrestore(&g_data.bufLock, flags);
			}
			break;
		case TRAPZ_CLR_TRIGGERS:
			spin_lock_irqsave(&g_data.bufLock, flags);
			{
				clear_triggers(&trigger_head);
			}
			spin_unlock_irqrestore(&g_data.bufLock, flags);
			rc = 0;
			break;
		case TRAPZ_CNT_TRIGGERS:
			rc = g_trigger_count;
			break;
		default:
			break;
	}
	return rc;
}

/**
 * Initialize the TRAPZ /proc node.
 */
static int __init trapz_proc_init(void)
{
	if (g_data.pProcInfo)
		return 0;

	g_data.pProcInfo = create_proc_entry(g_trapz_info_name, S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP, NULL);
	g_data.pProcData = create_proc_entry(g_trapz_data_name, S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP, NULL);
	g_data.pProcText = create_proc_entry(g_trapz_text_name, S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP, NULL);

	if (!g_data.pProcInfo || !g_data.pProcData || !g_data.pProcText)
		printk(KERN_INFO "trapz_proc_init: cannot create /proc node\n");

	if (g_data.pProcInfo) {
		g_data.pProcInfo->read_proc = trapz_proc_info_read;
		g_data.pProcInfo->write_proc = trapz_proc_write;
	}
	if (g_data.pProcData) {
		g_data.pProcData->read_proc = trapz_proc_data_read;
		g_data.pProcData->write_proc = trapz_proc_write;
	}
	if (g_data.pProcText) {
		g_data.pProcText->read_proc = trapz_proc_text_read;
		g_data.pProcText->write_proc = trapz_proc_write;
	}
	return 0;
}

/**
 * Initialize the TRAPZ device.
 */
static int __init trapz_device_init(void)
{
	int ret = 0;

	trapz_device_info.trapz_device.minor = MISC_DYNAMIC_MINOR;
	trapz_device_info.trapz_device.name = TRAPZ_DEV_NAME;
	trapz_device_info.trapz_device.fops = &fops;
	trapz_device_info.trapz_device.parent = NULL;

	init_waitqueue_head(&trapz_device_info.wq);
	trapz_device_info.blocked = 0;

	ret = misc_register(&trapz_device_info.trapz_device);
	if (ret)
	{
		printk(KERN_ERR "trapz_init: cannot register misc device\n");
		return ret;
	}
	return 0;
}

static int __init trapz_init(void)
{
	const int logbuffersize = compute_loglevel_buffsize();
	if (g_data.initialized)
		return 0;

	g_data.bufferSize = g_buffer_size;
	g_data.total = g_data.count = g_data.overtaken = 0;
	g_data.pProcInfo = g_data.pProcData = g_data.pProcText = 0;
	spin_lock_init(&g_data.bufLock);
	g_data.pLimit = g_data.pHead = g_data.pTail = g_data.pBuffer =
		(trapz_entry_t *)kmalloc(
			sizeof(trapz_entry_t) * g_data.bufferSize, GFP_KERNEL);
	g_data.pLimit += g_data.bufferSize;
	g_data.os_comp_loglevels = kmalloc(logbuffersize, GFP_KERNEL);
	g_data.app_comp_loglevels = kmalloc(logbuffersize, GFP_KERNEL);
	if (!g_data.pBuffer || !g_data.os_comp_loglevels || !g_data.app_comp_loglevels) {
		printk(KERN_ERR "trapz_init: cannot allocate kernel memory\n");
		if (g_data.pBuffer) {
			kfree(g_data.pBuffer);
		}
		if (g_data.os_comp_loglevels) {
			kfree(g_data.os_comp_loglevels);
		}
		if (g_data.app_comp_loglevels) {
			kfree(g_data.app_comp_loglevels);
		}
		return -ENOMEM;
	} else {
		init_logbuffers(0xFF, 0x55);
		g_data.initialized = 1;
		trapz_proc_init();
		trapz_device_init();
#ifdef CONFIG_TRAPZ_PERF
		trapz_debugfs_init();
#endif
		platform_driver_register(&trapz_platform);
	}
	return 0;
}

__initcall(trapz_init);

