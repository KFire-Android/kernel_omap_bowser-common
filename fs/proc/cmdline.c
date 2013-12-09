#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#ifdef CONFIG_MACH_OMAP4_BOWSER
#include <asm/system.h>
#endif

static int cmdline_proc_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_MACH_OMAP4_BOWSER
	seq_printf(m, "%s androidboot.serialno=%s\n", saved_command_line, system_serial16);
#else
	seq_printf(m, "%s\n", saved_command_line);
#endif
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_cmdline_init(void)
{
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
module_init(proc_cmdline_init);
