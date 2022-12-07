// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2020 Google LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/sched/clock.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <linux/vmalloc.h>
#include <linux/miscdevice.h>
#include <misc/logbuffer.h>

#include <uapi/linux/time.h>

#define LOG_BUFFER_ENTRIES      1024
#define LOG_BUFFER_ENTRY_SIZE   256
#define ID_LENGTH		50

struct logbuffer {
	int logbuffer_head;
	int logbuffer_tail;
	spinlock_t logbuffer_lock;	/* protect from multiple _log() */
	u8 *buffer;
	char id[ID_LENGTH];

	struct miscdevice misc;
	char name[50];
};

/* Device suspended since last logged. */
static bool suspend_since_last_logged;
/* Log index for logbuffer_logk */
static atomic_t log_index = ATOMIC_INIT(0);

static void __logbuffer_log(struct logbuffer *instance,
			    const char *tmpbuffer, bool record_utc)
{
	u64 ts_nsec = local_clock();
	unsigned long rem_nsec = do_div(ts_nsec, 1000000000);

	if (record_utc) {
		struct timespec64 ts;
		struct rtc_time tm;

		ktime_get_real_ts64(&ts);
		rtc_time64_to_tm(ts.tv_sec, &tm);
		scnprintf(instance->buffer + (instance->logbuffer_head *
			  LOG_BUFFER_ENTRY_SIZE),
			  LOG_BUFFER_ENTRY_SIZE,
			  "[%5lu.%06lu] %d-%02d-%02d %02d:%02d:%02d.%09lu UTC",
			  (unsigned long)ts_nsec, rem_nsec / 1000,
			  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	} else {
		scnprintf(instance->buffer + (instance->logbuffer_head *
			  LOG_BUFFER_ENTRY_SIZE),
			  LOG_BUFFER_ENTRY_SIZE, "[%5lu.%06lu] %s",
			  (unsigned long)ts_nsec, rem_nsec / 1000,
			  tmpbuffer);
	}

	instance->logbuffer_head = (instance->logbuffer_head + 1)
			% LOG_BUFFER_ENTRIES;
	if (instance->logbuffer_head == instance->logbuffer_tail) {
		instance->logbuffer_tail = (instance->logbuffer_tail + 1) %
					   LOG_BUFFER_ENTRIES;
	}
}

void logbuffer_vlog(struct logbuffer *instance, const char *fmt,
		    va_list args)
{
	char tmpbuffer[LOG_BUFFER_ENTRY_SIZE];
	unsigned long flags;

	if (!instance)
		return;

	/*
	 * Empty log msgs are passed from TCPM to log RTC. The RTC is printed
	 * if thats the first message printed after resume.
	 */
	if (fmt)
		vsnprintf(tmpbuffer, sizeof(tmpbuffer), fmt, args);

	spin_lock_irqsave(&instance->logbuffer_lock, flags);
	if (instance->logbuffer_head < 0 ||
	    instance->logbuffer_head >= LOG_BUFFER_ENTRIES) {
		pr_warn("Bad log buffer index %d\n", instance->logbuffer_head);
		goto abort;
	}

	/* Print UTC at the start of the buffer */
	if (instance->logbuffer_head == instance->logbuffer_tail ||
	    instance->logbuffer_head == LOG_BUFFER_ENTRIES - 1) {
		__logbuffer_log(instance, tmpbuffer, true);
	/* Print UTC when logging after suspend */
	} else if (suspend_since_last_logged) {
		__logbuffer_log(instance, tmpbuffer, true);
		suspend_since_last_logged = false;
	} else if (!fmt) {
		goto abort;
	}

	__logbuffer_log(instance, tmpbuffer, false);

abort:
	spin_unlock_irqrestore(&instance->logbuffer_lock, flags);
}
EXPORT_SYMBOL_GPL(logbuffer_vlog);

void logbuffer_log(struct logbuffer *instance, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	logbuffer_vlog(instance, fmt, args);
	va_end(args);
}
EXPORT_SYMBOL_GPL(logbuffer_log);

void logbuffer_logk(struct logbuffer *instance, int loglevel, const char *fmt, ...)
{
	char log[LOG_BUFFER_ENTRY_SIZE];
	unsigned int index;
	va_list args;

	if (!fmt || !instance)
		return;

	index = atomic_inc_return(&log_index);

	va_start(args, fmt);
	scnprintf(log, LOG_BUFFER_ENTRY_SIZE, "[%5u] %s", index, fmt);
	logbuffer_vlog(instance, log, args);
	scnprintf(log, LOG_BUFFER_ENTRY_SIZE, "%s: [%5u] %s\n", instance->name, index, fmt);
	vprintk_emit(0, loglevel, NULL, log, args);
	va_end(args);
}
EXPORT_SYMBOL_GPL(logbuffer_logk);

static int logbuffer_seq_show(struct seq_file *s, void *v)
{
	struct logbuffer *instance = (struct logbuffer *)s->private;
	int tail;

	spin_lock_irq(&instance->logbuffer_lock);
	tail = instance->logbuffer_tail;
	while (tail != instance->logbuffer_head) {
		seq_printf(s, "%s\n", instance->buffer +
			   (tail * LOG_BUFFER_ENTRY_SIZE));
		tail = (tail + 1) % LOG_BUFFER_ENTRIES;
	}

	spin_unlock_irq(&instance->logbuffer_lock);

	return 0;
}

static int logbuffer_dev_open(struct inode *inode, struct file *file)
{
	struct logbuffer *instance =
		container_of(file->private_data, struct logbuffer, misc);

	inode->i_private = instance;
	file->private_data = NULL;
	return single_open(file, logbuffer_seq_show, inode->i_private);
}

static const struct file_operations logbuffer_dev_operations = {
	.owner = THIS_MODULE,
	.open = logbuffer_dev_open,
	.read = seq_read,
	.release = single_release,
};

struct logbuffer *logbuffer_register(const char *name)
{
	struct logbuffer *instance;
	int ret;

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance)
		return ERR_PTR(-ENOMEM);

	instance->buffer = vzalloc(LOG_BUFFER_ENTRIES * LOG_BUFFER_ENTRY_SIZE);
	if (!instance->buffer) {
		instance = ERR_PTR(-ENOMEM);
		goto free_instance;
	}

	strlcpy(instance->name, "logbuffer_", sizeof(instance->name));
	strlcat(instance->name, name, sizeof(instance->name));
	instance->misc.minor = MISC_DYNAMIC_MINOR;
	instance->misc.name = instance->name;
	instance->misc.fops = &logbuffer_dev_operations;

	ret = misc_register(&instance->misc);
	if (ret) {
		pr_err("Logbuffer error while doing misc_register ret=%d\n", ret);
		goto free_buffer;
	}

	strlcpy(instance->id, name, sizeof(instance->id));

	spin_lock_init(&instance->logbuffer_lock);

	pr_info("id:%s registered\n", name);
	return instance;

free_buffer:
	vfree(instance->buffer);
free_instance:
	kfree(instance);

	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL_GPL(logbuffer_register);

void logbuffer_unregister(struct logbuffer *instance)
{
	if (!instance)
		return;

	misc_deregister(&instance->misc);

	vfree(instance->buffer);
	pr_info("id:%s unregistered\n", instance->id);
	kfree(instance);
}
EXPORT_SYMBOL_GPL(logbuffer_unregister);

int logbuffer_suspend(void)
{
	suspend_since_last_logged = true;
	return 0;
}

static struct syscore_ops logbuffer_ops = {
	.suspend        = logbuffer_suspend,
};

static int __init logbuffer_dev_init(void)
{
	register_syscore_ops(&logbuffer_ops);

	return 0;
}

static void logbuffer_dev_exit(void)
{
	unregister_syscore_ops(&logbuffer_ops);
}
early_initcall(logbuffer_dev_init);
module_exit(logbuffer_dev_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google BMS logbuffer");
