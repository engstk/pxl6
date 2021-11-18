/*
 * Google LWIS Top Level Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-top-dev: " fmt

#include "lwis_device_top.h"
#include "lwis_event.h"
#include "lwis_init.h"

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-top"

static int lwis_top_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				int access_size);
static int lwis_top_close(struct lwis_device *lwis_dev);
static struct lwis_device_subclass_operations top_vops = {
	.register_io = lwis_top_register_io,
	.register_io_barrier = NULL,
	.device_enable = NULL,
	.device_disable = NULL,
	.event_enable = NULL,
	.event_flags_updated = NULL,
	.close = lwis_top_close,
};

static int lwis_top_event_subscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				    int trigger_device_id, int receiver_device_id);
static int lwis_top_event_unsubscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				      int receiver_device_id);
static void lwis_top_event_notify(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				  int64_t trigger_event_count, int64_t trigger_event_timestamp,
				  bool in_irq);
static void lwis_top_event_subscribe_release(struct lwis_device *lwis_dev);
static struct lwis_event_subscribe_operations top_subscribe_ops = {
	.subscribe_event = lwis_top_event_subscribe,
	.unsubscribe_event = lwis_top_event_unsubscribe,
	.notify_event_subscriber = lwis_top_event_notify,
	.release = lwis_top_event_subscribe_release,
};

struct lwis_event_subscribe_info {
	/* Subscribed Event ID */
	int64_t event_id;
	/* LWIS device who subscribed an event */
	struct lwis_device *receiver_dev;
	/* LWIS device who will trigger an event */
	struct lwis_device *trigger_dev;
	/* Node in the lwis_top_device->event_subscribe hash table */
	struct hlist_node node;
};

struct lwis_trigger_event_info {
	/* Store emitted event id from trigger device */
	int64_t trigger_event_id;
	/* Store emitted event count from trigger device */
	int64_t trigger_event_count;
	/* Store emitted event timestamp from trigger device */
	int64_t trigger_event_timestamp;
	/* node of list head */
	struct list_head node;
};

static void subscribe_tasklet_func(unsigned long data)
{
	struct lwis_top_device *lwis_top_dev = (struct lwis_top_device *)data;
	struct lwis_trigger_event_info *trigger_event;
	struct lwis_event_subscribe_info *p;
	struct list_head *it_sub, *it_sub_tmp;
	unsigned long flags;

	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	list_for_each_safe (it_sub, it_sub_tmp, &lwis_top_dev->emitted_event_list_tasklet) {
		trigger_event = list_entry(it_sub, struct lwis_trigger_event_info, node);
		list_del(&trigger_event->node);
		hash_for_each_possible (lwis_top_dev->event_subscriber, p, node,
					trigger_event->trigger_event_id) {
			if (p->event_id == trigger_event->trigger_event_id) {
				/* Notify subscriber an event is happening */
				lwis_device_external_event_emit(
					p->receiver_dev, trigger_event->trigger_event_id,
					trigger_event->trigger_event_count,
					trigger_event->trigger_event_timestamp, false);
			}
		}
		kfree(trigger_event);
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
}

static void lwis_top_event_notify(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				  int64_t trigger_event_count, int64_t trigger_event_timestamp,
				  bool in_irq)
{
	struct lwis_top_device *lwis_top_dev = (struct lwis_top_device *)lwis_dev;
	unsigned long flags;

	struct lwis_trigger_event_info *trigger_event =
		kmalloc(sizeof(struct lwis_trigger_event_info), GFP_ATOMIC);
	if (trigger_event == NULL) {
		dev_err(lwis_top_dev->base_dev.dev, "Allocate trigger_event_info failed");
		return;
	}

	trigger_event->trigger_event_id = trigger_event_id;
	trigger_event->trigger_event_count = trigger_event_count;
	trigger_event->trigger_event_timestamp = trigger_event_timestamp;
	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	list_add_tail(&trigger_event->node, &lwis_top_dev->emitted_event_list_tasklet);
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
	/* Schedule deferred subscribed events */
	tasklet_schedule(&lwis_top_dev->subscribe_tasklet);
}

static int lwis_top_event_unsubscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				      int receiver_device_id)
{
	struct lwis_top_device *lwis_top_dev = (struct lwis_top_device *)lwis_dev;
	struct lwis_device *trigger_dev;
	struct lwis_event_subscribe_info *p;
	struct hlist_node *tmp;
	struct lwis_trigger_event_info *pending_event, *n;
	unsigned long flags;
	bool has_subscriber = false;

	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);

	/* Remove event from hash table */
	hash_for_each_possible_safe (lwis_top_dev->event_subscriber, p, tmp, node,
				     trigger_event_id) {
		/* Clear pending events */
		list_for_each_entry_safe (pending_event, n,
					  &lwis_top_dev->emitted_event_list_tasklet, node) {
			/* The condition indicates that clear pending events by subscriber
			 * because there are possibly other subscribers have the same pending event.
			 */
			if (p->receiver_dev->id == receiver_device_id &&
			    pending_event->trigger_event_id == trigger_event_id) {
				list_del(&pending_event->node);
				kfree(pending_event);
			}
		}

		if (p->event_id == trigger_event_id && p->receiver_dev->id == receiver_device_id) {
			dev_info(lwis_dev->dev,
				 "unsubscribe event: %llx, trigger device: %s, target device: %s\n",
				 trigger_event_id, p->trigger_dev->name, p->receiver_dev->name);
			trigger_dev = p->trigger_dev;
			hash_del(&p->node);
			kfree(p);
		} else if (p->event_id == trigger_event_id &&
			   p->receiver_dev->id != receiver_device_id) {
			/* The condition indicate there are other client still subscribe the event */
			trigger_dev = p->trigger_dev;
			has_subscriber = true;
		}
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
	lwis_device_event_update_subscriber(trigger_dev, trigger_event_id, has_subscriber);
	return 0;
}

static int lwis_top_event_subscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				    int trigger_device_id, int receiver_device_id)
{
	struct lwis_top_device *lwis_top_dev = (struct lwis_top_device *)lwis_dev;
	struct lwis_device *lwis_trigger_dev = lwis_find_dev_by_id(trigger_device_id);
	struct lwis_device *lwis_receiver_dev = lwis_find_dev_by_id(receiver_device_id);
	struct lwis_event_subscribe_info *p, *new_subscription;
	unsigned long flags;
	int ret = 0;
	bool has_subscriber = true;

	if (lwis_trigger_dev == NULL || lwis_receiver_dev == NULL) {
		dev_err(lwis_top_dev->base_dev.dev, "LWIS trigger/receiver device not found");
		return -EINVAL;
	}
	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	hash_for_each_possible (lwis_top_dev->event_subscriber, p, node, trigger_event_id) {
		/* event already registered for this device */
		if (p->event_id == trigger_event_id && p->receiver_dev->id == receiver_device_id) {
			spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
			dev_info(
				lwis_dev->dev,
				"already registered event: %llx, trigger device: %s, target device: %s\n",
				trigger_event_id, lwis_trigger_dev->name, lwis_receiver_dev->name);
			goto out;
		}
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);

	/* If the subscription does not exist in hash table, create one */
	new_subscription = kmalloc(sizeof(struct lwis_event_subscribe_info), GFP_KERNEL);
	if (!new_subscription) {
		dev_err(lwis_top_dev->base_dev.dev,
			"Failed to allocate memory for new subscription\n");
		has_subscriber = false;
		ret = -ENOMEM;
		goto out;
	}
	new_subscription->event_id = trigger_event_id;
	new_subscription->receiver_dev = lwis_receiver_dev;
	new_subscription->trigger_dev = lwis_trigger_dev;
	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	hash_add(lwis_top_dev->event_subscriber, &new_subscription->node, trigger_event_id);
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
	dev_info(lwis_dev->dev, "subscribe event: %llx, trigger device: %s, target device: %s",
		 trigger_event_id, lwis_trigger_dev->name, lwis_receiver_dev->name);

out:
	ret = lwis_device_event_update_subscriber(lwis_trigger_dev, trigger_event_id,
						  has_subscriber);
	if (ret < 0) {
		dev_err(lwis_top_dev->base_dev.dev, "Failed to subcribe event : %llx\n",
			trigger_event_id);
	}
	return ret;
}

static void lwis_top_event_subscribe_init(struct lwis_top_device *lwis_top_dev)
{
	hash_init(lwis_top_dev->event_subscriber);
	INIT_LIST_HEAD(&lwis_top_dev->emitted_event_list_tasklet);
	tasklet_init(&lwis_top_dev->subscribe_tasklet, subscribe_tasklet_func,
		     (unsigned long)lwis_top_dev);
}

static void lwis_top_event_subscribe_clear(struct lwis_device *lwis_dev)
{
	struct lwis_top_device *lwis_top_dev = (struct lwis_top_device *)lwis_dev;
	struct lwis_event_subscribe_info *subscribe_info;
	struct hlist_node *tmp;
	struct lwis_trigger_event_info *pending_event, *n;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	/* Clean up subscription table */
	hash_for_each_safe (lwis_top_dev->event_subscriber, i, tmp, subscribe_info, node) {
		hash_del(&subscribe_info->node);
		kfree(subscribe_info);
	}

	/* Clean up emitted event list */
	list_for_each_entry_safe (pending_event, n, &lwis_top_dev->emitted_event_list_tasklet,
				  node) {
		list_del(&pending_event->node);
		kfree(pending_event);
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
}

static void lwis_top_event_subscribe_release(struct lwis_device *lwis_dev)
{
	struct lwis_top_device *lwis_top_dev = (struct lwis_top_device *)lwis_dev;
	lwis_top_event_subscribe_clear(lwis_dev);
	/* Clean up tasklet process */
	tasklet_kill(&lwis_top_dev->subscribe_tasklet);
}

static int lwis_top_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				int access_size)
{
	struct lwis_top_device *top_dev = (struct lwis_top_device *)lwis_dev;
	struct lwis_io_entry_rw_batch *rw_batch;
	int i;
	uint64_t reg_value;

	BUG_ON(!entry);

	if (entry->type == LWIS_IO_ENTRY_READ) {
		if (entry->rw.offset >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev, "Offset (%llu) must be < %d\n",
				entry->rw.offset, SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		entry->rw.val = top_dev->scratch_mem[entry->rw.offset];
	} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH) {
		rw_batch = &entry->rw_batch;
		if (rw_batch->offset + rw_batch->size_in_bytes > SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev,
				"Read range (%llu) exceeds scratch memory (%d)\n",
				rw_batch->offset + rw_batch->size_in_bytes, SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		for (i = 0; i < rw_batch->size_in_bytes; ++i) {
			rw_batch->buf[i] = top_dev->scratch_mem[rw_batch->offset + i];
		}
	} else if (entry->type == LWIS_IO_ENTRY_WRITE) {
		if (entry->rw.offset >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev, "Offset (%llu) must be < %d\n",
				entry->rw.offset, SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		top_dev->scratch_mem[entry->rw.offset] = entry->rw.val;
	} else if (entry->type == LWIS_IO_ENTRY_WRITE_BATCH) {
		rw_batch = &entry->rw_batch;
		if (rw_batch->offset + rw_batch->size_in_bytes > SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev,
				"Write range (%llu) exceeds scratch memory (%d)\n",
				rw_batch->offset + rw_batch->size_in_bytes, SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		for (i = 0; i < rw_batch->size_in_bytes; ++i) {
			top_dev->scratch_mem[rw_batch->offset + i] = rw_batch->buf[i];
		}
	} else if (entry->type == LWIS_IO_ENTRY_MODIFY) {
		if (entry->mod.offset >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev, "Offset (%llu) must be < %d\n",
				entry->mod.offset, SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		reg_value = top_dev->scratch_mem[entry->mod.offset];
		reg_value &= ~entry->mod.val_mask;
		reg_value |= entry->mod.val_mask & entry->mod.val;
		top_dev->scratch_mem[entry->rw.offset] = reg_value;
	} else {
		dev_err(top_dev->base_dev.dev, "Invalid IO entry type: %d\n", entry->type);
		return -EINVAL;
	}

	return 0;
}

static int lwis_top_close(struct lwis_device *lwis_dev)
{
	lwis_top_event_subscribe_clear(lwis_dev);
	return 0;
}

static int lwis_top_device_setup(struct lwis_top_device *top_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_top_device_parse_dt(top_dev);
	if (ret) {
		dev_err(top_dev->base_dev.dev, "Failed to parse device tree\n");
	}
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -ENOSYS;
#endif

	return ret;
}

static int lwis_top_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_top_device *top_dev;

	/* Allocate top device specific data construct */
	top_dev = kzalloc(sizeof(struct lwis_top_device), GFP_KERNEL);
	if (!top_dev) {
		pr_err("Failed to allocate top device structure\n");
		return -ENOMEM;
	}

	top_dev->base_dev.type = DEVICE_TYPE_TOP;
	top_dev->base_dev.vops = top_vops;
	top_dev->base_dev.subscribe_ops = top_subscribe_ops;

	/* Call the base device probe function */
	ret = lwis_base_probe((struct lwis_device *)top_dev, plat_dev);
	if (ret) {
		pr_err("Error in lwis base probe\n");
		goto error_probe;
	}

	/* Call top device specific setup function */
	ret = lwis_top_device_setup(top_dev);
	if (ret) {
		dev_err(top_dev->base_dev.dev, "Error in top device initialization\n");
		lwis_base_unprobe((struct lwis_device *)top_dev);
		goto error_probe;
	}

	lwis_top_event_subscribe_init(top_dev);

	return 0;

error_probe:
	kfree(top_dev);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_TOP_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.probe = lwis_top_device_probe,
	.driver = {
		.name = LWIS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_id_match,
	},
};

#else /* CONFIG_OF not defined */
static struct platform_device_id lwis_driver_id[] = {
	{
		.name = LWIS_DRIVER_NAME,
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, lwis_driver_id);

static struct platform_driver lwis_driver = { .probe = lwis_top_device_probe,
					      .id_table = lwis_driver_id,
					      .driver = {
						      .name = LWIS_DRIVER_NAME,
						      .owner = THIS_MODULE,
					      } };
#endif /* CONFIG_OF */

/*
 *  lwis_top_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_top_device_init(void)
{
	int ret = 0;

	pr_info("Top device initialization\n");

	ret = platform_driver_register(&lwis_driver);
	if (ret) {
		pr_err("platform_driver_register failed: %d\n", ret);
	}

	return ret;
}

int lwis_top_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
