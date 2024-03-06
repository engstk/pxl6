/*
 * Google LWIS I2C BUS Manager
 *
 * Copyright (c) 2023 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-i2c-bus-manager: " fmt

#include "lwis_device.h"
#include "lwis_i2c_bus_manager.h"
#include "lwis_device_i2c.h"
#include "lwis_i2c_sched.h"

bool lwis_i2c_bus_manager_debug;
module_param(lwis_i2c_bus_manager_debug, bool, 0644);

/* Defines the global list of bus managers shared among various I2C devices
 * Each manager would control the transfers on a single I2C bus */
static struct mutex i2c_bus_manager_list_lock;
static struct lwis_i2c_bus_manager_list i2c_bus_manager_list;

/*
 * insert_bus_manager_id_in_list:
 * Inserts the newly created instance of I2C bus manager in the list
*/
static int insert_bus_manager_id_in_list(struct lwis_i2c_bus_manager *i2c_bus_manager,
					 int i2c_bus_handle)
{
	struct lwis_i2c_bus_manager_identifier *i2c_bus_manager_identifier_node = NULL;

	if (!i2c_bus_manager)
		return -EINVAL;

	i2c_bus_manager_identifier_node =
		kzalloc(sizeof(struct lwis_i2c_bus_manager_identifier), GFP_KERNEL);
	if (!i2c_bus_manager_identifier_node) {
		pr_err("Failed to allocate lwis i2c bus manager id list node\n");
		return -ENOMEM;
	}

	i2c_bus_manager_identifier_node->i2c_bus_manager_handle = i2c_bus_handle;
	i2c_bus_manager_identifier_node->i2c_bus_manager = i2c_bus_manager;
	INIT_LIST_HEAD(&i2c_bus_manager_identifier_node->i2c_bus_manager_list_node);

	mutex_lock(&i2c_bus_manager_list_lock);
	list_add_tail(&i2c_bus_manager_identifier_node->i2c_bus_manager_list_node,
		      &i2c_bus_manager_list.i2c_bus_manager_list_head);
	mutex_unlock(&i2c_bus_manager_list_lock);

	return 0;
}

/*
 * delete_bus_manager_id_in_list:
 * Deletes the newly created instance of I2C bus manager in the list
*/
static void delete_bus_manager_id_in_list(int i2c_bus_handle)
{
	struct lwis_i2c_bus_manager_identifier *i2c_bus_manager_identifier_node = NULL;
	struct list_head *i2c_bus_manager_list_node = NULL;
	struct list_head *i2c_bus_manager_list_tmp_node = NULL;

	mutex_lock(&i2c_bus_manager_list_lock);
	list_for_each_safe (i2c_bus_manager_list_node, i2c_bus_manager_list_tmp_node,
			    &i2c_bus_manager_list.i2c_bus_manager_list_head) {
		i2c_bus_manager_identifier_node = list_entry(i2c_bus_manager_list_node,
							     struct lwis_i2c_bus_manager_identifier,
							     i2c_bus_manager_list_node);
		if (i2c_bus_manager_identifier_node->i2c_bus_manager_handle == i2c_bus_handle) {
			list_del(&i2c_bus_manager_identifier_node->i2c_bus_manager_list_node);
			kfree(i2c_bus_manager_identifier_node);
			i2c_bus_manager_identifier_node = NULL;
			break;
		}
	}
	mutex_unlock(&i2c_bus_manager_list_lock);
}

/*
 * find_i2c_bus_manager:
 * Returns a valid I2C Bus Manager for a valid i2c_bus_handle.
 * Returns NULL if the bus manager hasn't been created for this handle.
*/
static struct lwis_i2c_bus_manager *find_i2c_bus_manager(int i2c_bus_handle)
{
	struct lwis_i2c_bus_manager *i2c_bus_manager = NULL;
	struct list_head *i2c_bus_manager_list_node = NULL;
	struct list_head *i2c_bus_manager_list_tmp_node = NULL;
	struct lwis_i2c_bus_manager_identifier *i2c_bus_manager_identifier = NULL;

	mutex_lock(&i2c_bus_manager_list_lock);
	list_for_each_safe (i2c_bus_manager_list_node, i2c_bus_manager_list_tmp_node,
			    &i2c_bus_manager_list.i2c_bus_manager_list_head) {
		i2c_bus_manager_identifier = list_entry(i2c_bus_manager_list_node,
							struct lwis_i2c_bus_manager_identifier,
							i2c_bus_manager_list_node);
		if (i2c_bus_manager_identifier->i2c_bus_manager_handle == i2c_bus_handle) {
			i2c_bus_manager = i2c_bus_manager_identifier->i2c_bus_manager;
			break;
		}
	}
	mutex_unlock(&i2c_bus_manager_list_lock);

	return i2c_bus_manager;
}

/*
 * create_i2c_kthread_workers:
 * Creates I2C worker threads, one per bus
*/
static int create_i2c_kthread_workers(struct lwis_i2c_bus_manager *i2c_bus_manager,
				      struct lwis_device *lwis_dev)
{
	char i2c_bus_thread_name[LWIS_MAX_NAME_STRING_LEN];
	if (!i2c_bus_manager) {
		dev_err(lwis_dev->dev, "lwis_create_kthread_workers: I2C Bus Manager is NULL\n");
		return -ENODEV;
	}
	scnprintf(i2c_bus_thread_name, LWIS_MAX_NAME_STRING_LEN, "lwis_%s",
		  i2c_bus_manager->i2c_bus_name);
	kthread_init_worker(&i2c_bus_manager->i2c_bus_worker);
	i2c_bus_manager->i2c_bus_worker_thread = kthread_run(
		kthread_worker_fn, &i2c_bus_manager->i2c_bus_worker, i2c_bus_thread_name);
	if (IS_ERR(i2c_bus_manager->i2c_bus_worker_thread)) {
		dev_err(lwis_dev->dev, "Creation of i2c_bus_worker_thread failed for bus %s\n",
			i2c_bus_manager->i2c_bus_name);
		return -EINVAL;
	}
	return 0;
}

/*
 * check_i2c_thread_priority:
 * Checks if the lwis device being connected has the same priority as other I2C threads
 * Prints a warning message if there is a difference between the priorities
*/
static void check_i2c_thread_priority(struct lwis_i2c_bus_manager *i2c_bus_manager,
				      struct lwis_device *lwis_dev)
{
	if (i2c_bus_manager->i2c_bus_thread_priority != lwis_dev->transaction_thread_priority) {
		dev_warn(
			lwis_dev->dev,
			"I2C bus manager thread %s priority(%d) is not the same as device thread priority(%d)\n",
			i2c_bus_manager->i2c_bus_name, i2c_bus_manager->i2c_bus_thread_priority,
			lwis_dev->transaction_thread_priority);
	}
}

/*
 * set_i2c_thread_priority:
 * Sets the priority for I2C threads
*/
static int set_i2c_thread_priority(struct lwis_i2c_bus_manager *i2c_bus_manager,
				   struct lwis_device *lwis_dev)
{
	int ret = 0;
	i2c_bus_manager->i2c_bus_thread_priority = lwis_dev->transaction_thread_priority;
	if (i2c_bus_manager->i2c_bus_thread_priority != 0) {
		ret = lwis_set_kthread_priority(lwis_dev, i2c_bus_manager->i2c_bus_worker_thread,
						i2c_bus_manager->i2c_bus_thread_priority);
	}
	return ret;
}

/*
 * is_valid_connected_device:
 * Makes sure a valid client connected to this I2C executes the job on this manager
 */
static bool is_valid_connected_device(struct lwis_device *lwis_dev,
				      struct lwis_i2c_bus_manager *i2c_bus_manager)
{
	struct lwis_i2c_connected_device *connected_i2c_device;
	struct list_head *i2c_connected_device_node, *i2c_connected_device_tmp_node;

	if ((lwis_dev == NULL) || (i2c_bus_manager == NULL)) {
		return false;
	}

	list_for_each_safe (i2c_connected_device_node, i2c_connected_device_tmp_node,
			    &i2c_bus_manager->i2c_connected_devices) {
		connected_i2c_device =
			list_entry(i2c_connected_device_node, struct lwis_i2c_connected_device,
				   connected_device_node);
		if (connected_i2c_device->connected_device == lwis_dev) {
			return true;
		}
	}

	return false;
}

/*
 * set_i2c_bus_manager_name:
 * Builds and sets the I2C Bus manager name
*/
static void set_i2c_bus_manager_name(struct lwis_i2c_bus_manager *i2c_bus_manager)
{
	scnprintf(i2c_bus_manager->i2c_bus_name, LWIS_MAX_NAME_STRING_LEN, "I2C_Bus_%d",
		  i2c_bus_manager->i2c_bus_id);
}

/*
 * destroy_i2c_bus_manager:
 * Destroys this instance of the I2C bus manager
 */
static void destroy_i2c_bus_manager(struct lwis_i2c_bus_manager *i2c_bus_manager,
				    struct lwis_device *lwis_dev)
{
	int i = 0;
	if (!i2c_bus_manager) {
		return;
	}

	dev_info(lwis_dev->dev, "Destroying I2C Bus Manager: %s\n", i2c_bus_manager->i2c_bus_name);
	mutex_lock(&i2c_bus_manager->i2c_process_queue_lock);
	for (i = 0; i < I2C_MAX_PRIORITY_LEVELS; i++) {
		lwis_i2c_process_request_queue_destroy(&i2c_bus_manager->i2c_bus_process_queue[i]);
	}
	mutex_unlock(&i2c_bus_manager->i2c_process_queue_lock);

	/* Delete the bus manager instance from the list */
	delete_bus_manager_id_in_list(i2c_bus_manager->i2c_bus_id);

	/* Free the bus manager */
	kfree(i2c_bus_manager);
	i2c_bus_manager = NULL;
}

/*
 * connect_i2c_bus_manager:
 * Connects a lwis device to this instance of the I2C bus manager.
*/
static int connect_i2c_bus_manager(struct lwis_i2c_bus_manager *i2c_bus_manager,
				   struct lwis_device *lwis_dev)
{
	int ret = 0;
	struct lwis_i2c_connected_device *connected_i2c_device;

	if ((!lwis_dev) || (!i2c_bus_manager)) {
		pr_err("Null lwis device or bus manager\n");
		return -EINVAL;
	}

	if (!lwis_check_device_type(lwis_dev, DEVICE_TYPE_I2C)) {
		dev_err(lwis_dev->dev,
			"Failed trying to connect non I2C device to a I2C bus manager\n");
		return -EINVAL;
	}

	connected_i2c_device = kzalloc(sizeof(struct lwis_i2c_connected_device), GFP_KERNEL);
	if (!connected_i2c_device) {
		dev_err(lwis_dev->dev, "Failed to connect device to I2C Bus Manager\n");
		return -ENOMEM;
	}
	connected_i2c_device->connected_device = lwis_dev;
	INIT_LIST_HEAD(&connected_i2c_device->connected_device_node);
	list_add_tail(&connected_i2c_device->connected_device_node,
		      &i2c_bus_manager->i2c_connected_devices);
	i2c_bus_manager->number_of_connected_devices++;

	return ret;
}

static bool i2c_device_priority_is_valid(int device_priority)
{
	if ((device_priority >= I2C_DEVICE_HIGH_PRIORITY) &&
	    (device_priority <= I2C_DEVICE_LOW_PRIORITY)) {
		return true;
	}
	return false;
}

/*
 * lwis_i2c_bus_manager_process_worker_queue:
 * Function to be called by i2c bus manager worker thread to
 * pick the next I2C client that is scheduled for transfer.
 * The process queue will be processed in order of I2C
 * device priority.
 */
void lwis_i2c_bus_manager_process_worker_queue(struct lwis_client *client)
{
	/* Get the correct I2C Bus manager to process it's queue */
	struct lwis_device *lwis_dev = NULL;
	struct lwis_i2c_bus_manager *i2c_bus_manager = NULL;
	int i = 0;

	/* The transfers will be processed in fifo order */
	struct lwis_client *client_to_process = NULL;
	struct lwis_device *lwis_dev_to_process = NULL;
	struct lwis_i2c_process_queue *process_queue = NULL;
	struct lwis_i2c_process_request *process_request = NULL;

	struct list_head *i2c_client_node, *i2c_client_tmp_node;

	lwis_dev = client->lwis_dev;
	i2c_bus_manager = lwis_i2c_bus_manager_get_manager(lwis_dev);

	if (lwis_i2c_bus_manager_debug) {
		dev_info(lwis_dev->dev, "%s scheduled by %s\n", i2c_bus_manager->i2c_bus_name,
			lwis_dev->name);
	}

	if (!i2c_bus_manager) {
		dev_err(lwis_dev->dev, "I2C Bus Manager is null\n");
		return;
	}

	mutex_lock(&i2c_bus_manager->i2c_process_queue_lock);
	for (i = 0; i < I2C_MAX_PRIORITY_LEVELS; i++) {
		process_queue = &i2c_bus_manager->i2c_bus_process_queue[i];
		list_for_each_safe (i2c_client_node, i2c_client_tmp_node, &process_queue->head) {
			if (lwis_i2c_bus_manager_debug) {
				dev_info(lwis_dev->dev,
					"Process request nodes for %s: cur %p tmp %p\n",
					i2c_bus_manager->i2c_bus_name, i2c_client_node,
					i2c_client_tmp_node);
			}
			process_request = list_entry(i2c_client_node,
						     struct lwis_i2c_process_request, request_node);
			if (!process_request) {
				dev_err(lwis_dev->dev, "I2C Bus Worker process_request is null\n");
				break;
			}

			client_to_process = process_request->requesting_client;
			if (!client_to_process) {
				dev_err(lwis_dev->dev,
					"I2C Bus Worker client_to_process is null\n");
				break;
			}

			lwis_dev_to_process = client_to_process->lwis_dev;
			if (!lwis_dev_to_process) {
				dev_err(lwis_dev->dev,
					"I2C Bus Worker lwis_dev_to_process is null\n");
				break;
			}

			if (lwis_i2c_bus_manager_debug) {
				dev_info(lwis_dev_to_process->dev, "Processing client start %s\n",
					lwis_dev_to_process->name);
			}

			if (is_valid_connected_device(lwis_dev_to_process, i2c_bus_manager)) {
				lwis_process_transactions_in_queue(client_to_process);
				lwis_process_periodic_io_in_queue(client_to_process);
			}

			if (lwis_i2c_bus_manager_debug) {
				dev_info(lwis_dev_to_process->dev, "Processing client end %s\n",
					lwis_dev_to_process->name);
			}
		}
	}
	mutex_unlock(&i2c_bus_manager->i2c_process_queue_lock);
}

/*
 * lwis_i2c_bus_manager_create:
 * Creates a new instance of I2C bus manager
 */
int lwis_i2c_bus_manager_create(struct lwis_i2c_device *i2c_dev)
{
	int ret = 0;
	int i = 0;
	struct lwis_i2c_bus_manager *i2c_bus_manager = NULL;
	struct lwis_device *i2c_base_device = &i2c_dev->base_dev;

	if (!lwis_check_device_type(i2c_base_device, DEVICE_TYPE_I2C)) {
		return 0;
	}

	i2c_bus_manager = find_i2c_bus_manager(i2c_dev->adapter->nr);
	if (!i2c_bus_manager) {
		/* Allocate memory for I2C Bus Manager */
		i2c_bus_manager = kzalloc(sizeof(struct lwis_i2c_bus_manager), GFP_KERNEL);
		if (!i2c_bus_manager) {
			dev_err(i2c_base_device->dev, "Failed to allocate lwis i2c bus manager\n");
			return -ENOMEM;
		}

		i2c_bus_manager->i2c_bus_id = i2c_dev->adapter->nr;
		set_i2c_bus_manager_name(i2c_bus_manager);

		/* Mutex and Lock initializations */
		mutex_init(&i2c_bus_manager->i2c_bus_lock);
		mutex_init(&i2c_bus_manager->i2c_process_queue_lock);

		/* List initializations */
		INIT_LIST_HEAD(&i2c_bus_manager->i2c_connected_devices);

		/* Create a I2C transfer process queue */
		for (i = 0; i < I2C_MAX_PRIORITY_LEVELS; i++) {
			lwis_i2c_process_request_queue_initialize(
				&i2c_bus_manager->i2c_bus_process_queue[i]);
		}

		/* Insert this instance of bus manager in the bus manager list */
		ret = insert_bus_manager_id_in_list(i2c_bus_manager, i2c_dev->adapter->nr);
		if (ret < 0) {
			goto error_creating_i2c_bus_manager;
		}

		/* Create worker thread to serve this bus manager */
		ret = create_i2c_kthread_workers(i2c_bus_manager, i2c_base_device);
		if (ret < 0) {
			goto error_creating_i2c_bus_manager;
		}

		/* Set priority for the worker threads */
		ret = set_i2c_thread_priority(i2c_bus_manager, i2c_base_device);
		if (ret < 0) {
			goto error_creating_i2c_bus_manager;
		}
	}

	/* Check the current device's thread priority with respect to the bus priority */
	check_i2c_thread_priority(i2c_bus_manager, i2c_base_device);

	/* Connect this lwis device to the I2C Bus manager found/created */
	ret = connect_i2c_bus_manager(i2c_bus_manager, i2c_base_device);
	if (ret < 0) {
		goto error_creating_i2c_bus_manager;
	}

	dev_info(i2c_base_device->dev,
		 "I2C Bus Manager: %s Connected Device: %s Connected device count: %d\n",
		 i2c_bus_manager->i2c_bus_name, i2c_base_device->name,
		 i2c_bus_manager->number_of_connected_devices);

	i2c_dev->i2c_bus_manager = i2c_bus_manager;
	return ret;

error_creating_i2c_bus_manager:
	dev_err(i2c_base_device->dev, "Error creating I2C Bus Manager\n");
	if (i2c_bus_manager) {
		kfree(i2c_bus_manager);
		i2c_bus_manager = NULL;
	}
	return -EINVAL;
}

/*
 * lwis_i2c_bus_manager_disconnect:
 * Disconnects a lwis device from this instance of the I2C bus manager.
 * Doesn't destroy the instance of I2C bus manager
*/
void lwis_i2c_bus_manager_disconnect(struct lwis_device *lwis_dev)
{
	struct lwis_i2c_bus_manager *i2c_bus_manager;
	struct lwis_i2c_connected_device *connected_i2c_device;
	struct list_head *i2c_connected_device_node, *i2c_connected_device_tmp_node;
	struct lwis_i2c_device *i2c_dev = NULL;

	i2c_bus_manager = lwis_i2c_bus_manager_get_manager(lwis_dev);
	if (!i2c_bus_manager) {
		return;
	}

	list_for_each_safe (i2c_connected_device_node, i2c_connected_device_tmp_node,
			    &i2c_bus_manager->i2c_connected_devices) {
		connected_i2c_device =
			list_entry(i2c_connected_device_node, struct lwis_i2c_connected_device,
				   connected_device_node);
		/* Reset the bus manager pointer for this i2c device */
		i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
		i2c_dev->i2c_bus_manager = NULL;

		if (connected_i2c_device->connected_device == lwis_dev) {
			list_del(&connected_i2c_device->connected_device_node);
			kfree(connected_i2c_device);
			connected_i2c_device = NULL;
			--i2c_bus_manager->number_of_connected_devices;

			/* Destroy the bus manager instance if there
			 * are no more I2C devices connected to it
			 */
			if (i2c_bus_manager->number_of_connected_devices == 0) {
				destroy_i2c_bus_manager(i2c_bus_manager, lwis_dev);
			}
			return;
		}
	}
}

/* lwis_i2c_bus_manager_lock_i2c_bus:
 * Locks the I2C bus for a given I2C Lwis Device
 */
void lwis_i2c_bus_manager_lock_i2c_bus(struct lwis_device *lwis_dev)
{
	struct lwis_i2c_bus_manager *i2c_bus_manager = NULL;
	i2c_bus_manager = lwis_i2c_bus_manager_get_manager(lwis_dev);
	if (i2c_bus_manager) {
		mutex_lock(&i2c_bus_manager->i2c_bus_lock);
		if (lwis_i2c_bus_manager_debug) {
			dev_info(lwis_dev->dev, "%s lock\n", i2c_bus_manager->i2c_bus_name);
		}
	}
}

/* lwis_i2c_bus_manager_unlock_i2c_bus:
 * Unlocks the I2C bus for a given I2C Lwis Device
 */
void lwis_i2c_bus_manager_unlock_i2c_bus(struct lwis_device *lwis_dev)
{
	struct lwis_i2c_bus_manager *i2c_bus_manager = NULL;
	i2c_bus_manager = lwis_i2c_bus_manager_get_manager(lwis_dev);
	if (i2c_bus_manager) {
		if (lwis_i2c_bus_manager_debug) {
			dev_info(lwis_dev->dev, "%s unlock\n", i2c_bus_manager->i2c_bus_name);
		}
		mutex_unlock(&i2c_bus_manager->i2c_bus_lock);
	}
}

/* lwis_i2c_bus_managlwis_i2c_bus_manager_get_managerr_get:
 * Gets I2C Bus Manager for a given lwis device
 */
struct lwis_i2c_bus_manager *lwis_i2c_bus_manager_get_manager(struct lwis_device *lwis_dev)
{
	struct lwis_i2c_device *i2c_dev = NULL;
	if (lwis_check_device_type(lwis_dev, DEVICE_TYPE_I2C)) {
		i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
		if (i2c_dev) {
			return i2c_dev->i2c_bus_manager;
		}
	}
	return NULL;
}

/* lwis_i2c_bus_manager_flush_i2c_worker:
 * Flushes the I2C Bus Manager worker
 */
void lwis_i2c_bus_manager_flush_i2c_worker(struct lwis_device *lwis_dev)
{
	struct lwis_i2c_bus_manager *i2c_bus_manager = lwis_i2c_bus_manager_get_manager(lwis_dev);

	if (i2c_bus_manager == NULL)
		return;

	kthread_flush_worker(&i2c_bus_manager->i2c_bus_worker);
}

/* lwis_i2c_bus_manager_list_initialize:
 * Initializes bus manager global list. This is the list that holds
 * actual bus manager pointers for a given physical I2C Bus connection
 */
void lwis_i2c_bus_manager_list_initialize(void)
{
	/* initialize_i2c_bus_manager_list */
	mutex_init(&i2c_bus_manager_list_lock);
	INIT_LIST_HEAD(&i2c_bus_manager_list.i2c_bus_manager_list_head);
}

/* lwis_i2c_bus_manager_list_deinitialize:
 * Deinitializes bus manager global list
 */
void lwis_i2c_bus_manager_list_deinitialize(void)
{
	struct list_head *i2c_bus_manager_list_node, *i2c_bus_manager_list_tmp_node;
	struct lwis_i2c_bus_manager_identifier *i2c_bus_manager_identifier;

	/* deinitialize_i2c_bus_manager_list */
	mutex_lock(&i2c_bus_manager_list_lock);
	list_for_each_safe (i2c_bus_manager_list_node, i2c_bus_manager_list_tmp_node,
			    &i2c_bus_manager_list.i2c_bus_manager_list_head) {
		i2c_bus_manager_identifier = list_entry(i2c_bus_manager_list_node,
							struct lwis_i2c_bus_manager_identifier,
							i2c_bus_manager_list_node);
		i2c_bus_manager_identifier->i2c_bus_manager = NULL;
		list_del(&i2c_bus_manager_identifier->i2c_bus_manager_list_node);
		kfree(i2c_bus_manager_identifier);
		i2c_bus_manager_identifier = NULL;
	}
	mutex_unlock(&i2c_bus_manager_list_lock);
}

/* lwis_i2c_bus_manager_connect_client:
 * Connects a lwis client to the bus manager to be processed by the worker.
 * The client will be connected to the appropriate priority queue based
 * on the I2C device priority specified in the dts for the I2C device node.
 * I2C lwis client is always connected when a new instance of client is
 * created.
 */
int lwis_i2c_bus_manager_connect_client(struct lwis_client *connecting_client)
{
	int ret = 0;
	int device_priority = I2C_MAX_PRIORITY_LEVELS;
	bool create_client_node = true;
	struct lwis_i2c_process_request *i2c_connecting_client_node;
	struct lwis_device *lwis_dev = NULL;
	struct lwis_i2c_process_queue *process_queue = NULL;
	struct lwis_i2c_device *i2c_dev = NULL;
	struct lwis_i2c_bus_manager *i2c_bus_manager = NULL;
	struct list_head *request, *request_tmp;
	struct lwis_i2c_process_request *search_node;

	if (!connecting_client) {
		pr_err("Connecting client pointer for I2C Bus Manager is NULL\n");
		return -EINVAL;
	}

	lwis_dev = connecting_client->lwis_dev;
	if (!lwis_dev) {
		pr_err("Connecting device for I2C Bus Manager is NULL\n");
		return -EINVAL;
	}

	if (!lwis_check_device_type(lwis_dev, DEVICE_TYPE_I2C)) {
		return ret;
	}

	i2c_bus_manager = lwis_i2c_bus_manager_get_manager(lwis_dev);
	if (!i2c_bus_manager) {
		dev_err(lwis_dev->dev, "I2C bus manager is NULL\n");
		return -EINVAL;
	}

	i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
	if (!i2c_dev) {
		dev_err(lwis_dev->dev, "I2C device is NULL\n");
		return -EINVAL;
	}

	device_priority = i2c_dev->device_priority;
	if (!i2c_device_priority_is_valid(device_priority)) {
		dev_err(lwis_dev->dev, "Invalid I2C device priority %d\n", device_priority);
		return -EINVAL;
	}

	mutex_lock(&i2c_bus_manager->i2c_process_queue_lock);

	// Search for existing client node in the queue, if client is already connected
	// to this bus then don't create a new client node
	process_queue = &i2c_bus_manager->i2c_bus_process_queue[device_priority];
	if (!lwis_i2c_process_request_queue_is_empty(process_queue)) {
		list_for_each_safe (request, request_tmp, &process_queue->head) {
			search_node =
				list_entry(request, struct lwis_i2c_process_request, request_node);
			if (search_node->requesting_client == connecting_client) {
				dev_info(lwis_dev->dev,
					 "I2C client already connected %s(%p) to bus %s \n",
					 lwis_dev->name, connecting_client,
					 i2c_bus_manager->i2c_bus_name);
				create_client_node = false;
				break;
			}
		}
	}

	if (create_client_node) {
		i2c_connecting_client_node =
			kzalloc(sizeof(struct lwis_i2c_process_request), GFP_KERNEL);
		if (!i2c_connecting_client_node) {
			dev_err(lwis_dev->dev, "Failed to connect client to I2C Bus Manager\n");
			return -ENOMEM;
		}
		i2c_connecting_client_node->requesting_client = connecting_client;
		INIT_LIST_HEAD(&i2c_connecting_client_node->request_node);
		list_add_tail(&i2c_connecting_client_node->request_node, &process_queue->head);
		++process_queue->number_of_nodes;
		dev_info(lwis_dev->dev, "Connecting client %s(%p) to bus %s\n", lwis_dev->name,
			 connecting_client, i2c_bus_manager->i2c_bus_name);
		if (lwis_i2c_bus_manager_debug) {
			dev_info(lwis_dev->dev, "Adding process request %p\n",
				i2c_connecting_client_node);
		}
	}

	mutex_unlock(&i2c_bus_manager->i2c_process_queue_lock);
	return ret;
}

/* lwis_i2c_bus_manager_disconnect_client:
 * Disconnects a lwis client to the bus manager. This will make sure that
 * the released client is not processed further by the I2C worker.
 * The client will be disconnected from the appropriate priority queue based
 * on the I2C device priority specified in the dts for the I2C device node.
 * I2C lwis client is always disconnected when the instance of client is
 * released/destroyed.
 */
void lwis_i2c_bus_manager_disconnect_client(struct lwis_client *disconnecting_client)
{
	int device_priority = I2C_MAX_PRIORITY_LEVELS;
	struct lwis_i2c_process_request *i2c_disconnecting_client_node;
	struct lwis_device *lwis_dev = NULL;
	struct lwis_i2c_process_queue *process_queue = NULL;
	struct lwis_i2c_device *i2c_dev = NULL;
	struct list_head *request, *request_tmp;
	struct lwis_i2c_bus_manager *i2c_bus_manager = NULL;

	if (!disconnecting_client) {
		pr_err("Disconnecting client pointer for I2C Bus Manager is NULL\n");
		return;
	}

	lwis_dev = disconnecting_client->lwis_dev;
	if (!lwis_dev) {
		pr_err("Disconnecting device for I2C Bus Manager is NULL\n");
		return;
	}

	if (!lwis_check_device_type(lwis_dev, DEVICE_TYPE_I2C)) {
		return;
	}

	i2c_bus_manager = lwis_i2c_bus_manager_get_manager(lwis_dev);
	if (!i2c_bus_manager) {
		dev_err(lwis_dev->dev, "I2C bus manager is NULL\n");
		return;
	}

	i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
	if (!i2c_dev) {
		dev_err(lwis_dev->dev, "I2C device is NULL\n");
		return;
	}

	device_priority = i2c_dev->device_priority;
	if (!i2c_device_priority_is_valid(device_priority)) {
		dev_err(lwis_dev->dev, "Invalid I2C device priority %d\n", device_priority);
		return;
	}

	mutex_lock(&i2c_bus_manager->i2c_process_queue_lock);
	process_queue = &i2c_bus_manager->i2c_bus_process_queue[device_priority];
	list_for_each_safe (request, request_tmp, &process_queue->head) {
		i2c_disconnecting_client_node =
			list_entry(request, struct lwis_i2c_process_request, request_node);
		if (i2c_disconnecting_client_node->requesting_client == disconnecting_client) {
			dev_info(lwis_dev->dev, "Disconnecting I2C client %s(%p) from bus %s\n",
				 lwis_dev->name, disconnecting_client,
				 i2c_bus_manager->i2c_bus_name);
			list_del(&i2c_disconnecting_client_node->request_node);
			i2c_disconnecting_client_node->requesting_client = NULL;
			if (lwis_i2c_bus_manager_debug) {
				dev_info(lwis_dev->dev, "Freeing process request %p\n",
					i2c_disconnecting_client_node);
			}
			kfree(i2c_disconnecting_client_node);
			i2c_disconnecting_client_node = NULL;
			--process_queue->number_of_nodes;
			break;
		}
	}
	mutex_unlock(&i2c_bus_manager->i2c_process_queue_lock);
}