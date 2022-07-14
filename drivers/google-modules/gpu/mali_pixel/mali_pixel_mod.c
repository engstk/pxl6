// SPDX-License-Identifier: GPL-2.0

#include "mali_pixel_mod.h"
#include <linux/module.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Pixel platform integration for GPU");
MODULE_AUTHOR("<sidaths@google.com>");
MODULE_VERSION("1.0");
MODULE_SOFTDEP("pre: pixel_stat_sysfs");
MODULE_SOFTDEP("pre: slc_pmon");
MODULE_SOFTDEP("pre: slc_dummy");
MODULE_SOFTDEP("pre: slc_acpm");

extern struct kobject *pixel_stat_kobj;

struct kobject *pixel_stat_gpu_kobj;

static int mali_pixel_init_pixel_stats(void)
{
	struct kobject *pixel_stat = pixel_stat_kobj;

	WARN_ON(pixel_stat_kobj == NULL);

	pixel_stat_gpu_kobj = kobject_create_and_add("gpu", pixel_stat);
	if (!pixel_stat_gpu_kobj)
		return -ENOMEM;

	return 0;
}

static int __init mali_pixel_init(void)
{
	int ret = 0;

	/* The Pixel Stats Sysfs module needs to be loaded first */
	if (pixel_stat_kobj == NULL)
		return -EPROBE_DEFER;

	ret = mali_pixel_init_pixel_stats();
	if (ret)
		goto fail_pixel_stats;

#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER
	ret = platform_driver_register(&memory_group_manager_driver);
#endif
	if (ret)
		goto fail_mgm;

#ifdef CONFIG_MALI_PRIORITY_CONTROL_MANAGER
	ret = platform_driver_register(&priority_control_manager_driver);
#else
#endif
	if (ret)
		goto fail_pcm;

	goto exit;

fail_pcm:
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER
	platform_driver_unregister(&memory_group_manager_driver);
#endif

fail_pixel_stats:
	/* nothing to clean up here */
fail_mgm:
	/* nothing to clean up here */

exit:
	return ret;
}
module_init(mali_pixel_init);

static void __exit mali_pixel_exit(void)
{
#ifdef CONFIG_MALI_PRIORITY_CONTROL_MANAGER
	platform_driver_unregister(&priority_control_manager_driver);
#endif
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER
	platform_driver_unregister(&memory_group_manager_driver);
#endif
}
module_exit(mali_pixel_exit);
