/*
 * Author (legacy sysfs): Chad Froebel <chadfroebel@gmail.com>
 *
 * Additional fixes, port to raviole and power supply hooks: engstk <eng.stk@sapo.pt>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Possible values for "force_fast_charge" are :
 *
 *   0 - Disabled (default)
 *   1 - Force faster charge
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/module.h>

int force_fast_charge = 0;
EXPORT_SYMBOL(force_fast_charge);

static ssize_t force_fast_charge_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", force_fast_charge);
}

static ssize_t force_fast_charge_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &force_fast_charge);
	if (force_fast_charge < 0 || force_fast_charge > 1)
		force_fast_charge = 0;
	return count;
}

static struct kobj_attribute force_fast_charge_attribute =
__ATTR(force_fast_charge, 0644, force_fast_charge_show, force_fast_charge_store);

static struct attribute *attrs[] = {
&force_fast_charge_attribute.attr,
NULL,
};

static struct attribute_group attr_group = {
.attrs = attrs,
};

static struct kobject *force_fast_charge_kobj;

int force_fast_charge_init(void)
{
	int retval;
        force_fast_charge_kobj = kobject_create_and_add("fast_charge", kernel_kobj);
        if (!force_fast_charge_kobj) {
                return -ENOMEM;
        }
        retval = sysfs_create_group(force_fast_charge_kobj, &attr_group);
        if (retval)
                kobject_put(force_fast_charge_kobj);
        return retval;
}

void force_fast_charge_exit(void)
{
	kobject_put(force_fast_charge_kobj);
}

module_init(force_fast_charge_init);
module_exit(force_fast_charge_exit);
