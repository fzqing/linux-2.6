/*
 * kernel/lspinfo.c
 *
 * Linux support package info to be exported in to /sys/vendor/mvista
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>
 *
 * sysfs code based on drivers/base/firmware.c, drivers/firmware/edd.c
 * and drivers/pci/pci-sysfs.c
 *
 * 2003-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/lspinfo.h>
#include <linux/lsppatchlevel.h>
#include <linux/mvl_patch.h>

#include <asm/bug.h>

struct mvista_info_dev {
	struct kobject kobj;
};

struct mvista_attribute {
	struct attribute attr;
	ssize_t(*show) (char *buf);
};

static struct mvista_info_dev *mvista_info_devs;

#define MVISTA_INFO_DEV_ATTR(_name)				\
static struct mvista_attribute mvista_info_dev_attr_##_name = __ATTR_RO(_name);

/*
 * Create a vendor top-level sysfs dir
 */
static decl_subsys(vendor, NULL, NULL);

static int vendor_register(struct subsystem * s)
{
	kset_set_kset_s(s, vendor_subsys);
	return subsystem_register(s);
}

static void vendor_unregister(struct subsystem * s)
{
	subsystem_unregister(s);
}

static int __init vendor_init(void)
{
	return subsystem_register(&vendor_subsys);
}

module_init(vendor_init);

/* Standard and very simple bits of information we need to export */
#define mvista_info_dev_attr(_type, _value)			\
static ssize_t							\
_type##_show(char *buf)						\
{								\
	return sprintf(buf, "%s\n", _value);			\
}
mvista_info_dev_attr(board_name, LSP_BOARD_NAME);
MVISTA_INFO_DEV_ATTR(board_name);
mvista_info_dev_attr(build_id, LSP_BUILD_ID);
MVISTA_INFO_DEV_ATTR(build_id);
mvista_info_dev_attr(name, LSP_NAME);
MVISTA_INFO_DEV_ATTR(name);
mvista_info_dev_attr(mvl_arch, LSP_MVL_ARCH);
MVISTA_INFO_DEV_ATTR(mvl_arch);
mvista_info_dev_attr(revision, LSP_REVISION);
MVISTA_INFO_DEV_ATTR(revision);
mvista_info_dev_attr(lsp_patch_level, LSP_PATCH_LEVEL);
MVISTA_INFO_DEV_ATTR(lsp_patch_level);

static ssize_t
summary_show(char *buf)
{
	return sprintf(buf,
			"Board Name		: %s\n"
			"Lsp Name		: %s\n"
			"LSP Revision		: %s.%s\n"
			"MVL Architecture	: %s\n"
			"Patch Level		: %s\n",
			LSP_BOARD_NAME, LSP_NAME,
			LSP_BUILD_ID, LSP_REVISION, LSP_MVL_ARCH,
			LSP_PATCH_LEVEL);
}
MVISTA_INFO_DEV_ATTR(summary);

/* Infrastructure for listing patches which have been applied post
 * release.
 */
typedef struct mvl_patchlist_s
{
	int                    common;
	struct mvl_patchlist_s *next;
} mvl_patchlist_t;

static mvl_patchlist_t *mvl_patchlist, *mvl_patchlist_tail;

static ssize_t show_lsp_patches(struct kobject *kobj, char *buf, loff_t off,
				size_t count);

static struct bin_attribute lsp_patches = {
	.attr = {
		.name = "lsp_patches",
		.mode = 0444,
		.owner = THIS_MODULE,
	},
	.size = 0,
	.read = show_lsp_patches,
};

int
mvl_register_patch(int common)
{
	mvl_patchlist_t *p;

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->common = common;
	p->next = NULL;
	if (mvl_patchlist) {
		mvl_patchlist_tail->next = p;
		mvl_patchlist_tail = p;
	} else {
		mvl_patchlist = p;
		mvl_patchlist_tail = p;
	}
	lsp_patches.size += 5;

	return 0;
}
EXPORT_SYMBOL(mvl_register_patch);

static ssize_t
show_lsp_patches(struct kobject *kobj, char *buf, loff_t off, size_t count)
{
	int             len = 0;
	mvl_patchlist_t *p = mvl_patchlist;
	char            name[6];
	off_t           left = off;

	while (p && left) {
		int l = snprintf(name, sizeof(name), "%4.4d\n", p->common);
		p = p->next;
		if (l > left) {
			len = l - left;
			if (len > count)
				len = count;
			memcpy(buf, name+left, len);
			count -= len;
			buf += len;
			break;
		}
		left -= l;
	}

	while (p && count) {
		int l = snprintf(name, sizeof(name), "%4.4d\n", p->common);
		p = p->next;
		if (l > count) {
			memcpy(buf, name, count);
			len += count;
			count = 0;
		} else {
			memcpy(buf, name, l);
			count -= l;
			len += l;
			buf += l;
		}
	}

	return len;
}

/* On top of that create an mvista dir to register all of our things
 * into.
 */
#define to_mvista_info_dev_attr(_attr)				\
	container_of(_attr, struct mvista_attribute, attr)

static ssize_t
mvista_info_dev_attr_show(struct kobject * kobj, struct attribute *attr,
		char *buf)
{
	struct mvista_attribute *mvista_info_dev_attr =
		to_mvista_info_dev_attr(attr);

	return mvista_info_dev_attr->show(buf);
}

static struct sysfs_ops mvista_info_dev_attr_ops = {
	.show = mvista_info_dev_attr_show,
};

/* Theser are the default attributes that exist on every LSP and are
 * unchanged from initial release. */
static struct attribute * def_attrs[] = {
	&mvista_info_dev_attr_board_name.attr,
	&mvista_info_dev_attr_build_id.attr,
	&mvista_info_dev_attr_name.attr,
	&mvista_info_dev_attr_mvl_arch.attr,
	&mvista_info_dev_attr_revision.attr,
	&mvista_info_dev_attr_summary.attr,
	&mvista_info_dev_attr_lsp_patch_level.attr,
	NULL,
};

static struct kobj_type ktype_mvista = {
	.sysfs_ops	= &mvista_info_dev_attr_ops,
	.default_attrs	= def_attrs,
};

static decl_subsys(mvista, &ktype_mvista, NULL);

static int __init
mvista_info_dev_init(void)
{
	int rc;

	rc = vendor_register(&mvista_subsys);
	if (rc)
		return rc;

	if (!(mvista_info_devs = kmalloc(sizeof (struct mvista_info_dev),
					GFP_KERNEL)))
		return -ENOMEM;

	memset(mvista_info_devs, 0, sizeof (struct mvista_info_dev));
	kobject_set_name(&mvista_info_devs->kobj, "lspinfo");
	kobj_set_kset_s(mvista_info_devs, mvista_subsys);
	rc = kobject_register(&mvista_info_devs->kobj);

	if (rc)
		goto error;

	rc = sysfs_create_bin_file(&mvista_info_devs->kobj, &lsp_patches);

	if (rc)
		goto error;

	return 0;
error:
	kfree(mvista_info_devs);
	vendor_unregister(&mvista_subsys);
	return rc;
}

late_initcall(mvista_info_dev_init);
