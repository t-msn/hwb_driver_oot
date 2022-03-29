// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 FUJITSU LIMITED
 */

#include <asm/cputype.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/wait.h>

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[%s:%s:%d] " fmt, KBUILD_MODNAME, __func__, __LINE__

/* Since miscdevice is used, /dev/hard_barrier will be created when module is loaded */
#define DEV_NAME "hard_barrier"

/* Implementation defined registers for barrier shared in CMG */
#define FHWB_INIT_SYNC_BB0_EL1  sys_reg(3, 0, 15, 13, 0)
#define FHWB_INIT_SYNC_BB1_EL1  sys_reg(3, 0, 15, 13, 1)
#define FHWB_INIT_SYNC_BB2_EL1  sys_reg(3, 0, 15, 13, 2)
#define FHWB_INIT_SYNC_BB3_EL1  sys_reg(3, 0, 15, 13, 3)
#define FHWB_INIT_SYNC_BB4_EL1  sys_reg(3, 0, 15, 13, 4)
#define FHWB_INIT_SYNC_BB5_EL1  sys_reg(3, 0, 15, 13, 5)

/* Implementation defined registers for barrier per PE */
#define FHWB_CTRL_EL1           sys_reg(3, 0, 11, 12, 0)
#define FHWB_BST_BIT_EL1        sys_reg(3, 0, 11, 12, 4)
#define FHWB_ASSIGN_SYNC_W0_EL1 sys_reg(3, 0, 15, 15, 0)
#define FHWB_ASSIGN_SYNC_W1_EL1 sys_reg(3, 0, 15, 15, 1)
#define FHWB_ASSIGN_SYNC_W2_EL1 sys_reg(3, 0, 15, 15, 2)
#define FHWB_ASSIGN_SYNC_W3_EL1 sys_reg(3, 0, 15, 15, 3)

/* Field definitions for above registers */
#define FHWB_INIT_SYNC_BB_EL1_MASK_FIELD  GENMASK_ULL(44, 32)
#define FHWB_INIT_SYNC_BB_EL1_BST_FIELD   GENMASK_ULL(12, 0)
#define FHWB_CTRL_EL1_EL1AE               BIT_ULL(63)
#define FHWB_CTRL_EL1_EL0AE               BIT_ULL(62)
#define FHWB_BST_BIT_EL1_CMG_FILED        GENMASK_ULL(5, 4)
#define FHWB_BST_BIT_EL1_PE_FILED         GENMASK_ULL(3, 0)
#define FHWB_ASSIGN_SYNC_W_EL1_VALID      BIT_ULL(63)

static enum cpuhp_state _hp_state;

/*
 * Each PE has its own CMG and Physical PE number (determined by BST_BIT_EL1 register).
 * Barrier operation can be performed by PEs which belong to the same CMG.
 */
struct pe_info {
	/* CMG number of this PE */
	u8 cmg;
	/* Physical PE number of this PE */
	u8 ppe;
};

/* Hardware information of running system */
struct hwb_hwinfo {
	/* CPU type (part number) */
	unsigned int type;
	/* Number of CMG */
	u8 num_cmg;
	/* Number of barrier blade(BB) per CMG */
	u8 num_bb;
	/* Number of barrier window(BW) per PE */
	u8 num_bw;
	/*
	 * Maximum number of PE per CMG.
	 * Depending on BIOS configuration, each CMG has up to max_pe_per_cmg PEs
	 * and each PE has unique physical PE number between 0 ~ (max_pe_per_cmg-1)
	 */
	u8 max_pe_per_cmg;

	/* Bitmap for currently allocated BB per CMG */
	unsigned long *used_bb_bmap;
	/* Bitmap for currently allocated BW per PE */
	unsigned long *used_bw_bmap;
	/* Mapping table of cpuid -> CMG/PE number */
	struct pe_info *core_map;

	/* Currently used BW per PE */
	atomic_t *used_bw_count;
};
static struct hwb_hwinfo _hwinfo;

/* List for barrier blade currently used */
struct hwb_private_data {
	struct list_head bb_list;
	spinlock_t list_lock;
};
static struct hwb_private_data *pdata;

/* Each barrier blade info */
#define BB_FREEING 1
struct bb_info {
	/* cpumask for PEs which participate synchronization */
	cpumask_var_t pemask;
	/* cpumask for PEs which currently assigned BW for this BB */
	cpumask_var_t assigned_pemask;
	/* Added to hwb_private_data::bb_list */
	struct list_head node;
	/* For indicating if this bb is currently being freed or not */
	unsigned long flag;
	/* For waiting ongoing assign/unassign operation to finish before freeing BB */
	wait_queue_head_t wq;
	/* Track ongoing assign/unassign operation count */
	atomic_t ongoing_assign_count;
	/* CMG  number of this blade */
	u8 cmg;
	/* BB number of this blade */
	u8 bb;
	/* Hold assigned window number of each PE corresponding to @assigned_pemask */
	u8 *bw;
	/* Track usage count as IOC_BB_FREE and IOC_BW_[UN]ASSIGN might be run in parallel */
	struct kref kref;
	/* pid holding this bb */
	pid_t pid;
};
static struct kmem_cache *bb_info_cachep;

static void free_bb_info(struct kref *kref)
{
	struct bb_info *bb_info = container_of(kref, struct bb_info, kref);

	free_cpumask_var(bb_info->assigned_pemask);
	free_cpumask_var(bb_info->pemask);
	kfree(bb_info->bw);
	kmem_cache_free(bb_info_cachep, bb_info);
}

static struct bb_info *alloc_bb_info(void)
{
	struct bb_info *bb_info;

	bb_info = kmem_cache_zalloc(bb_info_cachep, GFP_KERNEL);
	if (!bb_info)
		return NULL;

	bb_info->bw = kcalloc(_hwinfo.max_pe_per_cmg, sizeof(u8), GFP_KERNEL);
	if (!bb_info->bw) {
		free_bb_info(&bb_info->kref);
		return NULL;
	}
	if (!zalloc_cpumask_var(&bb_info->pemask, GFP_KERNEL) ||
		!zalloc_cpumask_var(&bb_info->assigned_pemask, GFP_KERNEL)) {
		free_bb_info(&bb_info->kref);
		return NULL;
	}

	init_waitqueue_head(&bb_info->wq);
	kref_init(&bb_info->kref);

	return bb_info;
}

static struct bb_info *get_bb_info(u8 cmg, u8 bb)
{
	struct bb_info *bb_info;

	if (cmg >= _hwinfo.num_cmg || bb >= _hwinfo.num_bb) {
		pr_err("CMG/BB number is invalid: %u/%u\n", cmg, bb);
		return ERR_PTR(-EINVAL);
	}

	if (!test_bit(bb, &_hwinfo.used_bb_bmap[cmg])) {
		pr_err("BB is not allocated: %u/%u\n", cmg, bb);
		return ERR_PTR(-ENOENT);
	}

	spin_lock(&pdata->list_lock);
	list_for_each_entry(bb_info, &pdata->bb_list, node) {
		if (bb_info->cmg == cmg && bb_info->bb == bb) {
			if (test_bit(BB_FREEING,  &bb_info->flag)) {
				pr_err("BB is currently being freed: %u/%u\n", cmg, bb);
				spin_unlock(&pdata->list_lock);
				return ERR_PTR(-EPERM);
			}

			kref_get(&bb_info->kref);
			spin_unlock(&pdata->list_lock);
			return bb_info;
		}
	}
	spin_unlock(&pdata->list_lock);

	pr_err("BB is not allocated by this process: %u/%u\n", cmg, bb);
	return ERR_PTR(-EPERM);
}

static inline void put_bb_info(struct bb_info *bb_info)
{
	kref_put(&bb_info->kref, free_bb_info);
}

/* Validate pemask's range and convert it to a mask based on physical PE number */
static int validate_and_convert_pemask(struct bb_info *bb_info, unsigned long *phys_pemask)
{
	int cpu;
	u8 cmg;

	if (cpumask_weight(bb_info->pemask) < 2) {
		pr_err("pemask needs at least two bit set: %*pbl\n",
						cpumask_pr_args(bb_info->pemask));
		return -EINVAL;
	}

	if (!cpumask_subset(bb_info->pemask, cpu_online_mask)) {
		pr_err("pemask needs to be subset of online cpu: %*pbl, %*pbl\n",
			cpumask_pr_args(bb_info->pemask), cpumask_pr_args(cpu_online_mask));
		return -EINVAL;
	}

	/*
	 * INIT_SYNC register requires a mask value based on physical PE number.
	 * So convert pemask to it while checking if all PEs belongs to the same CMG
	 */
	cpu = cpumask_first(bb_info->pemask);
	cmg = _hwinfo.core_map[cpu].cmg;
	*phys_pemask = 0;
	for_each_cpu(cpu, bb_info->pemask) {
		if (_hwinfo.core_map[cpu].cmg != cmg) {
			pr_err("All PEs must belong to the same CMG: %*pbl\n",
							cpumask_pr_args(bb_info->pemask));
			return -EINVAL;
		}
		set_bit(_hwinfo.core_map[cpu].ppe, phys_pemask);
	}
	bb_info->cmg = cmg;

	pr_debug("pemask: %*pbl, physical_pemask: %lx\n",
					cpumask_pr_args(bb_info->pemask), *phys_pemask);

	return 0;
}

struct init_sync_args {
	u64 val;
	u8 bb;
};

static void write_init_sync_reg(void *args)
{
	struct init_sync_args *sync_args = (struct init_sync_args *)args;

	switch (sync_args->bb) {
	case 0:
		write_sysreg_s(sync_args->val, FHWB_INIT_SYNC_BB0_EL1);
		break;
	case 1:
		write_sysreg_s(sync_args->val, FHWB_INIT_SYNC_BB1_EL1);
		break;
	case 2:
		write_sysreg_s(sync_args->val, FHWB_INIT_SYNC_BB2_EL1);
		break;
	case 3:
		write_sysreg_s(sync_args->val, FHWB_INIT_SYNC_BB3_EL1);
		break;
	case 4:
		write_sysreg_s(sync_args->val, FHWB_INIT_SYNC_BB4_EL1);
		break;
	case 5:
		write_sysreg_s(sync_args->val, FHWB_INIT_SYNC_BB5_EL1);
		break;
	}
}

/* Send IPI to initialize INIT_SYNC register */
static void setup_bb(struct bb_info *bb_info, unsigned long phys_pemask)
{
	struct init_sync_args args = {0};
	int cpu;

	/* INIT_SYNC register is shared resource in CMG. Pick one PE to set it up */
	cpu = cpumask_any(bb_info->pemask);

	args.bb = bb_info->bb;
	args.val = FIELD_PREP(FHWB_INIT_SYNC_BB_EL1_MASK_FIELD, phys_pemask);
	on_each_cpu_mask(cpumask_of(cpu), write_init_sync_reg, &args, 1);

	pr_debug("Setup bb. cpu: %d, CMG: %u, BB: %u, bimtap: %lx\n",
			cpu, bb_info->cmg, bb_info->bb, _hwinfo.used_bb_bmap[bb_info->cmg]);
}

static bool is_bound_only_one_pe(void)
{
	if (current->nr_cpus_allowed == 1)
		return true;

	pr_err("Thread must be bound to one PE between assign and unassign\n");
	return false;
}

/* Check if this PE can be assignable and set window number to be used to @window */
static int is_bw_assignable(struct bb_info *bb_info, int *window, int cpu)
{
	int i;

	if (test_bit(BB_FREEING, &bb_info->flag)) {
		pr_err("BB is currently being freed: %u/%u/%d\n", bb_info->cmg, bb_info->bb, cpu);
		return -EPERM;
	}

	if (!cpumask_test_cpu(cpu, bb_info->pemask)) {
		pr_err("This pe is not supposed to join sync, %u/%u/%d\n",
						bb_info->cmg, bb_info->bb, cpu);
		return -EINVAL;
	}

	if (cpumask_test_cpu(cpu, bb_info->assigned_pemask)) {
		pr_err("This pe is already assigned to window: %u/%u/%d\n",
						bb_info->cmg, bb_info->bb, cpu);
		return -EINVAL;
	}

	// search available window
	i = ffz(_hwinfo.used_bw_bmap[cpu]);
	if (i == _hwinfo.num_bw) {
		pr_err("There is no free window: %u/%u/%d\n",
				bb_info->cmg, bb_info->bb, cpu);
		return -EBUSY;
	}

	*window = i;

	return 0;
}

static void setup_ctl_reg(struct bb_info *bb_info, int cpu)
{
	u64 val;

	if (_hwinfo.used_bw_bmap[cpu] != 0)
		/* Already setup. Nothing todo */
		return;

	/*
	 * This is the first assign on this PE.
	 * Setup ctrl reg to allow access to BST_SYNC/LBSY_SYNC from EL0
	 */
	val = (FHWB_CTRL_EL1_EL1AE | FHWB_CTRL_EL1_EL0AE);
	write_sysreg_s(val, FHWB_CTRL_EL1);

	pr_debug("Setup ctl reg. cpu: %d\n", cpu);
}

static void write_bw_reg(u8 window, u64 val)
{
	switch (window) {
	case 0:
		write_sysreg_s(val, FHWB_ASSIGN_SYNC_W0_EL1);
		break;
	case 1:
		write_sysreg_s(val, FHWB_ASSIGN_SYNC_W1_EL1);
		break;
	case 2:
		write_sysreg_s(val, FHWB_ASSIGN_SYNC_W2_EL1);
		break;
	case 3:
		write_sysreg_s(val, FHWB_ASSIGN_SYNC_W3_EL1);
		break;
	}
}

static void setup_bw(struct bb_info *bb_info, int window, int cpu)
{
	u64 val;
	u8 ppe;

	/* Set valid bit and bb number */
	val = (FHWB_ASSIGN_SYNC_W_EL1_VALID | bb_info->bb);
	write_bw_reg(window, val);

	/* Update bitmap info */
	ppe = _hwinfo.core_map[cpu].ppe;
	set_bit(window, &_hwinfo.used_bw_bmap[cpu]);
	cpumask_set_cpu(cpu, bb_info->assigned_pemask);
	bb_info->bw[ppe] = window;

	pr_debug("Setup bw. cpu: %d, window: %u, BB: %u, bw_bmap: %lx, assigned_pemask: %*pbl\n",
			cpu, window, bb_info->bb,
			_hwinfo.used_bw_bmap[cpu], cpumask_pr_args(bb_info->assigned_pemask));
}

static int is_bw_unassignable(struct bb_info *bb_info, int cpu)
{
	u8 ppe;

	if (test_bit(BB_FREEING, &bb_info->flag)) {
		pr_err("This bb is currently being freed: %u/%u/%d\n",
							bb_info->cmg, bb_info->bb, cpu);
		return -EPERM;
	}

	if (!cpumask_test_and_clear_cpu(cpu, bb_info->assigned_pemask)) {
		pr_err("This pe is not assigned: %u/%u/%d\n", bb_info->cmg, bb_info->bb, cpu);
		return -EINVAL;
	}

	ppe = _hwinfo.core_map[cpu].ppe;
	if (!test_bit(bb_info->bw[ppe], &_hwinfo.used_bw_bmap[cpu])) {
		/* should not happen */
		pr_crit("Logic error. This window is not assigned: %u/%u/%d\n",
							bb_info->cmg, bb_info->bb, cpu);
		return -EINVAL;
	}

	return 0;
}

static void teardown_ctl_reg(struct bb_info *bb_info, int cpu)
{
	if (_hwinfo.used_bw_bmap[cpu] != 0)
		/* Other window on this PE is still in use. Nothing todo */
		return;

	/*
	 * This is the last unassign on this PE.
	 * Clear all bits to disallow access to BST_SYNC/LBSY_SYNC from EL0
	 */
	write_sysreg_s(0, FHWB_CTRL_EL1);

	pr_debug("Teardown ctl reg. cpu: %d\n", cpu);
}

static void teardown_bw(struct bb_info *bb_info, int cpu)
{
	u8 window;
	u8 ppe;

	/* Just clear all bits */
	ppe = _hwinfo.core_map[cpu].ppe;
	window = bb_info->bw[ppe];
	write_bw_reg(window, 0);

	/* Update bitmap info */
	clear_bit(window, &_hwinfo.used_bw_bmap[cpu]);
	bb_info->bw[ppe] = -1;

	pr_debug("Teardown bw. cpu: %d, window: %u, BB: %u, bw_bmap: %lx, assigned_pemask: %*pbl\n",
			cpu, window, bb_info->bb,
			_hwinfo.used_bw_bmap[cpu], cpumask_pr_args(bb_info->assigned_pemask));
}

static void cleanup_bw_func(void *args)
{
	struct bb_info *bb_info = (struct bb_info *)args;
	int cpu = smp_processor_id();

	teardown_bw(bb_info, cpu);
	teardown_ctl_reg(bb_info, cpu);
}

/* Send IPI to reset INIT_SYNC register */
static void teardown_bb(struct bb_info *bb_info)
{
	struct init_sync_args args = {0};
	int cpu;

	/* Reset BW on each PE if unassign is not called properly  */
	if (cpumask_weight(bb_info->assigned_pemask) != 0) {
		pr_warn("unassign is not called properly. CMG: %d, BB: %d, unassigned PE: %*pbl\n",
			bb_info->cmg, bb_info->bb, cpumask_pr_args(bb_info->assigned_pemask));
		on_each_cpu_mask(bb_info->assigned_pemask, cleanup_bw_func, bb_info, 1);
	}

	/* INIT_SYNC register is shared resource in CMG. Pick one PE */
	cpu = cpumask_any(bb_info->pemask);

	args.bb = bb_info->bb;
	/* Just clear all bits */
	args.val = 0;
	on_each_cpu_mask(cpumask_of(cpu), write_init_sync_reg, &args, 1);

	clear_bit(bb_info->bb, &_hwinfo.used_bb_bmap[bb_info->cmg]);

	for (cpu = 0; cpu < num_possible_cpus(); cpu++) {
		if (cpumask_test_cpu(cpu, bb_info->pemask))
			atomic_dec(&_hwinfo.used_bw_count[cpu]);
	}

	pr_debug("Teardown bb: cpu: %d, CMG: %u, BB: %u, bitmap: %lx\n",
			cpu, bb_info->cmg, bb_info->bb, _hwinfo.used_bb_bmap[bb_info->cmg]);
}


static struct miscdevice bar_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.mode  = 0666,
	.name  = DEV_NAME,
};

static void destroy_bb_info_cachep(void)
{
	kmem_cache_destroy(bb_info_cachep);
}

static int __init init_bb_info_cachep(void)
{
	/*
	 * Since cpumask value will be copied from userspace to the beginning of
	 * struct bb_info, use kmem_cache_create_usercopy to mark that region.
	 * Otherwise CONFIG_HARDENED_USERCOPY gives user_copy_warn.
	 */
	bb_info_cachep = kmem_cache_create_usercopy("bb_info_cache", sizeof(struct bb_info),
			0, SLAB_HWCACHE_ALIGN, 0, sizeof(cpumask_var_t), NULL);
	if (bb_info_cachep == NULL)
		return -ENOMEM;

	return 0;
}

static void free_map(void)
{
	kfree(_hwinfo.used_bw_bmap);
	kfree(_hwinfo.used_bb_bmap);
	kfree(_hwinfo.used_bw_count);
	kfree(_hwinfo.core_map);
}

static int __init alloc_map(void)
{
	_hwinfo.core_map = kcalloc(num_possible_cpus(), sizeof(struct pe_info), GFP_KERNEL);
	_hwinfo.used_bb_bmap = kcalloc(_hwinfo.num_cmg, sizeof(unsigned long), GFP_KERNEL);
	_hwinfo.used_bw_bmap = kcalloc(num_possible_cpus(), sizeof(unsigned long), GFP_KERNEL);
	_hwinfo.used_bw_count = kcalloc(num_possible_cpus(), sizeof(atomic_t), GFP_KERNEL);
	if (!_hwinfo.core_map || !_hwinfo.used_bb_bmap || !_hwinfo.used_bw_bmap || !_hwinfo.used_bw_count)
		goto fail;

	/* 0 is valid number for both CMG/PE. Set all bits to 1 to represents uninitialized state */
	memset(_hwinfo.core_map, 0xFF, sizeof(struct pe_info) * num_possible_cpus());

	return 0;

fail:
	free_map();
	return -ENOMEM;
}

/* Get this system's CPU type (part number). If it is not fujitsu CPU, return -1 */
static int __init get_cpu_type(void)
{
	if (read_cpuid_implementor() != ARM_CPU_IMP_FUJITSU)
		return -1;

	return read_cpuid_part_number();
}

static int __init setup_hwinfo(void)
{
	int type;

	type = get_cpu_type();
	if (type < 0)
		return -ENODEV;

	_hwinfo.type = type;
	switch (type) {
	case FUJITSU_CPU_PART_A64FX:
		_hwinfo.num_cmg = 4;
		_hwinfo.num_bb = 6;
		_hwinfo.num_bw = 4;
		// XXX: if support assistant core
		//_hwinfo.max_pe_per_cmg = 13;
		_hwinfo.max_pe_per_cmg = 12;
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

static int hwb_cpu_online(unsigned int cpu)
{
	u64 val;
	int i;

	/* Setup core_map by reading BST_BIT_EL1 register of each PE */
	val = read_sysreg_s(FHWB_BST_BIT_EL1);
	_hwinfo.core_map[cpu].cmg = FIELD_GET(FHWB_BST_BIT_EL1_CMG_FILED, val);
	_hwinfo.core_map[cpu].ppe = FIELD_GET(FHWB_BST_BIT_EL1_PE_FILED, val);

	/* Since these registers' values are UNKNOWN on reset, explicitly clear all */
	for (i = 0; i < _hwinfo.num_bw; i++)
		write_bw_reg(i, 0);

	write_sysreg_s(0, FHWB_CTRL_EL1);

	return 0;
}

struct hwb_attr {
	struct kobj_attribute attr;
	u8 bb;
};
/* kobject for each CMG */
static struct kobject **cmg_kobj;
static struct kobject ***bb_kobj;

/* Get CMG number based on index value of cmg_kobj */
static int get_cmg_from_kobj(struct kobject *kobj)
{
	int i;

	for (i = 0; i < _hwinfo.num_cmg; i++) {
		if (cmg_kobj[i] == kobj)
			return i;
	}
	/* should not happen */
	WARN_ON_ONCE("cmg_kobj not found\n");
	return 0;
}

/* Get CMG number based on index value of cmg_kobj */
static int get_bb_from_kobj(struct kobject *kobj)
{
	int i;
	int cmg;

	cmg = get_cmg_from_kobj(kobj->parent);

	for (i = 0; i < _hwinfo.num_bb; i++) {
		if (bb_kobj[cmg][i] == kobj)
			return i;
	}
	/* should not happen */
	WARN_ON_ONCE("bb_kobj not found\n");
	return 0;
}

#define BARRIER_ATTR(name) \
static struct kobj_attribute hwb_##name##_attribute = \
	__ATTR(name, 0444, hwb_##name##_show, NULL)

static ssize_t hwb_available_cpus_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	cpumask_var_t mask;
	ssize_t written = 0;
	int cmg;
	int cpu;

	if (!zalloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	cmg = get_cmg_from_kobj(kobj);
	for (cpu = 0; cpu < num_possible_cpus(); cpu++) {
		if (_hwinfo.core_map[cpu].cmg == cmg)
			cpumask_set_cpu(cpu, mask);
	}

	written += scnprintf(buf, PAGE_SIZE, "%*pbl\n", cpumask_pr_args(mask));
	free_cpumask_var(mask);

	return written;
}
BARRIER_ATTR(available_cpus);

static struct attribute *hwb_attrs[] = {
	&hwb_available_cpus_attribute.attr,
	NULL,
};

static const struct attribute_group hwb_attribute = {
	.attrs = hwb_attrs,
};

#define BB_ATTR_SHOW(name) \
static struct kobj_attribute bb_##name##_attribute = \
	__ATTR(name, 0440, bb_##name##_show, NULL)

#define BB_ATTR_SHOW_STORE(name) \
static struct kobj_attribute bb_##name##_attribute = \
	__ATTR(name, 0660, bb_##name##_show, bb_##name##_store)

static ssize_t bb_masks_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	struct bb_info *bb_info;
	ssize_t written = 0;
	int cmg;
	int bb;

	cmg = get_cmg_from_kobj(kobj->parent);
	bb = get_bb_from_kobj(kobj);
	bb_info = get_bb_info(cmg, bb);
	if (IS_ERR(bb_info)) {
		// not used
		written = scnprintf(buf, PAGE_SIZE, "\n");
		return written;
	}

	written += scnprintf(buf, PAGE_SIZE, "%*pbl\n", cpumask_pr_args(bb_info->pemask));
	put_bb_info(bb_info);

	return written;
}

// free no longer used bb
static int free_unused_bb(int cmg, int bb)
{
	struct bb_info *tmp_info;

	tmp_info = get_bb_info(cmg, bb);
	if (!IS_ERR(tmp_info)) {
		if (find_get_pid(tmp_info->pid)) {
			pr_debug("bb is still used:  %d/%d\n", cmg, bb);
			put_bb_info(tmp_info);
			return -EBUSY;
		}

		// the task which used this bb is already gone. reset resources
		teardown_bb(tmp_info);
		spin_lock(&pdata->list_lock);
		list_del_init(&tmp_info->node);
		spin_unlock(&pdata->list_lock);
		/* 1 put for alloc_bb_info, 1 put for get_bb_info */
		put_bb_info(tmp_info);
		put_bb_info(tmp_info);
	}

	return 0;
}

static int reserve_bb(struct bb_info *bb_info)
{
	int cpu;

	// check bw availability firt
	for (cpu = 0; cpu < num_possible_cpus(); cpu++) {
		if (cpumask_test_cpu(cpu, bb_info->pemask)) {
			if (!atomic_add_unless(&_hwinfo.used_bw_count[cpu], 1, _hwinfo.num_bw)) {
				pr_debug("cpu %d has already fully used all bw\n", cpu);
				cpu--;
				goto fail;
			}
		}
	}

	if (!test_and_set_bit(bb_info->bb, &_hwinfo.used_bb_bmap[bb_info->cmg])) {
		pr_debug("Use BB %u in CMG %u, bitmap: %lx\n",
					bb_info->bb, bb_info->cmg, _hwinfo.used_bb_bmap[bb_info->cmg]);
		// success
		return 0;
	}

	// try free no longer used barrier resource
	if (!free_unused_bb(bb_info->cmg, bb_info->bb)) {
		// retry
		if (!test_and_set_bit(bb_info->bb, &_hwinfo.used_bb_bmap[bb_info->cmg])) {
			pr_debug("Use BB %u in CMG %u, bitmap: %lx\n",
						bb_info->bb, bb_info->cmg, _hwinfo.used_bb_bmap[bb_info->cmg]);
			return 0;
		}
	}

fail:
	for (; cpu >= 0; cpu--) {
		if (cpumask_test_cpu(cpu, bb_info->pemask))
			atomic_dec(&_hwinfo.used_bw_count[cpu]);
	}
	return -EBUSY;
}

static int bb_alloc(const char *buf, int cmg, int bb)
{
	struct bb_info *bb_info;
	unsigned long physical_pemask;
	int ret;

	bb_info = alloc_bb_info();
	if (!bb_info)
		return -ENOMEM;

	ret = cpulist_parse(buf, bb_info->pemask);
	if (ret < 0)
		goto fail;

	bb_info->cmg = cmg;
	bb_info->bb = bb;
	bb_info->pid = task_tgid_vnr(current);

	pr_debug("newmask: %*pbl for %d\n", cpumask_pr_args(bb_info->pemask), bb_info->pid);

	ret = validate_and_convert_pemask(bb_info, &physical_pemask);
	if (ret < 0)
		goto fail;

	ret = reserve_bb(bb_info);
	if (ret < 0)
		goto fail;

	setup_bb(bb_info, physical_pemask);

	spin_lock(&pdata->list_lock);
	list_add_tail(&bb_info->node, &pdata->bb_list);
	spin_unlock(&pdata->list_lock);

	return 0;

fail:
	put_bb_info(bb_info);
	return ret;
}

static int bb_free(int cmg, int bb)
{
	struct bb_info *bb_info;

	bb_info = get_bb_info(cmg, bb);
	if (IS_ERR(bb_info)) {
		/* not allocated */
		return PTR_ERR(bb_info);
	}

	/* Forbid free/assign/unassign operation from now on */
	if (test_and_set_bit(BB_FREEING, &bb_info->flag)) {
		pr_err("free operation is already called. CMG: %u, BB: %u\n", cmg, bb);
		put_bb_info(bb_info);
		return -EPERM;
	}

	/* Wait current ongoing assign/unassign operation to finish */
	if (wait_event_interruptible(bb_info->wq,
					(atomic_read(&bb_info->ongoing_assign_count) == 0))) {
		clear_bit(BB_FREEING, &bb_info->flag);
		put_bb_info(bb_info);
		pr_debug("free operation is interrupted. CMG: %u, BB: %u\n", cmg, bb);
		return -EINTR;
	}

	pr_debug("reset status: %d/%d", cmg, bb);
	teardown_bb(bb_info);

	spin_lock(&pdata->list_lock);
	list_del_init(&bb_info->node);
	spin_unlock(&pdata->list_lock);

	/* 1 put for get_bb_info, 1 for alloc_bb_info */
	put_bb_info(bb_info);
	put_bb_info(bb_info);

	return 0;
}

static ssize_t bb_masks_store(struct kobject *kobj,
				     struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int cmg = get_cmg_from_kobj(kobj->parent);
	int bb = get_bb_from_kobj(kobj);

	if (count == 1) {
		// free allocated barrier resource
		ret = bb_free(cmg, bb);
		if (ret < 0)
			return ret;

		return count;
	}

	// allocate barrier resource
	ret = bb_alloc(buf, cmg, bb);
	if (ret < 0)
		return ret;

	return count;
}

BB_ATTR_SHOW_STORE(masks);

int get_window(struct bb_info *bb_info, int cpu)
{
	u8 ppe;

	if (!cpumask_test_cpu(cpu, bb_info->assigned_pemask))
		return -EINVAL;

	ppe = _hwinfo.core_map[cpu].ppe;
	return bb_info->bw[ppe];
}

static ssize_t bb_user_store(struct kobject *kobj,
				     struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int cmg;
	int bb;
	int cpu;
	pid_t pid;
	struct bb_info *bb_info;
	unsigned long flags;

	cmg = get_cmg_from_kobj(kobj->parent);
	bb = get_bb_from_kobj(kobj);
	cpu = smp_processor_id();

	pid = task_tgid_vnr(current);

	// only accepts empty write
	if (count != 1)
		return -EINVAL;

	if (!is_bound_only_one_pe())
		return -EPERM;

	bb_info = get_bb_info(cmg, bb);
	if (IS_ERR(bb_info))
		return PTR_ERR(bb_info);

	if (bb_info->pid != pid)  {
		pr_debug("this process is not part of this bb. pid: %d, bb_info->pid: %d\n", pid, bb_info->pid);
		put_bb_info(bb_info);
		return -EINVAL;

	}

	// unassign
	pr_debug("reset window status: %d/%d, bb_info->pid:%d, pid:%d", cmg, bb, bb_info->pid, pid);

	atomic_inc(&bb_info->ongoing_assign_count);

	local_irq_save(flags);
	ret = is_bw_unassignable(bb_info, cpu);
	if (!ret) {
		teardown_bw(bb_info, cpu);
		teardown_ctl_reg(bb_info, cpu);
	}
	local_irq_restore(flags);

	if (atomic_dec_and_test(&bb_info->ongoing_assign_count) &&
					test_bit(BB_FREEING, &bb_info->flag))
		wake_up(&bb_info->wq);

	put_bb_info(bb_info);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t bb_user_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	ssize_t ret;
	int cmg;
	int bb;
	int cpu;
	pid_t pid;
	struct bb_info *bb_info;
	int window;
	unsigned long flags;

	cmg = get_cmg_from_kobj(kobj->parent);
	bb = get_bb_from_kobj(kobj);
	cpu = smp_processor_id();

	pid = task_tgid_vnr(current);

	if (!is_bound_only_one_pe())
		return -EPERM;

	bb_info = get_bb_info(cmg, bb);
	if (IS_ERR(bb_info))
		return PTR_ERR(bb_info);

	if (bb_info->pid != pid)  {
		ret = -EINVAL;
		pr_debug("this process is not part of this bb. pid: %d, bb_info->pid: %d\n", pid, bb_info->pid);
		goto out;
	}

	ret = get_window(bb_info, cpu);
	if (ret >= 0) {
		// already assigned
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", (int)ret);
		goto out;
	}

	//  assign
	pr_debug("set window status: %d/%d, bb_info->pid:%d, pid:%d", cmg, bb, bb_info->pid, pid);

	atomic_inc(&bb_info->ongoing_assign_count);
	local_irq_save(flags);
	ret = is_bw_assignable(bb_info, &window, cpu);
	if (!ret) {
		setup_ctl_reg(bb_info, cpu);
		setup_bw(bb_info, window, cpu);
	}
	local_irq_restore(flags);

	if (atomic_dec_and_test(&bb_info->ongoing_assign_count) &&
					test_bit(BB_FREEING, &bb_info->flag))
		wake_up(&bb_info->wq);

	if (!ret)
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", window);

out:
	put_bb_info(bb_info);
	return ret;
}
BB_ATTR_SHOW_STORE(user);

static ssize_t bb_task_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	int cmg;
	int bb;
	struct bb_info *bb_info;
	ssize_t written;

	cmg = get_cmg_from_kobj(kobj->parent);
	bb = get_bb_from_kobj(kobj);

	bb_info = get_bb_info(cmg, bb);
	if (IS_ERR(bb_info)) {
		// not used
		written = scnprintf(buf, PAGE_SIZE, "\n");
		return written;
	}

	written = scnprintf(buf, PAGE_SIZE, "%d\n", bb_info->pid);
	put_bb_info(bb_info);

	return written;
}
BB_ATTR_SHOW(task);

static struct attribute *bb_attrs[] = {
	&bb_masks_attribute.attr,
	&bb_user_attribute.attr,
	&bb_task_attribute.attr,
	NULL,
};

static const struct attribute_group bb_attribute = {
	.attrs = bb_attrs,
};

static void destroy_sysfs(void)
{
	int cmg;
	int bb;

	for (cmg = 0; cmg < _hwinfo.num_cmg; cmg++) {
		if (cmg_kobj[cmg]) {

			if (bb_kobj[cmg]) {
				for (bb = 0; bb < _hwinfo.num_bb; bb++) {
					if (bb_kobj[cmg][bb]) {
						sysfs_remove_group(bb_kobj[cmg][bb], &bb_attribute);
						kobject_put(bb_kobj[cmg][bb]);
					}
				}
				kfree(bb_kobj[cmg]);
			}

			sysfs_remove_group(cmg_kobj[cmg], &hwb_attribute);
			kobject_put(cmg_kobj[cmg]);
		}
	}
	kfree(bb_kobj);
	kfree(cmg_kobj);
}

#define NAME_LEN 16
static int __init init_sysfs(void)
{
	char name[NAME_LEN];
	int ret;
	int cmg;
	int bb;

	cmg_kobj = kcalloc(_hwinfo.num_cmg, sizeof(struct kobject *), GFP_KERNEL);
	bb_kobj = kcalloc(_hwinfo.num_cmg, sizeof(struct kobject **), GFP_KERNEL);
	if (!cmg_kobj || !bb_kobj) {
		kfree(cmg_kobj);
		kfree(bb_kobj);
		return -ENOMEM;
	}

	/* Create folder for each CMG */
	for (cmg = 0; cmg < _hwinfo.num_cmg; cmg++) {
		scnprintf(name, NAME_LEN, "group%d", cmg);
		cmg_kobj[cmg] = kobject_create_and_add(name, &bar_miscdev.this_device->kobj);
		if (!cmg_kobj[cmg]) {
			ret = -ENOMEM;
			goto fail;
		}

		ret = sysfs_create_group(cmg_kobj[cmg], &hwb_attribute);
		if (ret)
			goto fail;

		bb_kobj[cmg] = kcalloc(_hwinfo.num_bb, sizeof(struct kobject *), GFP_KERNEL);
		if (!bb_kobj[cmg]) {
			ret = -ENOMEM;
			goto fail;
		}
		for (bb = 0; bb < _hwinfo.num_bb; bb++) {
			scnprintf(name, NAME_LEN, "barrier%d", bb);
			bb_kobj[cmg][bb] = kobject_create_and_add(name, cmg_kobj[cmg]);

			ret = sysfs_create_group(bb_kobj[cmg][bb], &bb_attribute);
			if (ret)
				goto fail;
		}
	}

	return 0;

fail:
	destroy_sysfs();
	return ret;
}

static int __init hwb_init(void)
{
	int ret;

	ret = setup_hwinfo();
	if (ret < 0) {
		pr_err("Unsupported CPU type\n");
		return ret;
	}

	ret = alloc_map();
	if (ret < 0)
		return ret;

	ret = init_bb_info_cachep();
	if (ret < 0)
		goto out1;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto out2;
	INIT_LIST_HEAD(&pdata->bb_list);
	spin_lock_init(&pdata->list_lock);

	/*
	 * Setup cpuhp callback to ensure each PE's resource will be initialized
	 * even if some PEs are offline at this point
	 */
	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "hard_barrier:online",
		hwb_cpu_online, NULL);
	if (ret < 0) {
		pr_err("cpuhp setup failed: %d\n", ret);
		goto out3;
	}
	_hp_state = ret;

	ret = misc_register(&bar_miscdev);
	if (ret < 0) {
		pr_err("misc_register failed: %d\n", ret);
		goto out4;
	}

	ret = init_sysfs();
	if (ret < 0) {
		pr_err("sysfs creation failed: %d\n", ret);
		goto out5;
	}

	return 0;

out5:
	misc_deregister(&bar_miscdev);
out4:
	cpuhp_remove_state(_hp_state);
out3:
	kfree(pdata);
out2:
	destroy_bb_info_cachep();
out1:
	free_map();

	return ret;
}

static void __exit hwb_exit(void)
{
	struct bb_info *bb_info, *tmp;
	/*
	 * Cleanup BB if free operation is not called properly.
	 * No lock for pdata->bb_list is needed cause there is no one else
	 */
	if (!list_empty(&pdata->bb_list)) {
		pr_warn("free operation is not called properly\n");

		list_for_each_entry_safe(bb_info, tmp, &pdata->bb_list, node) {
			teardown_bb(bb_info);
			list_del_init(&bb_info->node);
			/* 1 put for alloc_bb_info */
			put_bb_info(bb_info);
		}
	}

	destroy_sysfs();
	misc_deregister(&bar_miscdev);
	cpuhp_remove_state(_hp_state);
	kfree(pdata);
	destroy_bb_info_cachep();
	free_map();
}

module_init(hwb_init);
module_exit(hwb_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("FUJITSU LIMITED");
MODULE_DESCRIPTION("FUJITSU HPC Hardware Barrier Driver");
