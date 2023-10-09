#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/module.h>
#include <soc/google/ect_parser.h>

#include "cmucal.h"
#include "vclk.h"
#include "ra.h"
#include "acpm_dvfs.h"
#include "asv.h"

#define ECT_DUMMY_SFR	(0xFFFFFFFF)

static struct vclk_lut *get_lut(struct vclk *vclk, unsigned int rate)
{
	int i;

	for (i = 0; i < vclk->num_rates; i++)
		if (rate >= vclk->lut[i].rate)
			break;

	if (i == vclk->num_rates)
		return NULL;

	return &vclk->lut[i];
}

static unsigned int get_max_rate(unsigned int from, unsigned int to)
{
	unsigned int max_rate;

	if (from)
		max_rate = (from > to) ? from : to;
	else
		max_rate = to;

	return max_rate;
}

static void __select_switch_pll(struct vclk *vclk,
				unsigned int rate,
				unsigned int select)
{
	if (vclk->ops && vclk->ops->switch_pre)
		vclk->ops->switch_pre(vclk->vrate, rate);

	if (vclk->ops && vclk->ops->switch_trans && select)
		vclk->ops->switch_trans(vclk->vrate, rate);
	else if (vclk->ops && vclk->ops->restore_trans && !select)
		vclk->ops->restore_trans(vclk->vrate, rate);
	else
		ra_select_switch_pll(vclk->switch_info, select);

	if (vclk->ops && vclk->ops->switch_post)
		vclk->ops->switch_post(vclk->vrate, rate);
}

static int transition_switch(struct vclk *vclk, struct vclk_lut *lut,
			     unsigned int switch_rate)
{
	unsigned int *list = vclk->list;
	unsigned int num_list = vclk->num_list;

	/* Change to switching PLL */
	if (vclk->ops && vclk->ops->trans_pre)
		vclk->ops->trans_pre(vclk->vrate, lut->rate);

	ra_set_clk_by_type(list, lut, num_list, DIV_TYPE, TRANS_HIGH);

	__select_switch_pll(vclk, switch_rate, 1);

	ra_set_clk_by_type(list, lut, num_list, MUX_TYPE, TRANS_FORCE);
	ra_set_clk_by_type(list, lut, num_list, DIV_TYPE, TRANS_LOW);

	vclk->vrate = switch_rate;

	return 0;
}

static int transition_restore(struct vclk *vclk, struct vclk_lut *lut)
{
	unsigned int *list = vclk->list;
	unsigned int num_list = vclk->num_list;

	/* PLL setting */
	ra_set_pll_ops(list, lut, num_list, vclk->ops);

	ra_set_clk_by_type(list, lut, num_list, DIV_TYPE, TRANS_HIGH);

	__select_switch_pll(vclk, lut->rate, 0);

	ra_set_clk_by_type(list, lut, num_list, MUX_TYPE, TRANS_FORCE);
	ra_set_clk_by_type(list, lut, num_list, DIV_TYPE, TRANS_LOW);
	if (vclk->ops && vclk->ops->trans_post)
		vclk->ops->trans_post(vclk->vrate, lut->rate);

	return 0;
}

static int transition(struct vclk *vclk,
		      struct vclk_lut *lut)
{
	unsigned int *list = vclk->list;
	unsigned int num_list = vclk->num_list;

	ra_set_clk_by_type(list, lut, num_list, DIV_TYPE, TRANS_HIGH);
	ra_set_clk_by_type(list, lut, num_list, PLL_TYPE, TRANS_LOW);
	ra_set_clk_by_type(list, lut, num_list, MUX_TYPE, TRANS_FORCE);
	ra_set_clk_by_type(list, lut, num_list, PLL_TYPE, TRANS_HIGH);
	ra_set_clk_by_type(list, lut, num_list, DIV_TYPE, TRANS_LOW);

	return 0;
}

static bool is_switching_pll_ops(struct vclk *vclk, int cmd)
{
	int i;

	if (!vclk->switch_info)
		return false;

	if (cmd != ONESHOT_TRANS)
		return true;

	for (i = 0; i < vclk->num_list; i++) {
		if (IS_PLL(vclk->list[i]))
			return true;
	}

	return false;
}

static int __vclk_set_rate(unsigned int id, unsigned int rate, int cmd)
{
	struct vclk *vclk;
	struct vclk_lut *new_lut, *switch_lut;
	unsigned int switch_rate, max_rate;

	if (!IS_VCLK(id))
		return ra_set_rate(id, rate);

	vclk = cmucal_get_node(id);
	if (!vclk || !vclk->lut)
		return -EVCLKINVAL;

	if (IS_DFS_VCLK(id) || IS_COMMON_VCLK(id))
		new_lut = get_lut(vclk, rate);
	else
		new_lut = get_lut(vclk, rate / 1000);

	if (!new_lut)
		return -EVCLKINVAL;

	if (is_switching_pll_ops(vclk, cmd)) {
		switch_lut = new_lut;
		switch_rate = rate;
		if (is_oneshot_trans(cmd)) {
			max_rate = get_max_rate(vclk->vrate, rate);
			switch_rate = ra_set_rate_switch(vclk->switch_info,
							 max_rate);
			switch_lut = get_lut(vclk, switch_rate);
			if (!switch_lut)
				return -EVCLKINVAL;
		}
		if (is_switch_trans(cmd))
			transition_switch(vclk, switch_lut, switch_rate);
		if (is_restore_trans(cmd))
			transition_restore(vclk, new_lut);
	} else if (vclk->seq) {
		ra_set_clk_by_seq(vclk->list,
				  new_lut,
				  vclk->seq,
				  vclk->num_list);
	} else {
		transition(vclk, new_lut);
	}

	vclk->vrate = rate;

	return 0;
}

int vclk_set_rate(unsigned int id, unsigned long rate)
{
	int ret;

	ret = __vclk_set_rate(id, rate, ONESHOT_TRANS);

	return ret;
}

int vclk_set_rate_switch(unsigned int id, unsigned long rate)
{
	int ret;

	ret = __vclk_set_rate(id, rate, SWITCH_TRANS);

	return ret;
}

int vclk_set_rate_restore(unsigned int id, unsigned long rate)
{
	int ret;

	ret = __vclk_set_rate(id, rate, RESTORE_TRANS);

	return ret;
}

unsigned long vclk_recalc_rate(unsigned int id)
{
	struct vclk *vclk;
	int i, ret;

	if (!IS_VCLK(id))
		return ra_recalc_rate(id);

	vclk = cmucal_get_node(id);
	if (!vclk)
		return 0;

	if (IS_DFS_VCLK(vclk->id) ||
	    IS_COMMON_VCLK(vclk->id) ||
	    IS_ACPM_VCLK(vclk->id)) {
		for (i = 0; i < vclk->num_rates; i++) {
			ret = ra_compare_clk_list(vclk->lut[i].params,
						  vclk->list,
						  vclk->num_list);
			if (!ret) {
				vclk->vrate = vclk->lut[i].rate;
				break;
			}
		}

		if (i == vclk->num_rates) {
			vclk->vrate = 0;
			pr_debug("%s:%x failed\n", __func__, id);
		}
	} else {
		vclk->vrate = ra_recalc_rate(vclk->list[0]);
	}

	return vclk->vrate;
}

unsigned long vclk_get_rate(unsigned int id)
{
	struct vclk *vclk;

	if (IS_VCLK(id)) {
		vclk = cmucal_get_node(id);
		if (vclk)
			return vclk->vrate;
	}

	return 0;
}

int vclk_set_enable(unsigned int id)
{
	struct vclk *vclk;
	int ret = -EVCLKINVAL;

	if (IS_GATE_VCLK(id)) {
		vclk = cmucal_get_node(id);
		if (vclk)
			ret = ra_set_list_enable(vclk->list, vclk->num_list);
	} else if (IS_VCLK(id)) {
		ret = 0;
	} else {
		ret = ra_set_enable(id, 1);
	}

	return ret;
}

int vclk_set_disable(unsigned int id)
{
	struct vclk *vclk;
	int ret = -EVCLKINVAL;

	if (IS_GATE_VCLK(id)) {
		vclk = cmucal_get_node(id);
		if (vclk)
			ret = ra_set_list_disable(vclk->list, vclk->num_list);
	} else if (IS_VCLK(id)) {
		ret = 0;
	} else {
		ret = ra_set_enable(id, 0);
	}

	return ret;
}

unsigned int vclk_get_lv_num(unsigned int id)
{
	struct vclk *vclk;
	int lv_num = 0;

	vclk = cmucal_get_node(id);

	if (vclk && vclk->lut)
		lv_num = vclk->num_rates;

	return lv_num;

}

unsigned int vclk_get_max_freq(unsigned int id)
{
	struct vclk *vclk;
	int rate = 0;

	vclk = cmucal_get_node(id);

	if (vclk && vclk->lut)
		rate = vclk->max_freq;

	return rate;
}

unsigned int vclk_get_min_freq(unsigned int id)
{
	struct vclk *vclk;
	int rate = 0;

	vclk = cmucal_get_node(id);

	if (vclk && vclk->lut)
		rate = vclk->min_freq;

	return rate;
}

int vclk_get_rate_table(unsigned int id, unsigned long *table)
{
	struct vclk *vclk;
	int i;
	unsigned int nums = 0;

	vclk = cmucal_get_node(id);
	if (!vclk || !IS_VCLK(vclk->id))
		return 0;
	if (vclk->lut) {
		for (i = 0; i < vclk->num_rates; i++)
			table[i] = vclk->lut[i].rate;
		nums = vclk->num_rates;
	}

	return nums;
}

unsigned int vclk_get_boot_freq(unsigned int id)
{
	struct vclk *vclk;
	unsigned int rate = 0;

	vclk = cmucal_get_node(id);
	if (!vclk || !(IS_DFS_VCLK(vclk->id) || IS_ACPM_VCLK(vclk->id)))
		return rate;

	if (vclk->boot_freq)
		rate = vclk->boot_freq;
	else
		rate = (unsigned int)vclk_recalc_rate(id);

	return rate;
}

unsigned int vclk_get_resume_freq(unsigned int id)
{
	struct vclk *vclk;
	unsigned int rate = 0;

	vclk = cmucal_get_node(id);
	if (!vclk || !(IS_DFS_VCLK(vclk->id) || IS_ACPM_VCLK(vclk->id)))
		return rate;

	if (vclk->resume_freq)
		rate = vclk->resume_freq;
	else
		rate = (unsigned int)vclk_recalc_rate(id);

	return rate;
}

static int vclk_get_dfs_info(struct vclk *vclk, unsigned int minmax_idx)
{
	int i, j;
	void *dvfs_block;
	struct ect_dvfs_domain *dvfs_domain;
	void *gen_block;
	struct ect_gen_param_table *minmax = NULL;
	unsigned int *minmax_table = NULL;
	int *params, idx;
	int ret = 0;
	char buf[32];

	dvfs_block = ect_get_block("DVFS");
	if (dvfs_block == NULL)
		return -EVCLKNOENT;

	dvfs_domain = ect_dvfs_get_domain(dvfs_block, vclk->name);
	if (dvfs_domain == NULL)
		return -EVCLKINVAL;

	gen_block = ect_get_block("GEN");
	if (gen_block) {
		sprintf(buf, "MINMAX_%s", vclk->name);
		minmax = ect_gen_param_get_table(gen_block, buf);
		if (minmax != NULL) {
			if (minmax_idx > minmax->num_of_row)
				minmax_idx = 0;
			minmax_table =
			    &minmax->parameter[minmax_idx * minmax->num_of_col];
		}
	}

	vclk->num_rates = dvfs_domain->num_of_level;
	vclk->num_list = dvfs_domain->num_of_clock;
	vclk->max_freq = dvfs_domain->max_frequency;
	vclk->min_freq = dvfs_domain->min_frequency;

	if (minmax_table != NULL) {
		vclk->min_freq = minmax_table[MINMAX_MIN_FREQ] * 1000;
		vclk->max_freq = minmax_table[MINMAX_MAX_FREQ] * 1000;
	}
	pr_debug("ACPM_DVFS :%s\n", vclk->name);

	vclk->list = kcalloc(vclk->num_list, sizeof(unsigned int), GFP_KERNEL);
	if (!vclk->list)
		return -EVCLKNOMEM;

	vclk->lut = kzalloc(sizeof(struct vclk_lut) * vclk->num_rates,
			    GFP_KERNEL);
	if (!vclk->lut) {
		ret = -EVCLKNOMEM;
		goto err_nomem1;
	}

	for (i = 0; i < vclk->num_rates; i++) {
		vclk->lut[i].rate = dvfs_domain->list_level[i].level;
		params = kcalloc(vclk->num_list, sizeof(int), GFP_KERNEL);
		if (!params) {
			ret = -EVCLKNOMEM;
			if (i == 0)
				goto err_nomem2;
			for (i = i-1; i >= 0; i--)
				kfree(vclk->lut[i].params);
			goto err_nomem2;
		}

		for (j = 0; j < vclk->num_list; ++j) {
			idx = i * vclk->num_list + j;
			params[j] = dvfs_domain->list_dvfs_value[idx];
		}
		vclk->lut[i].params = params;
	}
	vclk->boot_freq = 0;
	vclk->resume_freq = 0;

	if (minmax_table != NULL) {
		for (i = 0; i <  vclk->num_rates; i++) {
			if (vclk->lut[i].rate == minmax_table[MINMAX_BOOT_FREQ] * 1000)
				vclk->boot_freq = vclk->lut[i].rate;
		}

		for (i = 0; i < vclk->num_rates; i++) {
			if (vclk->lut[i].rate == minmax_table[MINMAX_RESUME_FREQ] * 1000)
				vclk->resume_freq = vclk->lut[i].rate;
		}
	} else {
		if (dvfs_domain->boot_level_idx != -1)
			vclk->boot_freq = vclk->lut[dvfs_domain->boot_level_idx].rate;

		if (dvfs_domain->resume_level_idx != -1)
			vclk->resume_freq = vclk->lut[dvfs_domain->resume_level_idx].rate;
	}

	return ret;
err_nomem2:
	kfree(vclk->lut);
err_nomem1:
	kfree(vclk->list);

	return ret;
}

static void vclk_bind(unsigned int minmax_idx)
{
	struct vclk *vclk;
	int i;
	bool warn_on = 0;
	int ret;

	for (i = 0; i < cmucal_get_list_size(ACPM_VCLK_TYPE); i++) {
		vclk = cmucal_get_node(ACPM_VCLK_TYPE | i);
		if (!vclk) {
			pr_err("cannot found vclk node %x\n", i);
			continue;
		}

		ret = vclk_get_dfs_info(vclk, minmax_idx);
		if (ret == -EVCLKNOENT) {
			if (!warn_on)
				pr_warn("ECT DVFS not found\n");
			warn_on = 1;
		} else if (ret) {
			pr_err("ECT DVFS [%s] not found %d\n",
				   vclk->name, ret);
		}
	}
}

int vclk_register_ops(unsigned int id, struct vclk_trans_ops *ops)
{
	struct vclk *vclk;

	if (IS_DFS_VCLK(id)) {
		vclk = cmucal_get_node(id);
		if (!vclk)
			return -EVCLKINVAL;
		vclk->ops = ops;

		return 0;
	}

	return -EVCLKNOENT;
}

int vclk_initialize(unsigned int minmax_idx)
{
	pr_info("vclk initialize for cmucal\n");

	ra_init();

	vclk_bind(minmax_idx);

	return 0;
}

MODULE_LICENSE("GPL");
