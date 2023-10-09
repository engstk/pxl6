#include "pmucal_system.h"
#include "pmucal_rae.h"
#include "pmucal_powermode.h"
#include <soc/google/debug-snapshot.h>
#include "cmucal.h"
#include "acpm_dvfs.h"
#include <dt-bindings/clock/gs101.h>

unsigned int pmucal_sys_powermode[NUM_SYS_POWERDOWN] = {0xffffffff, };
static bool tcxo_req = true;

void pmucal_tcxo_demand(bool tcxo_demand)
{
	tcxo_req = tcxo_demand;
	pr_info("%s, tcxo_req: %d\n", __func__, tcxo_req);
}
EXPORT_SYMBOL(pmucal_tcxo_demand);

/**
 *  pmucal_system_enter - prepares to enter a system power mode.
 *		        exposed to PWRCAL interface.
 *
 *  @mode: index of system power mode described in pmucal_system.h.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_enter(int mode)
{
	int ret;
	char err_msg[128];

	err_msg[0] = '\0';
	if (mode == SYS_SLEEP && !tcxo_req)
		ret = exynos_acpm_set_rate(GET_IDX(ACPM_DVFS_HSI0_TCXO), 0);

	if (mode != SYS_SICD)
		dbg_snapshot_pmu(mode, __func__, DSS_FLAG_IN);

	if (mode >= NUM_SYS_POWERDOWN) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: mode index(%d) is out of supported range (0~%d).",
			PMUCAL_PREFIX, __func__, mode, NUM_SYS_POWERDOWN);
		ret = -EINVAL;
		goto err_out;
	}

	if (!pmucal_lpm_list[mode].enter) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: there is no sequence element for entering mode(%d).",
			PMUCAL_PREFIX, __func__, mode);
		ret = -ENOENT;
		goto err_out;
	}

	pmucal_powermode_hint(pmucal_sys_powermode[mode]);

	pmucal_rae_save_seq(pmucal_lpm_list[mode].save, pmucal_lpm_list[mode].num_save);

	ret = pmucal_rae_handle_seq(pmucal_lpm_list[mode].enter,
				pmucal_lpm_list[mode].num_enter);
	if (ret) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: error on handling enter sequence. (mode : %d)",
			PMUCAL_PREFIX, __func__, mode);
		goto err_out;
	}

	if (mode != SYS_SICD)
		dbg_snapshot_pmu(mode, __func__, DSS_FLAG_OUT);

	pmucal_dbg_set_emulation(pmucal_lpm_list[mode].dbg);
	pmucal_dbg_do_profile(pmucal_lpm_list[mode].dbg, false);

	return 0;

err_out:
	dbg_snapshot_emergency_reboot(err_msg);

	return ret;
}

/**
 *  pmucal_system_exit - prepares to exit a system power mode.
 *		        exposed to PWRCAL interface.
 *
 *  @mode: index of system power mode described in pmucal_system.h.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_exit(int mode)
{
	int ret;
	char err_msg[128];

	err_msg[0] = '\0';
	if (mode == SYS_SLEEP)
		tcxo_req = true;

	if (mode != SYS_SICD)
		dbg_snapshot_pmu(mode, __func__, DSS_FLAG_IN);

	if (mode >= NUM_SYS_POWERDOWN) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: mode index(%d) is out of supported range (0~%d).",
			PMUCAL_PREFIX, __func__, mode, NUM_SYS_POWERDOWN);
		ret = -EINVAL;
		goto err_out;
	}

	if (!pmucal_lpm_list[mode].exit) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: there is no sequence element for exiting mode(%d).",
			PMUCAL_PREFIX, __func__, mode);
		ret = -ENOENT;
		goto err_out;
	}

	ret = pmucal_rae_handle_seq(pmucal_lpm_list[mode].exit,
				pmucal_lpm_list[mode].num_exit);
	if (ret) {
		scnprintf(err_msg, sizeof(err_msg),
			  "%s %s: error on handling exit sequence. (mode : %d)",
			  PMUCAL_PREFIX, __func__, mode);
		goto err_out;
	}

	ret = pmucal_rae_restore_seq(pmucal_lpm_list[mode].save,
				pmucal_lpm_list[mode].num_save);
	if (ret) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: error on handling restore sequence. (mode : %d)",
			PMUCAL_PREFIX, __func__, mode);
		goto err_out;
	}

	if (mode != SYS_SICD)
		dbg_snapshot_pmu(mode, __func__, DSS_FLAG_OUT);

	pmucal_dbg_do_profile(pmucal_lpm_list[mode].dbg, true);

	return 0;

err_out:
	dbg_snapshot_emergency_reboot(err_msg);

	return ret;
}

/**
 *  pmucal_system_earlywakeup - prepares to early-wakeup from a system power mode.
 *				exposed to PWRCAL interface.
 *
 *  @mode: index of system power mode described in pmucal_system.h.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_earlywakeup(int mode)
{
	int ret;
	char err_msg[128];

	err_msg[0] = '\0';
	if (mode != SYS_SICD)
		dbg_snapshot_pmu(mode, __func__, DSS_FLAG_IN);

	if (mode >= NUM_SYS_POWERDOWN) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: mode index(%d) is out of supported range (0~%d).",
			PMUCAL_PREFIX, __func__, mode, NUM_SYS_POWERDOWN);
		ret = -EINVAL;
		goto err_out;
	}

	if (!pmucal_lpm_list[mode].early_wakeup) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: there is no sequence element for early_wkup mode(%d).",
			PMUCAL_PREFIX, __func__, mode);
		ret = -ENOENT;
		goto err_out;
	}

	ret = pmucal_rae_handle_seq(pmucal_lpm_list[mode].early_wakeup,
				pmucal_lpm_list[mode].num_early_wakeup);
	if (ret) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: error on handling elry_wkup sequence. (mode : %d)",
			PMUCAL_PREFIX, __func__, mode);
		goto err_out;
	}

	ret = pmucal_rae_restore_seq(pmucal_lpm_list[mode].save,
				pmucal_lpm_list[mode].num_save);
	if (ret) {
		scnprintf(
			err_msg, sizeof(err_msg),
			"%s %s: error on handling restore sequence. (mode : %d)",
			PMUCAL_PREFIX, __func__, mode);
		goto err_out;
	}
	pmucal_powermode_hint_clear();

	if (mode != SYS_SICD)
		dbg_snapshot_pmu(mode, __func__, DSS_FLAG_OUT);

	return 0;

err_out:
	dbg_snapshot_emergency_reboot(err_msg);

	return ret;
}

/**
 *  pmucal_system_init - Init function of PMUCAL SYSTEM common logic.
 *		            exposed to PWRCAL interface.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_system_init(void)
{
	int ret = 0, i;

	if (!pmucal_lpm_init_size || !pmucal_lpm_list_size) {
		pr_err("%s %s: there is no lpm init seq or lpm list. aborting init...\n",
				PMUCAL_PREFIX, __func__);
		return -ENOENT;
	}

	/* convert physical base address to virtual addr */
	for (i = 0; i < NUM_SYS_POWERDOWN; i++) {
		/* skip non-existing power mode */
		if (!pmucal_lpm_list[i].num_enter && !pmucal_lpm_list[i].num_exit && !pmucal_lpm_list[i].num_early_wakeup)
			continue;

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].enter,
					pmucal_lpm_list[i].num_enter);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:enter, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].save,
					pmucal_lpm_list[i].num_save);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:save, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].exit,
					pmucal_lpm_list[i].num_exit);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:exit, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}

		ret = pmucal_rae_phy2virt(pmucal_lpm_list[i].early_wakeup,
					pmucal_lpm_list[i].num_early_wakeup);
		if (ret) {
			pr_err("%s %s: error on PA2VA conversion. seq:erlywkup, mode_id:%d. aborting init...\n",
					PMUCAL_PREFIX, __func__, i);
			goto out;
		}
	}

	ret = pmucal_rae_phy2virt(pmucal_lpm_init, pmucal_lpm_init_size);
	if (ret) {
		pr_err("%s %s: error on PA2VA conversion for lpm_init seq. aborting init...\n",
				PMUCAL_PREFIX, __func__);
		goto out;
	}

	/* do default settings for PMU */
	ret = pmucal_rae_handle_seq(pmucal_lpm_init, pmucal_lpm_init_size);
	if (ret) {
		pr_err("%s %s: error on handling lpm_init sequence.\n",
				PMUCAL_PREFIX, __func__);
		goto out;
	}

out:
	return ret;
}
