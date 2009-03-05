/*
 * Monahans PMIC abstrction layer
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <asm/arch/mhn_pmic.h>

struct pmic_ops *mhn_pmic_ops;

void pmic_set_ops(struct pmic_ops *ops)
{
	if (mhn_pmic_ops != NULL) {
		printk(KERN_ERR "set pmic_ops when pmic_ops is not NULL\n");
		return;
	}
	mhn_pmic_ops = ops;
}

/*****************************************************************************
 *			Operation of PMIC				     *
 *****************************************************************************/
int check_pmic_ops(void)
{
	if (!mhn_pmic_ops) {
		printk(KERN_WARNING "No pmic_ops registered!\n");
		return -EINVAL;
	} else
		return 0;
}

int mhn_pmic_get_voltage(int cmd, int *pval)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (mhn_pmic_ops->get_voltage)
		return mhn_pmic_ops->get_voltage(cmd, pval);
	else
		return -EINVAL;
}

EXPORT_SYMBOL(mhn_pmic_get_voltage);

int mhn_pmic_set_voltage(int cmd, int val)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (mhn_pmic_ops->set_voltage)
		return mhn_pmic_ops->set_voltage(cmd, val);
	else
		return -EINVAL;
}

EXPORT_SYMBOL(mhn_pmic_set_voltage);

int mhn_pmic_is_vbus_assert(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)		/* If illegal pmic_ops, always no vbus activity */
		return 0;

	if (mhn_pmic_ops->is_vbus_assert)
		return mhn_pmic_ops->is_vbus_assert();

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_is_vbus_assert);

int mhn_pmic_is_avbusvld(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)		/* If illegal pmic_ops, always no A vbus valid */
		return 0;

	if (mhn_pmic_ops->is_avbusvld)
		return mhn_pmic_ops->is_avbusvld();

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_is_avbusvld);

int mhn_pmic_is_asessvld(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)		/* If illegal pmic_ops, always no A assert valid */
		return 0;

	if (mhn_pmic_ops->is_asessvld)
		return mhn_pmic_ops->is_asessvld();

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_is_asessvld);

int mhn_pmic_is_bsessvld(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)		/* If illegal pmic_ops, always no B assert valid */
		return 0;

	if (mhn_pmic_ops->is_bsessvld)
		return mhn_pmic_ops->is_bsessvld();

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_is_bsessvld);

int mhn_pmic_is_srp_ready(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)		/* If illegal pmic_ops, always no SRP detect */
		return 0;

	if (mhn_pmic_ops->is_srp_ready)
		return mhn_pmic_ops->is_srp_ready();

	return 0;

}

EXPORT_SYMBOL(mhn_pmic_is_srp_ready);

int mhn_pmic_set_pump(int enable)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (mhn_pmic_ops->set_pump)
		return mhn_pmic_ops->set_pump(enable);

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_set_pump);

int mhn_pmic_set_vbus_supply(int enable, int srp)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (mhn_pmic_ops->set_vbus_supply)
		return mhn_pmic_ops->set_vbus_supply(enable, srp);

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_set_vbus_supply);

int mhn_pmic_set_usbotg_a_mask(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (mhn_pmic_ops->set_usbotg_a_mask)
		return mhn_pmic_ops->set_usbotg_a_mask();

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_set_usbotg_a_mask);

int mhn_pmic_set_usbotg_b_mask(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (mhn_pmic_ops->set_usbotg_b_mask)
		return mhn_pmic_ops->set_usbotg_b_mask();

	return 0;
}

EXPORT_SYMBOL(mhn_pmic_set_usbotg_b_mask);
