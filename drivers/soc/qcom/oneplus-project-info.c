// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus SM8150 phones: complete the project info SMEM item for the modem.
 *
 * XBL allocates SMEM item 136 (struct project_info in OnePlus's downstream
 * include/linux/project_info.h) and fills in the board identity. OnePlus's
 * kernel then writes rf_v3, the RF test cable state: op_rf_cable_monitor.c
 * sets it to GPIO 36 || PM8009 GPIO 4, both pulled up, so 1 means no cable.
 * Left at 0, the modem believes cables are plugged into its RF test
 * connectors.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/smem.h>

#define SMEM_PROJECT_INFO	136

struct oneplus_project_info {
	char project_name[8];
	__le32 hw_version;
	__le32 rf_v1;
	__le32 rf_v2;
	__le32 rf_v3;
	__le32 modem;
	__le32 operator;
	__le32 ddr_manufacture_info;
	__le32 ddr_row;
	__le32 ddr_column;
	__le32 ddr_fw_version;
	__le32 ddr_reserve_info;
	__le32 platform_id;
	__le32 ftm_uart_boot_mode;
	__le32 feature_id;
	__le32 a_board_version;
	__le32 prj_version;
};

static int oneplus_project_info_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct oneplus_project_info *info;
	u32 rf_cable = 1;
	size_t size;

	info = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_PROJECT_INFO, &size);
	if (IS_ERR(info))
		return dev_err_probe(dev, PTR_ERR(info),
				     "no project info in SMEM\n");
	if (size < sizeof(*info))
		return dev_err_probe(dev, -EINVAL,
				     "project info too small (%zu bytes)\n", size);

	of_property_read_u32(dev->of_node, "oneplus,rf-cable-state", &rf_cable);

	dev_info(dev, "project %.8s hw %u rf %u/%u prj %u, rf cable state %u -> %u\n",
		 info->project_name, le32_to_cpu(info->hw_version),
		 le32_to_cpu(info->rf_v1), le32_to_cpu(info->rf_v2),
		 le32_to_cpu(info->prj_version), le32_to_cpu(info->rf_v3),
		 rf_cable);
	info->rf_v3 = cpu_to_le32(rf_cable);

	return 0;
}

static const struct of_device_id oneplus_project_info_of_match[] = {
	{ .compatible = "oneplus,sm8150-project-info" },
	{}
};
MODULE_DEVICE_TABLE(of, oneplus_project_info_of_match);

static struct platform_driver oneplus_project_info_driver = {
	.probe = oneplus_project_info_probe,
	.driver = {
		.name = "oneplus-project-info",
		.of_match_table = oneplus_project_info_of_match,
	},
};
module_platform_driver(oneplus_project_info_driver);

MODULE_DESCRIPTION("OnePlus SM8150 project info for the modem");
MODULE_LICENSE("GPL");
