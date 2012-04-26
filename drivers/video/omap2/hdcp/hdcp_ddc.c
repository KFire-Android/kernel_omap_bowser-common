/*
 * hdcp_ddc.c
 *
 * HDCP interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Fabrice Olivero
 *	Fabrice Olivero <f-olivero@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <media/hdcp.h>
#include "hdcp_ddc.h"

/*-----------------------------------------------------------------------------
 * Function: hdcp_suspend_resume_auto_ri
 *-----------------------------------------------------------------------------
 */
static int hdcp_suspend_resume_auto_ri(enum ri_suspend_resume state)
{
	static u8 OldRiStat, OldRiCommand;
	u8 TimeOut = 10;

	/* Suspend Auto Ri in order to allow FW access MDDC bus.
	 * Poll 0x72:0x26[0] for MDDC bus availability or timeout
	 */

	DBG("hdcp_suspend_resume_auto_ri() state=%s",
		state == AUTO_RI_SUSPEND ? "SUSPEND" : "RESUME");

	if (state == AUTO_RI_SUSPEND) {
		/* Save original Auto Ri state */
		OldRiCommand = RD_FIELD_32(hdcp.hdmi_wp_base_addr +
					   HDMI_IP_CORE_SYSTEM,
					   HDMI_IP_CORE_SYSTEM__RI_CMD, 0, 0);

		/* Disable Auto Ri */
		hdcp_lib_auto_ri_check(false);

		/* Wait for HW to release MDDC bus */
		/* TODO: while loop / timeout to be enhanced */
		while (--TimeOut) {
			if (!RD_FIELD_32(hdcp.hdmi_wp_base_addr +
					 HDMI_IP_CORE_SYSTEM,
					 HDMI_IP_CORE_SYSTEM__RI_STAT, 0, 0))
				break;
		}

		/* MDDC bus not relinquished */
		if (!TimeOut) {
			printk(KERN_ERR "HDCP: Suspending Auto Ri failed !\n");
			return -HDCP_DDC_ERROR;
		}

		OldRiStat = RD_FIELD_32(hdcp.hdmi_wp_base_addr +
					HDMI_IP_CORE_SYSTEM,
					HDMI_IP_CORE_SYSTEM__RI_STAT, 0, 0);
	} else {
		/* If Auto Ri was enabled before it was suspended */
		if ((OldRiStat) && (OldRiCommand))
			/* Re-enable Auto Ri */
			hdcp_lib_auto_ri_check(false);
	}

	return HDCP_OK;
}
