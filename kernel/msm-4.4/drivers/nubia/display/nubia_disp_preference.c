/*
 * nubia_disp_preference.c - nubia lcd display color enhancement and temperature setting
 *	      Linux kernel modules for mdss
 *
 * Copyright (c) 2015 nubia <nubia@nubia.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Supports NUBIA lcd display color enhancement and color temperature setting
 */

/*------------------------------ header file --------------------------------*/
#include "nubia_disp_preference.h"
#include <linux/delay.h>

/*------------------------------- variables ---------------------------------*/

static struct dsi_panel_cmds cabc_cmds_off;
static struct dsi_panel_cmds cabc_cmds_level1;
static struct dsi_panel_cmds cabc_cmds_level2;
static struct dsi_panel_cmds cabc_cmds_level3;

static struct kobject *enhance_kobj = NULL;
static struct mdss_dsi_ctrl_pdata *nubia_mdss_dsi_ctrl = NULL;
int panel_ready_for_cmd =0;

static struct nubia_disp_type nubia_disp_val = {
	.en_cabc = 1,
	.cabc = CABC_OFF,
};
extern int nubia_mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds);

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = flags;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;
	else if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

int nubia_mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	int ret = 0;
	if(panel_ready_for_cmd == 1){
		mdss_dsi_panel_cmds_send(ctrl,pcmds,CMD_REQ_COMMIT);
		ret = 0;
		pr_err("nubia lcd disp func");
	}else{
		pr_err("nubia lcd disp not ready");
		ret = -1;
	}
	return ret;
}

static int nubia_set_cabc(int cabc_val)
{
        int ret = 0;
	if (!nubia_disp_val.en_cabc) {
                ret = -1;
                NUBIA_DISP_ERROR("no saturation\n");
                return ret;
        }
        switch (cabc_val) {
		case CABC_OFF:
			if (cabc_cmds_off.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &cabc_cmds_off);
			break;
		case CABC_LEVER1:
			if (cabc_cmds_level1.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &cabc_cmds_level1);
			break;
		case CABC_LEVER2:
			if (cabc_cmds_level2.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &cabc_cmds_level2);
			break;
		case CABC_LEVER3:
			if (cabc_cmds_level3.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &cabc_cmds_level3);
			break;
    default:
			if (cabc_cmds_off.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &cabc_cmds_off);
			break;
	}
	return ret;
}

static ssize_t lcd_power_on_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;

	val = CABC_LEVER1;
	if ((val != CABC_OFF) && (val != CABC_LEVER1) &&
		(val != CABC_LEVER2) && (val != CABC_LEVER3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return snprintf(buf, PAGE_SIZE, "invalid cabc val = %d\n", val);
	}

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}else{
		NUBIA_DISP_ERROR("failed to set cabc as = %d\n", val);
	}
	return snprintf(buf, PAGE_SIZE, "nubia_disp_val.cabc = %d, nubia_disp_val.en_cabc = %d\n", nubia_disp_val.cabc, nubia_disp_val.en_cabc);
}

static ssize_t lcd_power_on_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;

	sscanf(buf, "%d", &val);

	if ((val != CABC_OFF) && (val != CABC_LEVER1) &&
		(val != CABC_LEVER2) && (val != CABC_LEVER3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return size;
	}

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}else{
		NUBIA_DISP_ERROR("failed to set cabc as = %d\n", val);
	}
	return size;
}


static ssize_t lcd_power_off_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;
	if(!nubia_disp_val.en_cabc) {
		NUBIA_DISP_ERROR("no cabc\n");
		return snprintf(buf, PAGE_SIZE, "no cabc\n");
	}

	val = CABC_OFF;

	if ((val != CABC_OFF) && (val != CABC_LEVER1) &&
		(val != CABC_LEVER2) && (val != CABC_LEVER3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return snprintf(buf, PAGE_SIZE, "invalid cabc val = %d\n", val);
        }

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}else{
		NUBIA_DISP_ERROR("failed to set cabc as = %d\n", val);
	}
	nubia_disp_val.en_cabc = 0;
	return snprintf(buf, PAGE_SIZE, "nubia_disp_val.cabc = %d, nubia_disp_val.en_cabc = %d\n", nubia_disp_val.cabc, nubia_disp_val.en_cabc);
}

static ssize_t lcd_power_off_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;
	if(!nubia_disp_val.en_cabc) {
		NUBIA_DISP_ERROR("no cabc\n");
		return size;
	}
	val = CABC_OFF;

	if ((val != CABC_OFF) && (val != CABC_LEVER1) &&
		(val != CABC_LEVER2) && (val != CABC_LEVER3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return size;
	}

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}else{
		NUBIA_DISP_ERROR("failed to set cabc as = %d\n", val);
	}
	nubia_disp_val.en_cabc = 0;
	return size;
}


static ssize_t cabc_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
       if (nubia_disp_val.en_cabc)
                return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.cabc);
        else
                return snprintf(buf, PAGE_SIZE, "NULL\n");

}

static ssize_t cabc_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
        uint32_t val = 0;
        int ret = 0;
	if(!nubia_disp_val.en_cabc) {
                NUBIA_DISP_ERROR("no cabc\n");
                return size;
        }

        sscanf(buf, "%d", &val);

        if ((val != CABC_OFF) && (val != CABC_LEVER1) &&
                (val != CABC_LEVER2) && (val != CABC_LEVER3)) {
                NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
                return size;
        }

        NUBIA_DISP_INFO("cabc value = %d\n", val);

        ret = nubia_set_cabc(val);
        if (ret == 0) {
                nubia_disp_val.cabc = val;
                NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
        }else{
                NUBIA_DISP_ERROR("failed to set cabc as = %d\n", val);
        }
        return size;
}

static struct kobj_attribute lcd_disp_attrs[] = {
	__ATTR(cabc,        0664, cabc_show,       cabc_store),
	__ATTR(lcd_power_on,        0664, lcd_power_on_show,       lcd_power_on_store),
	__ATTR(lcd_power_off,        0664, lcd_power_off_show,       lcd_power_off_store),
	//__ATTR(lcd_debug,  0664, lcd_debug_show, lcd_debug_store),
};

void set_panel_ready_for_cmd(int val){
	panel_ready_for_cmd = val;
}
EXPORT_SYMBOL(set_panel_ready_for_cmd);

void nubia_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata * ctrl)
{
	NUBIA_DISP_INFO("start\n");
	set_panel_ready_for_cmd(1);
	nubia_mdss_dsi_ctrl = ctrl;
}

EXPORT_SYMBOL(nubia_set_dsi_ctrl);

static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}

void nubia_lcd_preference_parse_dt(struct device_node *np)
{
	mdss_dsi_parse_dcs_cmds(np, &cabc_cmds_off,
                "nubia,mdss-dsi-cabc-command-off", "nubia,mdss-dsi-cabc-command-state");
	if(cabc_cmds_off.cmd_cnt == 0)
		pr_err("Unable to read nubia,mdss-dsi-cabc-command-off\n");

	mdss_dsi_parse_dcs_cmds(np, &cabc_cmds_level1,
                "nubia,mdss-dsi-cabc-command-level1", "nubia,mdss-dsi-cabc-command-state");
	if(cabc_cmds_level1.cmd_cnt == 0)
		pr_err("Unable to read nubia,mdss-dsi-cabc-command-level1\n");

	mdss_dsi_parse_dcs_cmds(np, &cabc_cmds_level2,
                "nubia,mdss-dsi-cabc-command-level2", "nubia,mdss-dsi-cabc-command-state");
	if(cabc_cmds_level2.cmd_cnt == 0)
                pr_err("Unable to read nubia,mdss-dsi-cabc-command-level2\n");

	mdss_dsi_parse_dcs_cmds(np, &cabc_cmds_level3,
                "nubia,mdss-dsi-cabc-command-level3", "nubia,mdss-dsi-cabc-command-state");
	if(cabc_cmds_level3.cmd_cnt == 0)
                pr_err("Unable to read nubia,mdss-dsi-cabc-command-level3\n");

}
EXPORT_SYMBOL(nubia_lcd_preference_parse_dt);

//bl curve
uint32_t backlight_curve[256];
void nubia_lcd_bl_curve_parse_dt(struct device_node *np)
{
	int rc = 0;
	rc = of_property_read_u32_array(np, "nubia,mdss-dsi-panel-backlight-curve", backlight_curve,256);
	 if (rc){
		pr_debug("%s:%d, nubia backlight curve array error reading , rc = %d\n",
	                __func__, __LINE__, rc);
		memset(backlight_curve,-1,256);
	}
}
EXPORT_SYMBOL(nubia_lcd_bl_curve_parse_dt);

struct msm_fb_data_type *mfd;
void get_mfd(struct msm_fb_data_type *mfd_para)
{
	mfd = mfd_para;
}
EXPORT_SYMBOL(get_mfd);
/*
 * mdss_fb_report_panel_dead() - Sends the PANEL_ALIVE=0 status to HAL layer.
 * @mfd   : frame buffer structure associated with fb device.
 *
 * This function is called if the panel fails to respond as expected to
 * the register read/BTA or if the TE signal is not coming as expected
 * from the panel. The function sends the PANEL_ALIVE=0 status to HAL
 * layer.
 */
void nubia_fb_report_panel_dead(void)
{
	char *envp[2] = {"PANEL_ALIVE=0", NULL};
	struct mdss_panel_data *pdata =
		dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("Nubia Panel data not available\n");
		return;
	}

	pdata->panel_info.panel_dead = true;
	kobject_uevent_env(&mfd->fbi->dev->kobj,
		KOBJ_CHANGE, envp);
	pr_err("Nubia Panel has gone bad, sending uevent - %s\n", envp[0]);
}
EXPORT_SYMBOL(nubia_fb_report_panel_dead);

int nubia_backlight_covert(struct msm_fb_data_type *mfd,
				      int value, int max_bl)
{
	u32 bl_lvl;

	if(!mfd){
		return -EINVAL;
	}

	if(backlight_curve[0] == 0 && value<256 && value>=0 \
		&& max_bl < 256){
		bl_lvl = backlight_curve[value];
	}else{
		if(value > 0){
			bl_lvl =value * (mfd->panel_info->bl_max -mfd->panel_info->bl_min);
			do_div(bl_lvl,max_bl);
			bl_lvl =value *bl_lvl;
			do_div(bl_lvl,max_bl);
			bl_lvl += mfd->panel_info->bl_min;
		}else{
			bl_lvl =0;
		}
	}
	return bl_lvl;
}
EXPORT_SYMBOL(nubia_backlight_covert);

static int __init nubia_disp_preference_init(void)
{
	int retval = 0;
	int attr_count = 0;

	NUBIA_DISP_INFO("start\n");

	enhance_kobj = kobject_create_and_add("lcd_enhance", kernel_kobj);

	if (!enhance_kobj) {
		NUBIA_DISP_ERROR("failed to create and add kobject\n");
		return -ENOMEM;
	}

	/* Create attribute files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++) {
		retval = sysfs_create_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);
		if (retval < 0) {
			NUBIA_DISP_ERROR("failed to create sysfs attributes\n");
			goto err_sys_creat;
		}
	}

	NUBIA_DISP_ERROR("success\n");

	return retval;

err_sys_creat:
	for (--attr_count; attr_count >= 0; attr_count--)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
	return retval;
}

static void __exit nubia_disp_preference_exit(void)
{
	int attr_count = 0;

	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
	nubia_mdss_dsi_ctrl = NULL;
}

MODULE_AUTHOR("NUBIA LCD Driver Team Software");
MODULE_DESCRIPTION("NUBIA LCD DISPLAY Color Saturation and Temperature Setting");
MODULE_LICENSE("GPL");
module_init(nubia_disp_preference_init);
module_exit(nubia_disp_preference_exit);
