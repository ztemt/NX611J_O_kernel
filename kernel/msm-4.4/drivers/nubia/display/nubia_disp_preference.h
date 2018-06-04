#ifndef _NUBIA_DISP_PREFERENCE_
#define _NUBIA_DISP_PREFERENCE_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include "../../video/fbdev/msm/mdss_mdp.h"
#include "../../video/fbdev/msm/mdss_dsi.h"
#include "../../video/fbdev/msm/mdss_mdp_pp.h"
#include "../../video/fbdev/msm/mdss_dsi_cmd.h"
#include "../../video/fbdev/msm/mdss_fb.h"

/* ------------------------- General Macro Definition ------------------------*/
enum {
	CABC_OFF = 23,
	CABC_LEVER1 ,
	CABC_LEVER2 ,
	CABC_LEVER3
};
#define NUBIA_DISP_LOG_TAG "ZtemtDisp"
#define NUBIA_DISP_LOG_ON

#ifdef  NUBIA_DISP_LOG_ON
#define NUBIA_DISP_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	NUBIA_DISP_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define NUBIA_DISP_INFO(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	NUBIA_DISP_LOG_TAG, __FUNCTION__, __LINE__, ##args)

    #ifdef  NUBIA_DISP_DEBUG_ON
#define  NUBIA_DISP_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt, \
	NUBIA_DISP_LOG_TAG, __FUNCTION__, __LINE__, ##args)
    #else
#define NUBIA_DISP_DEBUG(fmt, args...)
    #endif
#else
#define NUBIA_DISP_ERROR(fmt, args...)
#define NUBIA_DISP_INFO(fmt, args...)
#define NUBIA_DISP_DEBUG(fmt, args...)
#endif

/* ----------------------------- Structure ----------------------------------*/
struct nubia_disp_type{
  int en_cabc;
  unsigned int cabc;
};

/* ------------------------- Function Declaration ---------------------------*/
void nubia_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata * ctrl);
void nubia_disp_preference(void);
void nubia_lcd_preference_parse_dt(struct device_node *np);
void set_panel_ready_for_cmd(int val);
int nubia_backlight_covert(struct msm_fb_data_type *mfd, int value, int max_bl);
void nubia_lcd_bl_curve_parse_dt(struct device_node *np);
void get_mfd(struct msm_fb_data_type *mfd_para);
void nubia_fb_report_panel_dead(void);
#endif
