#ifndef _NUBIA_LCD_FEATURE_PREFERENCE_
#define _NUBIA_LCD_FEATURE_PREFERENCE_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define MDSS_MAX_PANEL_LEN      256

#define NUBIA_LCD_FEATURE_LOG_TAG "LCDFeature"
#define NUBIA_LCD_FEATURE_LOG_ON

#ifdef NUBIA_LCD_FEATURE_LOG_ON
#define NUBIA_LCD_FEATURE_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	NUBIA_LCD_FEATURE_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define NUBIA_LCD_FEATURE_INFO(fmt, args...) printk(KERN_INFO "[%s] [%s: %d] "  fmt, \
	NUBIA_LCD_FEATURE_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define  NUBIA_LCD_FEATURE_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt, \
	NUBIA_LCD_FEATURE_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define NUBIA_LCD_FEATURE_ERROR(fmt, args...)
#define NUBIA_LCD_FEATURE_INFO(fmt, args...)
#define NUBIA_LCD_FEATURE_DEBUG(fmt, args...)
#endif


int nubia_set_vddio_power(bool enable);
void nubia_mdss_mdp_get_pan_id(char *panel_name);
int nubia_lcd_feature_parse_dt(struct device_node *np);
int nubia_hx_smwp_en(void);
int nubia_set_tp_reset(bool enable);

#endif

