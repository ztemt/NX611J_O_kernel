#include "nubia_lcd_feature.h"

int vddio_post_on_delay = -1;
int vddio_pre_off_delay = -1;
int vddio_gpio_num = -1;
int tp_reset_gpio = 0;

extern uint8_t nubia_is_smwp_en;

int nubia_hx_smwp_en(void)
{
	return nubia_is_smwp_en;
}
EXPORT_SYMBOL(nubia_hx_smwp_en);

static int nubia_lcd_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	pr_err("%s:  gpio setup gpio = %d , config = %d, dir = %d, state = %d",
			__func__, gpio, config, dir, state);

	if (config) {
		snprintf(buf, PAGE_SIZE, "lcd_gpio_%u\n", gpio);

		if (gpio_is_valid(gpio)) {
		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
		gpio_set_value(gpio, state);
	} else {
			pr_err("%s: gpio is not valid while enable%d\n",
						__func__, gpio);
		}
	} else {
		if (gpio_is_valid(gpio)) {
			gpio_set_value(gpio, 0);
			gpio_free(gpio);
		} else {
			pr_err("%s: gpio is not valid while disable%d\n",
						__func__, gpio);
		}
	}

	return retval;
}

int nubia_set_tp_reset(bool enable){
	int ret = 0;
	if(tp_reset_gpio > 0 ){
		//get GPIO num from panel dtsi and delay time
		NUBIA_LCD_FEATURE_ERROR("enter\n");
		if(enable){
			ret = nubia_lcd_gpio_setup(
					tp_reset_gpio,
					true, 1, 1);
			if (ret < 0) {
				NUBIA_LCD_FEATURE_ERROR("gpio %d set true fail\n", tp_reset_gpio);
			}
		}else{
			ret = nubia_lcd_gpio_setup(
					tp_reset_gpio,
					false, 0, 0);
			if (ret < 0) {
				NUBIA_LCD_FEATURE_ERROR("gpio %d set false fail\n", tp_reset_gpio);
			}
		}
	}
	return ret;
}


int nubia_set_vddio_power(bool enable){
	int ret = 0;
	if(vddio_gpio_num > 0 || vddio_post_on_delay > 0 || vddio_pre_off_delay > 0){
		//get GPIO num from panel dtsi and delay time
		NUBIA_LCD_FEATURE_ERROR("enter\n");
		if(enable){
			ret = nubia_lcd_gpio_setup(
					vddio_gpio_num,
					true, 1, 1);
			if (ret < 0) {
				NUBIA_LCD_FEATURE_ERROR("gpio %d set true fail\n", vddio_gpio_num);
			}

			usleep_range(vddio_post_on_delay, vddio_post_on_delay);
		}else{
			usleep_range(vddio_pre_off_delay, vddio_pre_off_delay);
			ret = nubia_lcd_gpio_setup(
					vddio_gpio_num,
					false, 0, 0);
			if (ret < 0) {
				NUBIA_LCD_FEATURE_ERROR("gpio %d set false fail\n", vddio_gpio_num);
			}
		}
	}
	return ret;
}
// It control by macros NUBIA_LCD_VDDIO_CONTROL_GPIO
EXPORT_SYMBOL(nubia_set_vddio_power);

int nubia_panel_id = -1;
void nubia_mdss_mdp_get_pan_id(char *panel_name)
{
	if (!panel_name ) {
		pr_err("invalid panel name\n");
		return;
	}

	NUBIA_LCD_FEATURE_ERROR("panel_name:%s\n", panel_name);

	if (strnstr(panel_name, "qcom,mdss_dsi_lead_hx83112a_1080_2160_5p65_video", MDSS_MAX_PANEL_LEN))
		nubia_panel_id = 1;
	else if (strnstr(panel_name, "qcom,mdss_dsi_jdi_hx83112a_1080_2160_5p65_video", MDSS_MAX_PANEL_LEN))
		nubia_panel_id = 0;
	else
		pr_err("do not find valid panel id\n");
	NUBIA_LCD_FEATURE_ERROR("nubia_panel_id:%d\n", nubia_panel_id);
}
// It control by macros NUBIA_LCD_PANEL_ID
EXPORT_SYMBOL(nubia_mdss_mdp_get_pan_id);

int nubia_lcd_feature_parse_dt(struct device_node *np)
{
	int tmp = 0;
	int rc=0;
	rc = of_property_read_u32(np, "qcom,nubia-vddio-on-post-delay", &tmp);
	vddio_post_on_delay = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,nubia-vddio-off-pre-delay", &tmp);
	vddio_pre_off_delay = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,nubia-vddio-gpio-num", &tmp);
	vddio_gpio_num = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,nubia-tp-reset-gpio", &tmp);
	tp_reset_gpio = (!rc ? tmp : 0);
	return rc;
}
// It control by macros NUBIA_LCD_FEATURE
EXPORT_SYMBOL(nubia_lcd_feature_parse_dt);

static int __init nubia_lcd_feature_init(void)
{
	int retval = 0;
	// parse dt
	NUBIA_LCD_FEATURE_ERROR("enter\n");
	return retval;
}

static void __exit nubia_lcd_feature_exit(void)
{
	NUBIA_LCD_FEATURE_ERROR("enter\n");
}

MODULE_AUTHOR("NUBIA LCD Driver Team Software");
MODULE_LICENSE("GPL");
module_init(nubia_lcd_feature_init);
module_exit(nubia_lcd_feature_exit);

