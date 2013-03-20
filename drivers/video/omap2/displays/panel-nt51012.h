#ifndef __PANEL_NT51012_H__
#define __PANEL_NT51012_H__

#define NT51012_WIDTH		800
#define NT51012_HEIGHT		1280
#define NT51012_PCLK		66800
/* DISPC timings */
#define NT51012_HFP		102
#define NT51012_HSW		9
#define NT51012_HBP		1
#define NT51012_VFP		10
#define NT51012_VSW		1
#define NT51012_VBP		10

static ssize_t nt51012_set_cabc_mode(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count);

static ssize_t nt51012_show_cabc_mode(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf);

static ssize_t nt51012_show_cabc_modes(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf);

static ssize_t nt51012_set_gamma_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count);

static ssize_t nt51012_show_gamma_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf);

static ssize_t nt51012_set_pwm_step_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count);

static ssize_t nt51012_show_pwm_step_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf);

static ssize_t nt51012_set_pwm_frame_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count);

static ssize_t nt51012_show_pwm_frame_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf);

static int nt51012_suspend(struct omap_dss_device *dssdev);

static int nt51012_resume(struct omap_dss_device *dssdev);

static int nt51012_reset(struct omap_dss_device *dssdev);

static void panel_reset_work(struct work_struct *work);

static void nt51012_queue_reset_work(struct omap_dss_device *dssdev);

static void nt51012_cancel_reset_work(struct omap_dss_device *dssdev);

#endif
