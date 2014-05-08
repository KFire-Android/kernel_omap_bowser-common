/*
 * bowser.c  --  SoC audio for TI OMAP4430 SDP
 *
 * Author: Misael Lopez Cruz <x0052729@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <sound/soc-dsp.h>
#include <asm/mach-types.h>
#include <plat/hardware.h>
#include <plat/mux.h>
#include <plat/mcbsp.h>

#include "omap-mcpdm.h"
#include "omap-abe.h"
#include "omap-pcm.h"
#include "omap-mcbsp.h"
#include "omap-dmic.h"

#include "../codecs/wm8962.h"

#ifdef CONFIG_SND_OMAP_SOC_HDMI
#include "omap-hdmi.h"
#endif

#include "abe/abe_main.h"

#define ALSA_DEBUG
#include "bowser_alsa_debug.h"

#define WM8962_MCLK_RATE 19200000

#ifdef CONFIG_ABE_44100
#define WM8962_SYS_CLK_RATE	(44100 * 512)
#else
#define WM8962_SYS_CLK_RATE	(48000 * 512)
#endif

/* static struct regulator *av_switch_reg; */
static struct clk *wm8962_mclk;
static unsigned int fll_clk = WM8962_SYS_CLK_RATE;
static unsigned int sys_clk = WM8962_SYS_CLK_RATE;

static struct snd_soc_dai *codec_dai;
static enum snd_soc_bias_level bias_level = SND_SOC_BIAS_OFF;
static struct snd_soc_jack bowser_jack;

static int bowser_set_bias_level(struct snd_soc_card *card,
				 struct snd_soc_dapm_context *dapm,
				 enum snd_soc_bias_level level)
{
        int ret;
	if (codec_dai->dev == NULL){
		pr_err("no run time codec_dai initialized yet\n");
		return -EINVAL;
	}

	dev_dbg(codec_dai->dev, "Setting bias %d\n", level);

	if (dapm->dev != codec_dai->dev) {
		dev_dbg(dapm->dev,"dapm->dev!=codec_dai->dev\n");
		return 0;
        }

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (bias_level == SND_SOC_BIAS_OFF) {
		        ret = clk_enable(wm8962_mclk);
			if (ret < 0) {
				dev_err(codec_dai->dev,
				        "Failed to enable MCLK: %d\n", ret);
				return ret;
			}
		}
		break;
	case SND_SOC_BIAS_PREPARE:
		if (bias_level == SND_SOC_BIAS_STANDBY) {
			ret = snd_soc_dai_set_pll(codec_dai, WM8962_FLL,
						  WM8962_FLL_MCLK,
						  WM8962_MCLK_RATE,
						  fll_clk);
			if (ret < 0) {
			        dev_err(codec_dai->dev,
					"Failed to start CODEC FLL: %d\n",
					ret);
				return ret;
			}

			ret = snd_soc_dai_set_sysclk(codec_dai,
						     WM8962_SYSCLK_FLL,
						     sys_clk, 0);
			if (ret < 0) {
			        dev_err(codec_dai->dev,
					"Failed to set CODEC SYSCLK: %d\n",
					ret);
				return ret;
			}
		}
		break;
        default:
		break;
	}

	return 0;
}

static int bowser_set_bias_level_post(struct snd_soc_card *card,
				      struct snd_soc_dapm_context *dapm,
				      enum snd_soc_bias_level level)
{
        int ret;
	if (codec_dai->dev == NULL){
		pr_err("no run time codec_dai initialized yet\n");
		return -EINVAL;
	}
	dev_dbg(codec_dai->dev, "Setting bias post %d\n", level);

	if (dapm->dev != codec_dai->dev) {
		dev_dbg(dapm->dev,"dapm->dev!=codec_dai->dev\n");
		return 0;
        }

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (bias_level == SND_SOC_BIAS_PREPARE) {
			ret = snd_soc_dai_set_sysclk(codec_dai,
						     WM8962_SYSCLK_MCLK,
						     sys_clk, 0);
			if (ret < 0) {
			        dev_err(codec_dai->dev,
					"Failed to set CODEC SYSCLK: %d\n",
					ret);
				return ret;
			}

			ret = snd_soc_dai_set_pll(codec_dai, WM8962_FLL,
						  WM8962_FLL_MCLK, 0, 0);

			if (ret < 0) {
			        dev_err(codec_dai->dev,
					"Failed to stop CODEC FLL: %d\n", ret);
				return ret;
			}
	        }
		break;

	case SND_SOC_BIAS_OFF:
	       clk_disable(wm8962_mclk);
	       break;

	default:
	        break;
	}

	bias_level = level;

	return 0;
}

static int bowser_suspend_pre(struct snd_soc_card *card)
{
	dev_crit(codec_dai->dev,"%s\n",__func__);
	if (codec_dai->dev == NULL){
		dev_err(codec_dai->dev,"no run time codec_dai initialized yet\n");
		return -EINVAL;
	}
	snd_soc_dapm_disable_pin(&codec_dai->codec->dapm, "SYSCLK");
	snd_soc_dapm_sync(&codec_dai->codec->dapm);
	msleep(10);
	snd_soc_dapm_disable_pin(&codec_dai->codec->dapm, "MICBIAS");
	snd_soc_dapm_sync(&codec_dai->codec->dapm);
	return 0;
}

static int bowser_resume_post(struct snd_soc_card *card)
{
	int ret=0;
	dev_dbg(codec_dai->dev,"%s is calling mic_detect\n", __func__);
	ret = wm8962_mic_detect(codec_dai->codec, &bowser_jack);
	return ret;
}

static int bowser_wm8962_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	dev_dbg(codec_dai->dev, "%s() - enter\n", __func__);

	sys_clk = fll_clk =  WM8962_SYS_CLK_RATE;
	ret = snd_soc_dai_set_pll(codec_dai, WM8962_FLL, WM8962_FLL_MCLK,
				  WM8962_MCLK_RATE, fll_clk);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Failed to start CODEC FLL: %d\n",
			ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8962_SYSCLK_FLL,
				     sys_clk, 0);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Failed to set CODEC SYSCLK: %d\n",
			ret);
		return ret;
	}
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_CBM_CFM |
				  SND_SOC_DAIFMT_NB_NF);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Failed to set CODEC DAI format: %d\n",
			ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_CBM_CFM |
				  SND_SOC_DAIFMT_NB_NF);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "Failed to set CPU DAI format: %d\n",
			ret);
		return ret;
	}

	dev_dbg(codec_dai->dev, "%s() - exit\n", __func__);

	return 0;
}

static int bowser_abe_wm8962_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	dev_dbg(codec_dai->dev, "%s() - enter\n", __func__);

	if (wm8962_mclk != NULL)
		clk_enable(wm8962_mclk);

	dev_dbg(codec_dai->dev, "%s() - exit\n", __func__);

	return 0;
}

static void bowser_abe_wm8962_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	dev_dbg(codec_dai->dev, "%s() - enter\n", __func__);

	if (wm8962_mclk != NULL)
		clk_disable(wm8962_mclk);

	dev_dbg(codec_dai->dev, "%s() - exit\n", __func__);
}

static struct snd_soc_ops bowser_abe_ops = {
  /*	.startup = bowser_abe_wm8962_startup,
	.shutdown = bowser_abe_wm8962_shutdown,*/  /*may need them for extra stuffs TI wants*/
	.hw_params = bowser_wm8962_hw_params,
};


static int bowser_mcbsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned int be_id, channels;

	ret = snd_soc_dai_set_fmt(cpu_dai,
			  SND_SOC_DAIFMT_DSP_C |
			  SND_SOC_DAIFMT_NB_NF |
			  SND_SOC_DAIFMT_CBM_CFM);

	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	omap_mcbsp_set_tx_threshold(cpu_dai->id, 1);

	return ret;
}

static struct snd_soc_ops bowser_mcbsp_ops = {
	.hw_params = bowser_mcbsp_hw_params,
};

static const struct snd_soc_dapm_widget bowser_dapm_widgets[] = {
	SND_SOC_DAPM_HP("HP", NULL),
	SND_SOC_DAPM_SPK("SPK", NULL),
};

static const struct snd_kcontrol_new bowser_controls[] = {
	SOC_DAPM_PIN_SWITCH("DMICDAT"),
	SOC_DAPM_PIN_SWITCH("HP"),
	SOC_DAPM_PIN_SWITCH("SPK"),
};

static const struct snd_soc_dapm_route bowser_dapm_routes[] = {
	{ "HP", NULL, "HPOUTL" },
	{ "HP", NULL, "HPOUTR" },

	{ "SPK", NULL, "SPKOUTL" },
	{ "SPK", NULL, "SPKOUTR" },
};

static int bowser_wm8962_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	codec_dai = rtd->codec_dai;
	ret = snd_soc_dapm_new_controls(dapm, bowser_dapm_widgets,
					ARRAY_SIZE(bowser_dapm_widgets));
	if (ret < 0)
		pr_err("Failed to register DAPM widgets for Bowser\n");

	ret = snd_soc_dapm_add_routes(dapm, bowser_dapm_routes,
				      ARRAY_SIZE(bowser_dapm_routes));
	if (ret < 0)
		pr_err("Failed to register DAPM routes for Bowser\n");

	ret = snd_soc_add_controls(codec, bowser_controls,
				   ARRAY_SIZE(bowser_controls));
	if (ret < 0)
		pr_err("Failed to add Bowser controls\n");

	ret = snd_soc_jack_new(codec, "h2w",
			      SND_JACK_HEADSET | SND_JACK_HEADPHONE,
			       &bowser_jack);
	if (ret) {
		pr_err("Failed to create jack: %d\n", ret);
		return ret;
	}

	snd_jack_set_key(bowser_jack.jack, SND_JACK_BTN_0, KEY_MEDIA);

	ret = wm8962_get_jack(codec, &bowser_jack);
	if (ret) {
		pr_err("Failed to get jack: %d\n", ret);
		return ret;
	}

	return 0;
}

static int mcbsp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_interval *channels = hw_param_interval(params,
							  SNDRV_PCM_HW_PARAM_CHANNELS);
	unsigned int be_id = rtd->dai_link->be_id;
	unsigned int threshold;

	switch (be_id) {
	case OMAP_ABE_DAI_MM_FM:
		channels->min = 2;
		threshold = 2;
		break;
	case OMAP_ABE_DAI_BT_VX:
		channels->min = 1;
		threshold = 1;
		break;
	default:
		threshold = 1;
		break;
	}

	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S16_LE);

	omap_mcbsp_set_tx_threshold(cpu_dai->id, threshold);
	omap_mcbsp_set_rx_threshold(cpu_dai->id, threshold);

	return 0;
}

static struct snd_soc_dai_driver bt_dai[] = {
{
	.name = "Bluetooth",
	.playback = {
		.stream_name = "BT Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "BT Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
}
};

struct snd_soc_dsp_link fe_lp_media = {
	.playback   = true,
	.trigger    =
		{SND_SOC_DSP_TRIGGER_BESPOKE, SND_SOC_DSP_TRIGGER_BESPOKE},
};

struct snd_soc_dsp_link fe_media = {
	.playback   = true,
	.capture    = true,
	.trigger    =
		{SND_SOC_DSP_TRIGGER_BESPOKE, SND_SOC_DSP_TRIGGER_BESPOKE},
};

struct snd_soc_dsp_link fe_media_capture = {
	.capture    = true,
	.trigger    =
		{SND_SOC_DSP_TRIGGER_BESPOKE, SND_SOC_DSP_TRIGGER_BESPOKE},
};
/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link bowser_dai[] = {
	{
		.name = "wm8962-lp",
		.stream_name = "Multimedia",
		/* ABE components - MM-DL (mmap) */
		.cpu_dai_name = "MultiMedia1 LP",
		.platform_name = "aess",
		.dynamic = 1, /* BE is dynamic */
		.dsp_link = &fe_lp_media,
	},
	{
		.name = "wm8962",
		.stream_name = "Multimedia",
		/* ABE components - MM-UL & MM_DL */
		.cpu_dai_name = "MultiMedia1",
		.platform_name = "omap-pcm-audio",
		.dynamic = 1, /* BE is dynamic */
		.dsp_link = &fe_media,
	},
	{
		.name = "wm8962-noabe",
		.stream_name = "wm8962",
		.cpu_dai_name = "omap-mcbsp-dai.1",   /* McBSP2 */
		.platform_name = "omap-pcm-audio",
		.codec_dai_name = "wm8962",
		.codec_name = "wm8962.3-001a",
		.ops = &bowser_abe_ops,
		.init = &bowser_wm8962_init,
	},
	{
		.name = "wm8962-mm-ul2",
		.stream_name = "Multimedia Capture",
		/* ABE components - MM-UL2 */
		.cpu_dai_name = "MultiMedia2",
		.platform_name = "omap-pcm-audio",
		.dynamic = 1, /* BE is dynamic */
		.dsp_link = &fe_media_capture,
	},
	{
		.name = "BT Playback",
		.stream_name = "Bluetooth Playback",
		/* MCBSP3 ->BT SCO  */
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.platform_name = "omap-pcm-audio",
		/* Bluetooth */
		.codec_dai_name = "Bluetooth",
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.ops = &bowser_mcbsp_ops,
	},
	{
		.name = "BT Capture",
		.stream_name = "Bluetooth Capture",
		/* MCBSP3 <-BT SCO  */
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.platform_name = "omap-pcm-audio",
		/* Bluetooth */
		.codec_dai_name = "Bluetooth",
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.ops = &bowser_mcbsp_ops,
	},
/*
 * Backend DAIs - i.e. dynamically matched interfaces, invisible to userspace.
 * Matched to above interfaces at runtime, based upon use case.
 */
	{
		.name = OMAP_ABE_BE_MM_EXT0_DL,
		.stream_name = "PCM Playback",
		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "aess",
		/* FM */
		.codec_dai_name = "wm8962",
		.codec_name = "wm8962.3-001a",
		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &bowser_abe_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
	},
	{
		.name = OMAP_ABE_BE_MM_EXT0_UL,
		.stream_name = "PCM Capture",
		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "aess",
		/* FM */
		.codec_dai_name = "wm8962",
		.codec_name = "wm8962.3-001a",
		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &bowser_abe_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
	},
};
/* Audio machine driver */
static struct snd_soc_card snd_soc_bowser = {
	.name = "bowser",
	.long_name = "TI OMAP4 bowser Board",
	.dai_link = bowser_dai,
	.num_links = ARRAY_SIZE(bowser_dai),
	.set_bias_level = bowser_set_bias_level,
	.set_bias_level_post = bowser_set_bias_level_post,
	.suspend_pre = bowser_suspend_pre,
	.resume_post = bowser_resume_post,
};

static struct platform_device *bowser_snd_device;

static int __init bowser_soc_init(void)
{
	int ret = 0;

	if (!machine_is_omap_4430sdp() &&
	    !machine_is_omap4_panda() &&
	    !machine_is_omap4_bowser()) {
		pr_debug("Not bowser or PandaBoard!\n");
		return -ENODEV;
	}

	wm8962_mclk = clk_get(NULL, "auxclk0_ck");
	if (IS_ERR(wm8962_mclk)) {
		pr_err("Failed to get WM8962 MCLK: %ld\n",
			PTR_ERR(wm8962_mclk));
		return -ENODEV;
	}

	pr_debug("Old codec mclk rate = %lu\n", clk_get_rate(wm8962_mclk));

	ret = clk_set_rate(wm8962_mclk, WM8962_MCLK_RATE);
	if (ret < 0) {
		pr_err("Failed to set MCLK rate: %d\n", ret);
		goto clk_err;
	}

	pr_debug("New codec mclk rate = %lu\n", clk_get_rate(wm8962_mclk));

	bowser_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bowser_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		ret = -ENOMEM;
		goto clk_err;
	}

	ret = snd_soc_register_dais(&bowser_snd_device->dev,
					bt_dai, ARRAY_SIZE(bt_dai));
	if (ret < 0)
		goto dai_err;

	platform_set_drvdata(bowser_snd_device, &snd_soc_bowser);

	ret = platform_device_add(bowser_snd_device);
	if (ret) {
		pr_err("Couldn't add bowser snd device ret: %d\n", ret);
		goto plat_err;
	}

	/*	av_switch_reg = regulator_get(&bowser_snd_device->dev, "av-switch");
	  if (IS_ERR(av_switch_reg)) {
	  ret = PTR_ERR(av_switch_reg);
	  printk(KERN_ERR "couldn't get AV Switch regulator %d\n",
	  ret);
	  goto reg_err;
	  }*/

	return ret;

reg_err:
	platform_device_del(bowser_snd_device);
plat_err:
	platform_device_put(bowser_snd_device);
clk_err:
	clk_put(wm8962_mclk);
dai_err:
	return ret;
}
module_init(bowser_soc_init);

static void __exit bowser_soc_exit(void)
{
	/* regulator_put(av_switch_reg); */
	platform_device_unregister(bowser_snd_device);
	clk_put(wm8962_mclk);
}
module_exit(bowser_soc_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("ALSA SoC bowser");
MODULE_LICENSE("GPL");

