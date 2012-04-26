/*
 * ALSA SoC HMDI codec driver
 *
 * Contact: Ricardo Neri <ricardo.neri@ti.com>
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

#ifndef __OMAP_HDMI_CODEC_H
#define __OMAP_HDMI_CODEC_H

int hdmi_audio_get_max_channels(void);
void hdmi_audio_update_edid_info(void);

#endif
