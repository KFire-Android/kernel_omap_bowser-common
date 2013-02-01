# Tate/EVT1.1, Jem/EVT1.2, and older units do not support OOB.
# To explicitly build for the older devices: "make bcmdhd OOB_SUPPORT=no"
#
# The default build is for EVT 2 and beyond, and the driver relies on 
# the board id to switch to non-OOB interrupt in runtime.
OOB_SUPPORT := yes

ifeq ($(OOB_SUPPORT),no)
OOB_FLAGS =
else
OOB_FLAGS = -DCUSTOM_OOB_GPIO_NUM=204 -DOOB_INTR_ONLY -DHW_OOB
endif

# bcmdhd
#	-DCUSTOMER_HW2 -DCUSTOM_OOB_GPIO_NUM=2 -DOOB_INTR_ONLY -DHW_OOB       \

# Turn off -DDHD_DEBUG
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
	-DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DSDTEST -DBDC -DTOE    \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DBCMDBG               \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
	-DKEEP_ALIVE -DCSCAN -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT     \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-DCUSTOMER_HW2 -DSET_RANDOM_MAC_SOFTAP -DWL_CFG80211_STA_EVENT        \
	$(OOB_FLAGS)                                                          \
	-Idrivers/net/wireless/bcmdhd -Idrivers/net/wireless/bcmdhd/include


#	-DCUSTOM_OOB_GPIO_NUM=242 -DOOB_INTR_ONLY -DHW_OOB                    \
#	-DFORCED_TO_2G_ONLY	\
#
#
# KS: 
# (1) If not using OOB interrupt, comment out the following line
#	-DCUSTOM_OOB_GPIO_NUM=nnn -DOOB_INTR_ONLY -DHW_OOB       \
#
# (2) For OOB: Add the line above, where nnn should be programmed as below:
#   (2.1)  If GPIO_WALN_HOST_WAKE = 44 => -DCUSTOM_OOB_GPIO_NUM=204
#          If GPIO_WALN_HOST_WAKE = 82 => -DCUSTOM_OOB_GPIO_NUM=242
#
#   (2.2)  If CUSTOM_OOB_GPIO_NUM is not defined here, driver picks up the irq number
#          from bowser_wifi_irq_num, exported from board_bowser3.c.
#   (2.3)  To force to use 2.4 GHz band only (due to interoperability problem with Aruba AP's):
#          -DFORCED_TO_2G_ONLY
#   (2.4)  To force to use 5 GHz band only:
#          -DFORCED_TO_5G_ONLY
#   (2.5)  To set 5G preference during roaming-rejoining:
#          -DPREFER_ASSOCIATION_5G

DHDCFLAGS += -DBCMDHD_PRODUCTION_BUILD

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o bcmwifi.o dhd_sdio.o bcmevent.o dhd_bta.o hndpmu.o  \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o dhd_cfg80211.o

obj-$(CONFIG_BCMDHD) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o
DHDCFLAGS += -DWL_CFG80211
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-65
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
ifneq ($(CONFIG_DHD_ENABLE_P2P),)
DHDCFLAGS += -DWL_ENABLE_P2P_IF
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif
