/*
 * Panasonic MN86471A DP->HDMI bridge driver (via PS4 Aeolia ICC interface)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


// TODO (ps4patches): Make functions atomic,
//  https://lore.kernel.org/linux-arm-kernel/20211020181901.2114645-5-sam@ravnborg.org/

#include <asm/ps4.h>

#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>

#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#define CMD_READ	1, 1
#define CMD_WRITE	2, 2
#define CMD_MASK	2, 3
#define CMD_DELAY	3, 1
#define CMD_WAIT_SET	3, 2
#define CMD_WAIT_CLEAR	3, 3

#define TSYSCTRL 0x7005
#define TSYSCTRL_HDMI BIT(7)

#define TSRST 0x7006
#define TSRST_AVCSRST BIT(0)
#define TSRST_ENCSRST BIT(1)
#define TSRST_FIFOSRST BIT(2)
#define TSRST_CCSRST BIT(3)
#define TSRST_HDCPSRST BIT(4)
#define TSRST_AUDSRST BIT(6)
#define TSRST_VIFSRST BIT(7)

#define TMONREG 0x7008
#define TMONREG_HPD BIT(3)

#define TDPCMODE 0x7009


#define UPDCTRL 0x7011
#define UPDCTRL_ALLUPD BIT(7)
#define UPDCTRL_AVIIUPD BIT(6)
#define UPDCTRL_AUDIUPD BIT(5)
#define UPDCTRL_CLKUPD BIT(4)
#define UPDCTRL_HVSIUPD BIT(3)
#define UPDCTRL_VIFUPD BIT(2)
#define UPDCTRL_AUDUPD BIT(1)
#define UPDCTRL_CSCUPD BIT(0)


#define VINCNT 0x7040
#define VINCNT_VIF_FILEN BIT(6)

#define VMUTECNT 0x705f
#define VMUTECNT_CCVMUTE BIT(7)
#define VMUTECNT_DUMON BIT(6)
#define VMUTECNT_LINEWIDTH_80 (0<<4)
#define VMUTECNT_LINEWIDTH_90 (1<<4)
#define VMUTECNT_LINEWIDTH_180 (2<<4)
#define VMUTECNT_LINEWIDTH_360 (3<<4)
#define VMUTECNT_VMUTE_MUTE_ASYNC 1
#define VMUTECNT_VMUTE_MUTE_NORMAL 2
#define VMUTECNT_VMUTE_MUTE_RAMPA 4
#define VMUTECNT_VMUTE_MUTE_RAMPB 8
#define VMUTECNT_VMUTE_MUTE_COLORBAR_RGB 10
#define VMUTECNT_VMUTE_MUTE_TOGGLE 12
#define VMUTECNT_VMUTE_MUTE_COLORBAR_YCBCR 14

#define CSCMOD 0x70c0
#define C420SET 0x70c2
#define OUTWSET 0x70c3

#define PKTENA 0x7202

#define INFENA 0x7203
#define INFENA_AVIEN BIT(6)

#define AKESTA 0x7a84
#define AKESTA_BUSY BIT(0)

#define AKESRST 0x7a88

#define HDCPEN 0x7a8b
#define HDCPEN_NONE 0x00
#define HDCPEN_ENC_EN 0x03
#define HDCPEN_ENC_DIS 0x05

#define PCI_DEVICE_ID_CUH_11XX 0x9920
#define PCI_DEVICE_ID_CUH_12XX 0x9922
#define PCI_DEVICE_ID_CUH_2XXX 0x9923
#define PCI_DEVICE_ID_CUH_7XXX 0x9924

struct edid *drm_get_edid(struct drm_connector *connector,
 				 struct i2c_adapter *adapter);

struct i2c_cmd_hdr {
	u8 major;
	u8 length;
	u8 minor;
	u8 count;
} __packed;

struct i2c_cmdqueue {
	struct {
		u8 code;
		u16 length;
		u8 count;
		u8 cmdbuf[0x7ec];
	} __packed req;
	struct {
		u8 res1, res2;
		u8 unk1, unk2;
		u8 count;
		u8 databuf[0x7eb];
	} __packed reply;

	u8 *p;
	struct i2c_cmd_hdr *cmd;
};

struct ps4_bridge {
	struct device *dev;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;

	struct i2c_cmdqueue cq;
	struct mutex mutex;
	int mode;
};

static void cq_init(struct i2c_cmdqueue *q, u8 code)
{
	q->req.code = code;
	q->req.count = 0;
	q->p = q->req.cmdbuf;
	q->cmd = NULL;
}

static void cq_cmd(struct i2c_cmdqueue *q, u8 major, u8 minor)
{
	if (!q->cmd || q->cmd->major != major || q->cmd->minor != minor) {
		if (q->cmd)
			q->cmd->length = q->p - (u8 *)q->cmd;
		q->cmd = (struct i2c_cmd_hdr *)q->p;
		q->cmd->major = major;
		q->cmd->minor = minor;
		q->cmd->length = 0;
		q->cmd->count = 1;
		q->req.count += 1;
		q->p += sizeof(*q->cmd);
	} else {
		q->cmd->count += 1;
	}
}

static int cq_exec(struct i2c_cmdqueue *q)
{
	int res;

	if (!q->cmd)
		return 0;

	q->cmd->length = q->p - (u8 *)q->cmd;
	q->req.length = q->p - (u8 *)&q->req;

	res = apcie_icc_cmd(0x10, 0, &q->req, q->req.length,
		      &q->reply, sizeof(q->reply));

	if (res < 5) {
		// DRM_ERROR("icc i2c commandqueue failed: %d\n", res);
		return -EIO;
	}
	if (q->reply.res1 != 0 || q->reply.res2) {
		// DRM_ERROR("icc i2c commandqueue failed: %d, %d\n",
		// 	  q->reply.res1, q->reply.res2);
		return -EIO;
	}

	return res;
}

static void cq_read(struct i2c_cmdqueue *q, u16 addr, u8 count)
{
	cq_cmd(q, CMD_READ);
	*q->p++ = count;
	*q->p++ = addr >> 8;
	*q->p++ = addr & 0xff;
	*q->p++ = 0;
}

static void cq_writereg(struct i2c_cmdqueue *q, u16 addr, u8 data)
{
	cq_cmd(q, CMD_WRITE);
	*q->p++ = 1;
	*q->p++ = addr >> 8;
	*q->p++ = addr & 0xff;
	*q->p++ = data;
}

#if 0
static void cq_write(struct i2c_cmdqueue *q, u16 addr, u8 *data, u8 count)
{
	cq_cmd(q, CMD_WRITE);
	*q->p++ = count;
	*q->p++ = addr >> 8;
	*q->p++ = addr & 0xff;
	while (count--)
		*q->p++ = *data++;
}
#endif

static void cq_mask(struct i2c_cmdqueue *q, u16 addr, u8 value, u8 mask)
{
	cq_cmd(q, CMD_MASK);
	*q->p++ = 1;
	*q->p++ = addr >> 8;
	*q->p++ = addr & 0xff;
	*q->p++ = value;
	*q->p++ = mask;
}

#if 1
static void cq_delay(struct i2c_cmdqueue *q, u16 time)
{
	cq_cmd(q, CMD_DELAY);
	*q->p++ = 0;
	*q->p++ = time & 0xff;
	*q->p++ = time>>8;
	*q->p++ = 0;
}
#endif

static void cq_wait_set(struct i2c_cmdqueue *q, u16 addr, u8 mask)
{
	cq_cmd(q, CMD_WAIT_SET);
	*q->p++ = 0;
	*q->p++ = addr >> 8;
	*q->p++ = addr & 0xff;
	*q->p++ = mask;
}

static void cq_wait_clear(struct i2c_cmdqueue *q, u16 addr, u8 mask)
{
	cq_cmd(q, CMD_WAIT_CLEAR);
	*q->p++ = 0;
	*q->p++ = addr >> 8;
	*q->p++ = addr & 0xff;
	*q->p++ = mask;
}

static inline struct ps4_bridge *bridge_to_ps4_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ps4_bridge, bridge);
}

static void ps4_bridge_pre_enable(struct drm_bridge *bridge,
                                  struct drm_bridge_state *old_bridge_state)
{
	struct ps4_bridge *mn_bridge = bridge_to_ps4_bridge(bridge);
	dev_info(mn_bridge->dev, "ps4_bridge_pre_enable\n");
	dev_info(mn_bridge->dev, "Enable ps4_bridge_pre_enable\n");
	mutex_lock(&mn_bridge->mutex);
	cq_init(&mn_bridge->cq, 4);

#if 0
	/* No idea. DP stuff probably. This borks for some reason. Meh. */
	cq_writereg(&mn_bridge->cq, 0x7657,0xff);
	cq_writereg(&mn_bridge->cq, 0x76a5,0x80);
	cq_writereg(&mn_bridge->cq, 0x76a6,0x04);
	cq_writereg(&mn_bridge->cq, 0x7601,0x0a);
	cq_writereg(&mn_bridge->cq, 0x7602,0x84);
	cq_writereg(&mn_bridge->cq, 0x7603,0x00);
	cq_writereg(&mn_bridge->cq, 0x76a8,0x09);
	cq_writereg(&mn_bridge->cq, 0x76ae,0xd1);
	cq_writereg(&mn_bridge->cq, 0x76af,0x50);
	cq_writereg(&mn_bridge->cq, 0x76b0,0x70);
	cq_writereg(&mn_bridge->cq, 0x76b1,0xb0);
	cq_writereg(&mn_bridge->cq, 0x76b2,0xf0);
	cq_writereg(&mn_bridge->cq, 0x76db,0x00);
	cq_writereg(&mn_bridge->cq, 0x76dc,0x64);
	cq_writereg(&mn_bridge->cq, 0x76dd,0x22);
	cq_writereg(&mn_bridge->cq, 0x76e4,0x00);
	cq_writereg(&mn_bridge->cq, 0x76e6,0x1e); /* 0 for (DP?) scramble off */
	cq_writereg(&mn_bridge->cq, 0x7670,0xff);
	cq_writereg(&mn_bridge->cq, 0x7671,0xff);
	cq_writereg(&mn_bridge->cq, 0x7672,0xff);
	cq_writereg(&mn_bridge->cq, 0x7673,0xff);
	cq_writereg(&mn_bridge->cq, 0x7668,0xff);
	cq_writereg(&mn_bridge->cq, 0x7669,0xff);
	cq_writereg(&mn_bridge->cq, 0x766a,0xff);
	cq_writereg(&mn_bridge->cq, 0x766b,0xff);
	cq_writereg(&mn_bridge->cq, 0x7655,0x04);
	cq_writereg(&mn_bridge->cq, 0x7007,0xff);
	cq_writereg(&mn_bridge->cq, 0x7098,0xff);
	cq_writereg(&mn_bridge->cq, 0x7099,0x00);
	cq_writereg(&mn_bridge->cq, 0x709a,0x0f);
	cq_writereg(&mn_bridge->cq, 0x709b,0x00);
	cq_writereg(&mn_bridge->cq, 0x709c,0x50);
	cq_writereg(&mn_bridge->cq, 0x709d,0x00);
	cq_writereg(&mn_bridge->cq, 0x709e,0x00);
	cq_writereg(&mn_bridge->cq, 0x709f,0xd0);
	cq_writereg(&mn_bridge->cq, 0x7a9c,0x2e);
	cq_writereg(&mn_bridge->cq, 0x7021,0x04);
	cq_writereg(&mn_bridge->cq, 0x7028,0x00);
	cq_writereg(&mn_bridge->cq, 0x7030,0xa3);
	cq_writereg(&mn_bridge->cq, 0x7016,0x04);
#endif

	/* Disable InfoFrames */
	cq_writereg(&mn_bridge->cq, INFENA, 0x00);
	/* Reset HDCP */
	cq_writereg(&mn_bridge->cq, TSRST, TSRST_ENCSRST | TSRST_HDCPSRST);
	/* Disable HDCP flag */
	cq_writereg(&mn_bridge->cq, TSRST, HDCPEN_ENC_DIS);
	/* HDCP AKE reset */
	cq_writereg(&mn_bridge->cq, AKESRST, 0xff);
	/* Wait AKE busy */
	cq_wait_clear(&mn_bridge->cq, AKESTA, AKESTA_BUSY);

	if (cq_exec(&mn_bridge->cq) < 0) {
		dev_err(mn_bridge->dev, "failed to run pre-enable sequence");
	}
	mutex_unlock(&mn_bridge->mutex);
}

static void ps4_bridge_enable(struct drm_bridge *bridge,
                              struct drm_bridge_state *old_bridge_state)
{
	struct ps4_bridge *mn_bridge = bridge_to_ps4_bridge(bridge);
	struct drm_atomic_state *state = old_bridge_state->base.state;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct drm_display_mode *mode;

	u8 dp[3];

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	if (WARN_ON(!connector))
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		return;
	
	mode = &crtc_state->adjusted_mode;

	
	struct drm_device *dev = connector->dev;
	struct pci_dev *pdev = to_pci_dev(dev->dev);

	mn_bridge->mode = drm_match_cea_mode(mode);

	dev_info(mn_bridge->dev, "Enable PS4_BRIDGE_ENABLE\n");
	if (!mn_bridge->mode) {
		dev_err(mn_bridge->dev, "mode not available\n");
		return;
	}

	if (pdev->vendor != (PCI_VENDOR_ID_ATI || PCI_VENDOR_ID_AMD)) {
		dev_err(mn_bridge->dev, "Invalid vendor: %04x", pdev->vendor);
		return;
	}

	dev_info(mn_bridge->dev, "ps4_bridge_enable (mode: %d)\n", mn_bridge->mode);

	/* Here come the dragons */

	if(pdev->device == PCI_DEVICE_ID_CUH_11XX)
	{
		/* Panasonic MN86471A */
		mutex_lock(&mn_bridge->mutex);
		cq_init(&mn_bridge->cq, 4);

		/* Read DisplayPort status (?) */
		cq_read(&mn_bridge->cq, 0x76e1, 3);
		if (cq_exec(&mn_bridge->cq) < 11) {
			mutex_unlock(&mn_bridge->mutex);
			dev_err(mn_bridge->dev, "could not read DP status");
		return;
		}
		memcpy(dp, &mn_bridge->cq.reply.databuf[3], 3);

		cq_init(&mn_bridge->cq, 4);

		/* Wait for DP lane status */
		cq_wait_set(&mn_bridge->cq, 0x761e, 0x77);
		cq_wait_set(&mn_bridge->cq, 0x761f, 0x77);
		/* Wait for ?? */
		cq_wait_set(&mn_bridge->cq, 0x7669, 0x01);
		cq_writereg(&mn_bridge->cq, 0x76d9, (dp[0] & 0x1f) | (dp[0] << 5));
		cq_writereg(&mn_bridge->cq, 0x76da, (dp[1] & 0x7c) | ((dp[0] >> 3) & 3) | ((dp[1] << 5) & 0x80));
		cq_writereg(&mn_bridge->cq, 0x76db, 0x80 | ((dp[1] >> 3) & 0xf));
		cq_writereg(&mn_bridge->cq, 0x76e4, 0x01);
		cq_writereg(&mn_bridge->cq, TSYSCTRL, TSYSCTRL_HDMI);
		cq_writereg(&mn_bridge->cq, VINCNT, VINCNT_VIF_FILEN);
		cq_writereg(&mn_bridge->cq, 0x7071, 0);
		cq_writereg(&mn_bridge->cq, 0x7062, mn_bridge->mode);
		cq_writereg(&mn_bridge->cq, 0x765a, 0);
		cq_writereg(&mn_bridge->cq, 0x7062, mn_bridge->mode | 0x80);
		cq_writereg(&mn_bridge->cq, 0x7215, 0x28); /* aspect */
		cq_writereg(&mn_bridge->cq, 0x7217, mn_bridge->mode);
		cq_writereg(&mn_bridge->cq, 0x7218, 0);
		cq_writereg(&mn_bridge->cq, CSCMOD, 0xdc);
		cq_writereg(&mn_bridge->cq, C420SET, 0xaa);
		cq_writereg(&mn_bridge->cq, TDPCMODE, 0x4a);
		cq_writereg(&mn_bridge->cq, OUTWSET, 0x00);
		cq_writereg(&mn_bridge->cq, 0x70c4, 0x08);
		cq_writereg(&mn_bridge->cq, 0x70c5, 0x08);
		cq_writereg(&mn_bridge->cq, 0x7096, 0xff);
		cq_writereg(&mn_bridge->cq, 0x7027, 0x00);
		cq_writereg(&mn_bridge->cq, 0x7020, 0x20);
		cq_writereg(&mn_bridge->cq, 0x700b, 0x01);
		cq_writereg(&mn_bridge->cq, PKTENA, 0x20);
		cq_writereg(&mn_bridge->cq, 0x7096, 0xff);
		cq_writereg(&mn_bridge->cq, INFENA, INFENA_AVIEN);
		cq_writereg(&mn_bridge->cq, UPDCTRL, UPDCTRL_ALLUPD | UPDCTRL_AVIIUPD |
						     UPDCTRL_CLKUPD | UPDCTRL_VIFUPD |
						     UPDCTRL_CSCUPD);
		cq_wait_set(&mn_bridge->cq, 0x7096, 0x80);

		cq_mask(&mn_bridge->cq, 0x7216, 0x00, 0x80);
		cq_writereg(&mn_bridge->cq, 0x7218, 0x00);

		cq_writereg(&mn_bridge->cq, 0x7096, 0xff);
		cq_writereg(&mn_bridge->cq, VMUTECNT, VMUTECNT_LINEWIDTH_90 | VMUTECNT_VMUTE_MUTE_NORMAL);
		cq_writereg(&mn_bridge->cq, 0x7016, 0x04);
		cq_writereg(&mn_bridge->cq, 0x7a88, 0xff);
		cq_writereg(&mn_bridge->cq, 0x7a83, 0x88);
		cq_writereg(&mn_bridge->cq, 0x7204, 0x40);

		cq_wait_set(&mn_bridge->cq, 0x7096, 0x80);

		cq_writereg(&mn_bridge->cq, 0x7006, 0x02);
		cq_writereg(&mn_bridge->cq, 0x7020, 0x21);
		cq_writereg(&mn_bridge->cq, 0x7a8b, 0x00);
		cq_writereg(&mn_bridge->cq, 0x7020, 0x21);

		cq_writereg(&mn_bridge->cq, VMUTECNT, VMUTECNT_LINEWIDTH_90);
		if (cq_exec(&mn_bridge->cq) < 0) {
			dev_err(mn_bridge->dev, "Failed to configure ps4-bridge (MN86471A) mode\n");
		}
		#if 1
		// preinit
		cq_init(&mn_bridge->cq, 4);
		cq_writereg(&mn_bridge->cq,0x70b3, 0x00);
		cq_writereg(&mn_bridge->cq,0x70b7, 0x0b);
		cq_writereg(&mn_bridge->cq,0x70a8, 0x24);

		cq_mask(&mn_bridge->cq,0x70b9, 0x06, 0x06);
		cq_mask(&mn_bridge->cq,0x70b6, 0x02, 0x0f);
		cq_mask(&mn_bridge->cq,0x70ba, 0x40, 0x70);
		cq_mask(&mn_bridge->cq,0x70b2, 0x20, 0xe0);
		cq_mask(&mn_bridge->cq,0x7257, 0x00, 0xff);
		cq_mask(&mn_bridge->cq,0x70b0, 0x01, 0x21);
		cq_mask(&mn_bridge->cq,0x70ba, 0x00, 0x88);
		cq_mask(&mn_bridge->cq,0x70b9, 0x01, 0x01);
		if (cq_exec(&mn_bridge->cq) < 0) {
			dev_err(mn_bridge->dev, "failed to run enable MN86471A hdmi audio seq. 0");
		}

		cq_init(&mn_bridge->cq, 4);
		cq_writereg(&mn_bridge->cq,0x7ed8, 0x01);

		cq_mask(&mn_bridge->cq,0x70b4, 0x00, 0x3e);
		cq_mask(&mn_bridge->cq,0x70b5, 0x79, 0xff);
		cq_mask(&mn_bridge->cq,0x70ab, 0x00, 0xff);
		cq_mask(&mn_bridge->cq,0x70b6, 0x02, 0x3f);
		cq_mask(&mn_bridge->cq,0x70b7, 0x0b, 0x0f);
		cq_mask(&mn_bridge->cq,0x70ac, 0x00, 0xff);
		cq_mask(&mn_bridge->cq,0x70bd, 0x00, 0xff);

		cq_writereg(&mn_bridge->cq, 0x7204, 0x10);
		cq_writereg(&mn_bridge->cq,0x7011, 0xa2);

		cq_wait_set(&mn_bridge->cq,0x7096, 0x80);
		cq_writereg(&mn_bridge->cq,0x7096, 0xff);

		cq_mask(&mn_bridge->cq,0x7203, 0x10, 0x10);
		cq_writereg(&mn_bridge->cq,0x70b1, 0xc0);
		if (cq_exec(&mn_bridge->cq) < 0) {
			dev_err(mn_bridge->dev, "failed to run enable hdmi MN86471A audio seq. 1");
		}
		#endif
		mutex_unlock(&mn_bridge->mutex);
	}
	else
	{
		/* Panasonic MN864729 */
		mutex_lock(&mn_bridge->mutex);
		cq_init(&mn_bridge->cq, 4);
		cq_mask(&mn_bridge->cq, 0x6005, 0x01, 0x01);
		cq_writereg(&mn_bridge->cq, 0x6a03, 0x47);

		/* Wait for DP lane status */
		cq_wait_set(&mn_bridge->cq, 0x60f8, 0xff);
		cq_wait_set(&mn_bridge->cq, 0x60f9, 0x01);
		cq_writereg(&mn_bridge->cq, 0x6a01, 0x4d);
		cq_wait_set(&mn_bridge->cq, 0x60f9, 0x1a);

		cq_mask(&mn_bridge->cq, 0x1e00, 0x00, 0x21);
		cq_mask(&mn_bridge->cq, 0x1e02, 0x00, 0x70);
		// 03 08 01 01 00  2c 01 00
		cq_delay(&mn_bridge->cq, 0x012c);
		cq_writereg(&mn_bridge->cq, 0x6020, 0x00);
		cq_delay(&mn_bridge->cq, 0x0032);
		cq_writereg(&mn_bridge->cq, 0x7402, 0x1c);
		cq_writereg(&mn_bridge->cq, 0x6020, 0x04);
		cq_writereg(&mn_bridge->cq, TSYSCTRL, TSYSCTRL_HDMI);
		cq_writereg(&mn_bridge->cq, 0x10c7, 0x38);
		cq_writereg(&mn_bridge->cq, 0x1e02, 0x88);
		cq_writereg(&mn_bridge->cq, 0x1e00, 0x66);
		cq_writereg(&mn_bridge->cq, 0x100c, 0x01);
		cq_writereg(&mn_bridge->cq, TSYSCTRL, TSYSCTRL_HDMI);

		cq_writereg(&mn_bridge->cq, 0x7009, 0x00);
		cq_writereg(&mn_bridge->cq, 0x7040, 0x42);
		cq_writereg(&mn_bridge->cq, 0x7225, 0x28);
		cq_writereg(&mn_bridge->cq, 0x7227, mn_bridge->mode);
		cq_writereg(&mn_bridge->cq, 0x7228, 0x00);
		cq_writereg(&mn_bridge->cq, 0x7070, mn_bridge->mode);
		cq_writereg(&mn_bridge->cq, 0x7071, mn_bridge->mode | 0x80);
		cq_writereg(&mn_bridge->cq, 0x7072, 0x00);
		cq_writereg(&mn_bridge->cq, 0x7073, 0x00);
		cq_writereg(&mn_bridge->cq, 0x7074, 0x00);
		cq_writereg(&mn_bridge->cq, 0x7075, 0x00);
		cq_writereg(&mn_bridge->cq, 0x70c4, 0x0a);
		cq_writereg(&mn_bridge->cq, 0x70c5, 0x0a);
		cq_writereg(&mn_bridge->cq, 0x70c2, 0x00);
		cq_writereg(&mn_bridge->cq, 0x70fe, 0x12);
		cq_writereg(&mn_bridge->cq, 0x70c3, 0x10);

		if(pdev->device == PCI_DEVICE_ID_CUH_12XX) {
			/* newer ps4 phats need here 0x03 idk why. */
			cq_writereg(&mn_bridge->cq, 0x10c5, 0x03);
		} else {
			cq_writereg(&mn_bridge->cq, 0x10c5, 0x00);
		}

		cq_writereg(&mn_bridge->cq, 0x10f6, 0xff);
		cq_writereg(&mn_bridge->cq, 0x7202, 0x20);
		cq_writereg(&mn_bridge->cq, 0x7203, 0x60);
		cq_writereg(&mn_bridge->cq, 0x7011, 0xd5);
		//cq_writereg(&mn_bridge->cq, 0x7a00, 0x0e);

		cq_wait_set(&mn_bridge->cq, 0x10f6, 0x80);
		cq_mask(&mn_bridge->cq, 0x7226, 0x00, 0x80);
		cq_mask(&mn_bridge->cq, 0x7228, 0x00, 0xFF);
		cq_delay(&mn_bridge->cq, 0x012c);
		cq_writereg(&mn_bridge->cq, 0x7204, 0x40);
		cq_wait_clear(&mn_bridge->cq, 0x7204, 0x40);
		cq_writereg(&mn_bridge->cq, 0x7a8b, 0x05);
		cq_mask(&mn_bridge->cq, 0x1e02, 0x70, 0x70);
		cq_mask(&mn_bridge->cq, 0x1034, 0x02, 0x02);
		cq_mask(&mn_bridge->cq, 0x1e00, 0x01, 0x01);
		cq_writereg(&mn_bridge->cq, VMUTECNT, VMUTECNT_LINEWIDTH_90);
		cq_writereg(&mn_bridge->cq, HDCPEN, 0x00);
		if (cq_exec(&mn_bridge->cq) < 0) {
			dev_err(mn_bridge->dev, "Failed to configure ps4-bridge (MN864729) mode\n");
		}
		#if 1
		// AUDIO preinit
		cq_init(&mn_bridge->cq, 4);
		cq_writereg(&mn_bridge->cq,0x70aa, 0x00);
		cq_writereg(&mn_bridge->cq,0x70af, 0x07);
		cq_writereg(&mn_bridge->cq,0x70a9, 0x5a);

		cq_mask(&mn_bridge->cq,0x70af, 0x06, 0x06);
		cq_mask(&mn_bridge->cq,0x70af, 0x02, 0x0f);
		cq_mask(&mn_bridge->cq,0x70b3, 0x02, 0x0f);
		cq_mask(&mn_bridge->cq,0x70ae, 0x80, 0xe0);
		cq_mask(&mn_bridge->cq,0x70ae, 0x01, 0x07);
		cq_mask(&mn_bridge->cq,0x70ac, 0x01, 0x21);
		cq_mask(&mn_bridge->cq,0x70ab, 0x80, 0x88);
		cq_mask(&mn_bridge->cq,0x70a9, 0x01, 0x01);
		if (cq_exec(&mn_bridge->cq) < 0) {
				dev_err(mn_bridge->dev, "failed to run enable hdmi audio seq. 0");
		}

		cq_init(&mn_bridge->cq, 4);
		cq_writereg(&mn_bridge->cq,0x70b0, 0x01);
		cq_mask(&mn_bridge->cq,0x70b0, 0x00, 0xff);
		cq_mask(&mn_bridge->cq,0x70b1, 0x79, 0xff);
		cq_mask(&mn_bridge->cq,0x70b2, 0x00, 0xff);
		cq_mask(&mn_bridge->cq,0x70b3, 0x02, 0xff);
		cq_mask(&mn_bridge->cq,0x70b4, 0x0b, 0x0f);
		cq_mask(&mn_bridge->cq,0x70b5, 0x00, 0xff);
		cq_mask(&mn_bridge->cq,0x70b6, 0x00, 0xff);
		cq_writereg(&mn_bridge->cq,0x10f6, 0xff);
		cq_writereg(&mn_bridge->cq,0x7011, 0xa2);
		cq_wait_set(&mn_bridge->cq,0x10f6, 0xa2);
		cq_mask(&mn_bridge->cq,0x7267, 0x00, 0xff);
		cq_writereg(&mn_bridge->cq,0x7204, 0x10);
		cq_wait_clear(&mn_bridge->cq,0x7204, 0x10);
		cq_writereg(&mn_bridge->cq,0x10f6, 0xff);
		cq_mask(&mn_bridge->cq,0x7203, 0x10, 0x10);
		cq_writereg(&mn_bridge->cq,0x70a8, 0xc0);
		if (cq_exec(&mn_bridge->cq) < 0) {
				dev_err(mn_bridge->dev, "failed to run enable hdmi audio seq. 1");
		}
		#endif
		mutex_unlock(&mn_bridge->mutex);
	}

}

static void ps4_bridge_disable(struct drm_bridge *bridge,
                               struct drm_bridge_state *old_bridge_state)
{
	struct ps4_bridge *mn_bridge = bridge_to_ps4_bridge(bridge);
	dev_info(mn_bridge->dev, "ps4_bridge_disable\n");

	mutex_lock(&mn_bridge->mutex);
	cq_init(&mn_bridge->cq, 4);
	cq_writereg(&mn_bridge->cq, VMUTECNT, VMUTECNT_LINEWIDTH_90 | VMUTECNT_VMUTE_MUTE_NORMAL);
	cq_writereg(&mn_bridge->cq, INFENA, 0x00);
	if (cq_exec(&mn_bridge->cq) < 0) {
		dev_err(mn_bridge->dev, "Failed to disable bridge\n");
	}
	mutex_unlock(&mn_bridge->mutex);
}

static void ps4_bridge_post_disable(struct drm_bridge *bridge,
                                    struct drm_bridge_state *old_bridge_state)
{
	struct ps4_bridge *mn_bridge = bridge_to_ps4_bridge(bridge);

	dev_info(mn_bridge->dev, "ps4_bridge_post_disable\n");
}

enum drm_connector_status ps4_bridge_detect(struct drm_bridge *bridge)
{
	struct ps4_bridge *mn_bridge = bridge_to_ps4_bridge(bridge);
	u8 reg;

	mutex_lock(&mn_bridge->mutex);
	cq_init(&mn_bridge->cq, 4);
	cq_read(&mn_bridge->cq, TMONREG, 1);
	if (cq_exec(&mn_bridge->cq) < 9) {
		mutex_unlock(&mn_bridge->mutex);
		dev_err(mn_bridge->dev, "could not read TMONREG");
		return connector_status_disconnected;
	}
	reg = mn_bridge->cq.reply.databuf[3];
	mutex_unlock(&mn_bridge->mutex);

	dev_info(mn_bridge->dev, "TMONREG=0x%02x\n", reg);

	if (reg & TMONREG_HPD)
		return connector_status_connected;
	else
		return connector_status_disconnected;
}

static enum drm_mode_status ps4_bridge_mode_valid(struct drm_bridge *bridge,
					const struct drm_display_info *info,
					const struct drm_display_mode *mode)
{
	int vic = drm_match_cea_mode(mode);

	/* Allow anything that we can match up to a VIC (CEA modes) */
	if (!vic || (vic != 16 && vic != 4 && vic != 63)) {
		return MODE_BAD;
	}

	return MODE_OK;
}

static int ps4_bridge_attach(struct drm_bridge *bridge,
			     enum drm_bridge_attach_flags flags)
{
	struct ps4_bridge *mn_bridge = bridge_to_ps4_bridge(bridge);

	return drm_bridge_attach(bridge->encoder, mn_bridge->next_bridge,
                             NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);
}

static struct drm_bridge_funcs ps4_bridge_funcs = {
	.attach = ps4_bridge_attach,
    .mode_valid = ps4_bridge_mode_valid,
    .detect = ps4_bridge_detect,

    .atomic_pre_enable = ps4_bridge_pre_enable,
    .atomic_enable = ps4_bridge_enable,
    .atomic_disable = ps4_bridge_disable,
    .atomic_post_disable = ps4_bridge_post_disable,
};

static int ps4_bridge_probe(struct platform_device *pdev) 
{
	struct ps4_bridge *mn_bridge;
	struct device *dev = &pdev->dev;

	if (!dev->of_node)
		return -ENODEV;

	mn_bridge = devm_kzalloc(&pdev->dev, sizeof(*mn_bridge), GFP_KERNEL);
	if (!mn_bridge)
		return -ENOMEM;

	mn_bridge->dev = dev;

	mn_bridge->bridge.funcs = &ps4_bridge_funcs;
	mn_bridge->bridge.of_node = dev->of_node;
	mn_bridge->bridge.type = DRM_MODE_CONNECTOR_HDMIA;

	drm_bridge_add(&mn_bridge->bridge);

	return 0;
}

static struct platform_driver ps4_bridge_driver = {
	.probe = ps4_bridge_probe,
	.driver = {
		   .name = "ps4 bridge",
	},
};
module_platform_driver(ps4_bridge_driver);

MODULE_DESCRIPTION("Panasonic MN8647XX DP to HDMI bridge driver");
MODULE_LICENSE("GPL v2");
