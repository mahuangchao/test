// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * X-Box gamepad driver
 *
 * Copyright (c) 2002 Marko Friedemann <mfr@bmx-chemnitz.de>
 *               2004 Oliver Schwartz <Oliver.Schwartz@gmx.de>,
 *                    Steven Toth <steve@toth.demon.co.uk>,
 *                    Franz Lehner <franz@caos.at>,
 *                    Ivan Hawkes <blackhawk@ivanhawkes.com>
 *               2005 Dominic Cerquetti <binary1230@yahoo.com>
 *               2006 Adam Buchbinder <adam.buchbinder@gmail.com>
 *               2007 Jan Kratochvil <honza@jikos.cz>
 *               2010 Christoph Fritz <chf.fritz@googlemail.com>
 *
 * This driver is based on:
 *  - information from     http://euc.jp/periphs/xbox-controller.ja.html
 *  - the iForce driver    drivers/char/joystick/iforce.c
 *  - the skeleton-driver  drivers/usb/usb-skeleton.c
 *  - Xbox 360 information http://www.free60.org/wiki/Gamepad
 *  - Xbox One information https://github.com/quantus/xbox-one-controller-protocol
 *
 * Thanks to:
 *  - ITO Takayuki for providing essential zhixu_xpad information on his website
 *  - Vojtech Pavlik     - iforce driver / input subsystem
 *  - Greg Kroah-Hartman - usb-skeleton driver
 *  - XBOX Linux project - extra USB id's
 *  - Pekka Pöyry (quantus) - Xbox One controller reverse engineering
 *
 * TODO:
 *  - fine tune axes (especially trigger axes)
 *  - fix "analog" buttons (reported as digital now)
 *  - get rumble working
 *  - need USB IDs for other dance pads
 *
 * History:
 *
 * 2002-06-27 - 0.0.1 : first version, just said "XBOX HID controller"
 *
 * 2002-07-02 - 0.0.2 : basic working version
 *  - all axes and 9 of the 10 buttons work (german InterAct device)
 *  - the black button does not work
 *
 * 2002-07-14 - 0.0.3 : rework by Vojtech Pavlik
 *  - indentation fixes
 *  - usb + input init sequence fixes
 *
 * 2002-07-16 - 0.0.4 : minor changes, merge with Vojtech's v0.0.3
 *  - verified the lack of HID and report descriptors
 *  - verified that ALL buttons WORK
 *  - fixed d-pad to axes mapping
 *
 * 2002-07-17 - 0.0.5 : simplified d-pad handling
 *
 * 2004-10-02 - 0.0.6 : DDR pad support
 *  - borrowed from the XBOX linux kernel
 *  - USB id's for commonly used dance pads are present
 *  - dance pads will map D-PAD to buttons, not axes
 *  - pass the module paramater 'dpad_to_buttons' to force
 *    the D-PAD to map to buttons if your pad is not detected
 *
 * Later changes can be tracked in SCM.
 */

#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/usb/input.h>
#include <linux/usb/quirks.h>
#include <uapi/linux/sched/types.h>
#include <linux/kthread.h>

#define XPAD_PKT_LEN 64

/*
 * xbox d-pads should map to buttons, as is required for DDR pads
 * but we map them to axes when possible to simplify things
 */
#define MAP_DPAD_TO_BUTTONS		(1 << 0)
#define MAP_TRIGGERS_TO_BUTTONS		(1 << 1)
#define MAP_STICKS_TO_NULL		(1 << 2)
#define DANCEPAD_MAP_CONFIG	(MAP_DPAD_TO_BUTTONS |			\
				MAP_TRIGGERS_TO_BUTTONS | MAP_STICKS_TO_NULL)

#define XTYPE_XBOX        0
#define XTYPE_XBOX360     1
#define XTYPE_XBOX360W    2
#define XTYPE_XBOXONE     3
#define XTYPE_UNKNOWN     4

//extern void abxylute_share_input_f10(int value);
//extern void share_set_cont(u8 cont_wait_num_left,u8 cont_wait_num_right);
extern void share_set_vibrator_level_left(u8 level);
extern void share_set_vibrator_level_right(u8 level);
//static struct work_struct vibrator_wq;
static DECLARE_WAIT_QUEUE_HEAD(wait_left);
static DECLARE_WAIT_QUEUE_HEAD(wait_right);
//static int tpd_flag; 
	
static bool dpad_to_buttons;
module_param(dpad_to_buttons, bool, S_IRUGO);
MODULE_PARM_DESC(dpad_to_buttons, "Map D-PAD to buttons rather than axes for unknown pads");

static bool triggers_to_buttons;
module_param(triggers_to_buttons, bool, S_IRUGO);
MODULE_PARM_DESC(triggers_to_buttons, "Map triggers to buttons rather than axes for unknown pads");

static bool sticks_to_null;
module_param(sticks_to_null, bool, S_IRUGO);
MODULE_PARM_DESC(sticks_to_null, "Do not map sticks at all for unknown pads");

static bool auto_poweroff = true;
module_param(auto_poweroff, bool, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(auto_poweroff, "Power off wireless controllers on suspend");

static const struct zhixu_xpad_device {
	u16 idVendor;
	u16 idProduct;
	char *name;
	u8 mapping;
	u8 xtype;
} zhixu_xpad_device[] = { 
	{ 0x045f, 0x029f, "Microsoft X-Box 360 pad", 0, XTYPE_XBOX360 }, //Ysjie modify 
	//{ 0x045e, 0x028e, "Microsoft X-Box 360 pad", 0, XTYPE_XBOX360 }, //Ysjie modify  
	{ 0xffff, 0xffff, "Chinese-made Xbox Controller", 0, XTYPE_XBOX },
	{ 0x0000, 0x0000, "Generic X-Box pad", 0, XTYPE_UNKNOWN }

};

/* buttons shared with xbox and xbox360 */
static const signed short zhixu_xpad_common_btn[] = {
	BTN_A, BTN_B, BTN_X, BTN_Y,			/* "analog" buttons */
	BTN_START, BTN_SELECT, BTN_THUMBL, BTN_THUMBR,	/* start/back/sticks */
	-1						/* terminating entry */
};

/* original xbox controllers only */
static const signed short zhixu_xpad_btn[] = {
	BTN_C, BTN_Z,		/* "analog" buttons */
	-1			/* terminating entry */
};

/* used when dpad is mapped to buttons */
static const signed short zhixu_xpad_btn_pad[] = {
	BTN_TRIGGER_HAPPY1, BTN_TRIGGER_HAPPY2,		/* d-pad left, right */
	BTN_TRIGGER_HAPPY3, BTN_TRIGGER_HAPPY4,		/* d-pad up, down */
	-1				/* terminating entry */
};

/* used when triggers are mapped to buttons */
static const signed short zhixu_xpad_btn_triggers[] = {
	BTN_TL2, BTN_TR2,		/* triggers left/right */
	-1
};

static const signed short zhixu_xpad360_btn[] = {  /* buttons for x360 controller */
	BTN_TL, BTN_TR,		/* Button LB/RB */
	BTN_MODE,		/* The big X button */
	KEY_F10,
	-1
};

static const signed short zhixu_xpad_abs[] = {
	ABS_X, ABS_Y,		/* left stick */
	ABS_RX, ABS_RY,		/* right stick */
	-1			/* terminating entry */
};

/* used when dpad is mapped to axes */
static const signed short zhixu_xpad_abs_pad[] = {
	ABS_HAT0X, ABS_HAT0Y,	/* d-pad axes */
	-1			/* terminating entry */
};

/* used when triggers are mapped to axes */
static const signed short zhixu_xpad_abs_triggers[] = {
	ABS_Z, ABS_RZ,		/* triggers left/right */
	-1
};

/*
 * Xbox 360 has a vendor-specific class, so we cannot match it with only
 * USB_INTERFACE_INFO (also specifically refused by USB subsystem), so we
 * match against vendor id as well. Wired Xbox 360 devices have protocol 1,
 * wireless controllers have protocol 129.
 */
#define XPAD_XBOX360_VENDOR_PROTOCOL(vend, pr) \
	.match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO, \
	.idVendor = (vend), \
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
	.bInterfaceSubClass = 93, \
	.bInterfaceProtocol = (pr)
#define XPAD_XBOX360_VENDOR(vend) \
	{ XPAD_XBOX360_VENDOR_PROTOCOL((vend), 1) }, \
	{ XPAD_XBOX360_VENDOR_PROTOCOL((vend), 129) }

/* The Xbox One controller uses subclass 71 and protocol 208. */
#define XPAD_XBOXONE_VENDOR_PROTOCOL(vend, pr) \
	.match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO, \
	.idVendor = (vend), \
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
	.bInterfaceSubClass = 71, \
	.bInterfaceProtocol = (pr)
#define XPAD_XBOXONE_VENDOR(vend) \
	{ XPAD_XBOXONE_VENDOR_PROTOCOL((vend), 208) }

static const struct usb_device_id zhixu_xpad_table[] = {
	{ USB_INTERFACE_INFO('X', 'B', 0) },	/* X-Box USB-IF not approved class */ 
	XPAD_XBOX360_VENDOR(0x045f),		/* Microsoft X-Box 360 controllers */  //Ysjie modify 
	//XPAD_XBOX360_VENDOR(0x045e),		/* Microsoft X-Box 360 controllers */  //Ysjie modify 
	{ }
};

MODULE_DEVICE_TABLE(usb, zhixu_xpad_table);

struct xboxone_init_packet {
	u16 idVendor;
	u16 idProduct;
	const u8 *data;
	u8 len;
};

#define XBOXONE_INIT_PKT(_vid, _pid, _data)		\
	{						\
		.idVendor	= (_vid),		\
		.idProduct	= (_pid),		\
		.data		= (_data),		\
		.len		= ARRAY_SIZE(_data),	\
	}


/*
 * This packet is required for all Xbox One pads with 2015
 * or later firmware installed (or present from the factory).
 */
static const u8 xboxone_fw2015_init[] = {
	0x05, 0x20, 0x00, 0x01, 0x00
};

/*
 * This packet is required for Xbox One S (0x045e:0x02ea)
 * and Xbox One Elite Series 2 (0x045e:0x0b00) pads to
 * initialize the controller that was previously used in
 * Bluetooth mode.
 */
static const u8 xboxone_s_init[] = {
	0x05, 0x20, 0x00, 0x0f, 0x06
};

/*
 * This packet is required for the Titanfall 2 Xbox One pads
 * (0x0e6f:0x0165) to finish initialization and for Hori pads
 * (0x0f0d:0x0067) to make the analog sticks work.
 */
static const u8 xboxone_hori_init[] = {
	0x01, 0x20, 0x00, 0x09, 0x00, 0x04, 0x20, 0x3a,
	0x00, 0x00, 0x00, 0x80, 0x00
};

/*
 * This packet is required for most (all?) of the PDP pads to start
 * sending input reports. These pads include: (0x0e6f:0x02ab),
 * (0x0e6f:0x02a4), (0x0e6f:0x02a6).
 */
static const u8 xboxone_pdp_init1[] = {
	0x0a, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14
};

/*
 * This packet is required for most (all?) of the PDP pads to start
 * sending input reports. These pads include: (0x0e6f:0x02ab),
 * (0x0e6f:0x02a4), (0x0e6f:0x02a6).
 */
static const u8 xboxone_pdp_init2[] = {
	0x06, 0x20, 0x00, 0x02, 0x01, 0x00
};

/*
 * A specific rumble packet is required for some PowerA pads to start
 * sending input reports. One of those pads is (0x24c6:0x543a).
 */
static const u8 xboxone_rumblebegin_init[] = {
	0x09, 0x00, 0x00, 0x09, 0x00, 0x0F, 0x00, 0x00,
	0x1D, 0x1D, 0xFF, 0x00, 0x00
};

/*
 * A rumble packet with zero FF intensity will immediately
 * terminate the rumbling required to init PowerA pads.
 * This should happen fast enough that the motors don't
 * spin up to enough speed to actually vibrate the gamepad.
 */
static const u8 xboxone_rumbleend_init[] = {
	0x09, 0x00, 0x00, 0x09, 0x00, 0x0F, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00
};

/*
 * This specifies the selection of init packets that a gamepad
 * will be sent on init *and* the order in which they will be
 * sent. The correct sequence number will be added when the
 * packet is going to be sent.
 */
static const struct xboxone_init_packet xboxone_init_packets[] = {
	XBOXONE_INIT_PKT(0x0e6f, 0x0165, xboxone_hori_init),
	XBOXONE_INIT_PKT(0x0f0d, 0x0067, xboxone_hori_init),
	XBOXONE_INIT_PKT(0x0000, 0x0000, xboxone_fw2015_init),
	XBOXONE_INIT_PKT(0x045e, 0x02ea, xboxone_s_init),
	XBOXONE_INIT_PKT(0x045e, 0x0b00, xboxone_s_init),
	XBOXONE_INIT_PKT(0x0e6f, 0x0000, xboxone_pdp_init1),
	XBOXONE_INIT_PKT(0x0e6f, 0x0000, xboxone_pdp_init2),
	XBOXONE_INIT_PKT(0x24c6, 0x541a, xboxone_rumblebegin_init),
	XBOXONE_INIT_PKT(0x24c6, 0x542a, xboxone_rumblebegin_init),
	XBOXONE_INIT_PKT(0x24c6, 0x543a, xboxone_rumblebegin_init),
	XBOXONE_INIT_PKT(0x24c6, 0x541a, xboxone_rumbleend_init),
	XBOXONE_INIT_PKT(0x24c6, 0x542a, xboxone_rumbleend_init),
	XBOXONE_INIT_PKT(0x24c6, 0x543a, xboxone_rumbleend_init),
};

struct zhixu_xpad_output_packet {
	u8 data[XPAD_PKT_LEN];
	u8 len;
	bool pending;
};

#define XPAD_OUT_CMD_IDX	0
#define XPAD_OUT_FF_IDX		1
#define XPAD_OUT_LED_IDX	(1 + IS_ENABLED(CONFIG_JOYSTICK_XPAD_FF))
#define XPAD_NUM_OUT_PACKETS	(1 + \
				 IS_ENABLED(CONFIG_JOYSTICK_XPAD_FF) + \
				 IS_ENABLED(CONFIG_JOYSTICK_XPAD_LEDS))

struct usb_zhixu_xpad {
	struct input_dev *dev;		/* input device interface */
	struct input_dev __rcu *x360w_dev;
	struct usb_device *udev;	/* usb device */
	struct usb_interface *intf;	/* usb interface */

	bool pad_present;
	bool input_created;

	struct urb *irq_in;		/* urb for interrupt in report */
	unsigned char *idata;		/* input data */
	dma_addr_t idata_dma;

	struct urb *irq_out;		/* urb for interrupt out report */
	struct usb_anchor irq_out_anchor;
	bool irq_out_active;		/* we must not use an active URB */
	u8 odata_serial;		/* serial number for xbox one protocol */
	unsigned char *odata;		/* output data */
	dma_addr_t odata_dma;
	spinlock_t odata_lock;

	struct zhixu_xpad_output_packet out_packets[XPAD_NUM_OUT_PACKETS];
	int last_out_packet;
	int init_seq;

#if defined(CONFIG_JOYSTICK_XPAD_LEDS)
	struct zhixu_xpad_led *led;
#endif

	char phys[64];			/* physical device path */

	int mapping;			/* map d-pad to buttons or to axes */
	int xtype;			/* type of xbox device */
	int pad_nr;			/* the order x360 pads were attached */
	const char *name;		/* name of the device */
	struct work_struct work;	/* init/remove device from callback */
	
	/*Ysjie modify*/
	//struct work_struct vibrator_left_work;
	//struct work_struct vibrator_right_work;
	//struct workqueue_struct *vibrator_left_workqueue;
	//struct workqueue_struct *vibrator_right_workqueue;
	struct task_struct *thread_vibrator_left;
	struct task_struct *thread_vibrator_right;
	atomic_t work_left_running;
	atomic_t work_right_running;
	u8 vibrator_level_left;
	u8 vibrator_level_right;
	//wait_queue_head_t wait_left;
	//wait_queue_head_t wait_right;
};

static int zhixu_xpad_init_input(struct usb_zhixu_xpad *zhixu_xpad);
static void zhixu_xpad_deinit_input(struct usb_zhixu_xpad *zhixu_xpad);
static void zhixu_xpadone_ack_mode_report(struct usb_zhixu_xpad *zhixu_xpad, u8 seq_num);

/*
 *	zhixu_xpad_process_packet
 *
 *	Completes a request by converting the data into events for the
 *	input subsystem.
 *
 *	The used report descriptor was taken from ITO Takayukis website:
 *	 http://euc.jp/periphs/xbox-controller.ja.html
 */
static void zhixu_xpad_process_packet(struct usb_zhixu_xpad *zhixu_xpad, u16 cmd, unsigned char *data)
{
	struct input_dev *dev = zhixu_xpad->dev;

	if (!(zhixu_xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* left stick */
		input_report_abs(dev, ABS_X,
				 (__s16) le16_to_cpup((__le16 *)(data + 12)));
		input_report_abs(dev, ABS_Y,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 14)));

		/* right stick */
		input_report_abs(dev, ABS_RX,
				 (__s16) le16_to_cpup((__le16 *)(data + 16)));
		input_report_abs(dev, ABS_RY,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 18)));
	}

	/* triggers left/right */
	if (zhixu_xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		input_report_key(dev, BTN_TL2, data[10]);
		input_report_key(dev, BTN_TR2, data[11]);
	} else {
		input_report_abs(dev, ABS_Z, data[10]);
		input_report_abs(dev, ABS_RZ, data[11]);
	}

	/* digital pad */
	if (zhixu_xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		/* dpad as buttons (left, right, up, down) */
		input_report_key(dev, BTN_TRIGGER_HAPPY1, data[2] & 0x04);
		input_report_key(dev, BTN_TRIGGER_HAPPY2, data[2] & 0x08);
		input_report_key(dev, BTN_TRIGGER_HAPPY3, data[2] & 0x01);
		input_report_key(dev, BTN_TRIGGER_HAPPY4, data[2] & 0x02);
	} else {
		input_report_abs(dev, ABS_HAT0X,
				 !!(data[2] & 0x08) - !!(data[2] & 0x04));
		input_report_abs(dev, ABS_HAT0Y,
				 !!(data[2] & 0x02) - !!(data[2] & 0x01));
	}

	/* start/back buttons and stick press left/right */
	input_report_key(dev, BTN_START,  data[2] & 0x10);
	input_report_key(dev, BTN_SELECT, data[2] & 0x20);
	input_report_key(dev, BTN_THUMBL, data[2] & 0x40);
	input_report_key(dev, BTN_THUMBR, data[2] & 0x80);

	/* "analog" buttons A, B, X, Y */
	input_report_key(dev, BTN_A, data[4]);
	input_report_key(dev, BTN_B, data[5]);
	input_report_key(dev, BTN_X, data[6]);
	input_report_key(dev, BTN_Y, data[7]);

	/* "analog" buttons black, white */
	input_report_key(dev, BTN_C, data[8]);
	input_report_key(dev, BTN_Z, data[9]);

	input_sync(dev);
}

/*
 *	zhixu_xpad360_process_packet
 *
 *	Completes a request by converting the data into events for the
 *	input subsystem. It is version for xbox 360 controller
 *
 *	The used report descriptor was taken from:
 *		http://www.free60.org/wiki/Gamepad
 */

static void zhixu_xpad360_process_packet(struct usb_zhixu_xpad *zhixu_xpad, struct input_dev *dev,
				   u16 cmd, unsigned char *data)
{
	//int i = 0;
	/* valid pad data */
	if (data[0] != 0x00)
		return;
	
	//for(i = 0;i < 14;i++){
	//	pr_info("zhixu_xpad360_process_packet data[%d] = 0x%x\n",i,data[i]);
	//} 
	
	/* digital pad */
	if (zhixu_xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		/* dpad as buttons (left, right, up, down) */
		input_report_key(dev, BTN_TRIGGER_HAPPY1, data[2] & 0x04);
		input_report_key(dev, BTN_TRIGGER_HAPPY2, data[2] & 0x08);
		input_report_key(dev, BTN_TRIGGER_HAPPY3, data[2] & 0x01);
		input_report_key(dev, BTN_TRIGGER_HAPPY4, data[2] & 0x02);
	}

	/*
	 * This should be a simple else block. However historically
	 * xbox360w has mapped DPAD to buttons while xbox360 did not. This
	 * made no sense, but now we can not just switch back and have to
	 * support both behaviors.
	 */
	if (!(zhixu_xpad->mapping & MAP_DPAD_TO_BUTTONS) ||
	    zhixu_xpad->xtype == XTYPE_XBOX360W) {
		input_report_abs(dev, ABS_HAT0X,
				 !!(data[2] & 0x08) - !!(data[2] & 0x04));
		input_report_abs(dev, ABS_HAT0Y,
				 !!(data[2] & 0x02) - !!(data[2] & 0x01));
	}

	/* start/back buttons */
	input_report_key(dev, BTN_START,  data[2] & 0x10);
	input_report_key(dev, BTN_SELECT, data[2] & 0x20);

	/* stick press left/right */
	input_report_key(dev, BTN_THUMBL, data[2] & 0x40);
	input_report_key(dev, BTN_THUMBR, data[2] & 0x80);

	/* buttons A,B,X,Y,TL,TR and MODE */
	input_report_key(dev, BTN_A,	data[3] & 0x10);
	input_report_key(dev, BTN_B,	data[3] & 0x20);
	input_report_key(dev, BTN_X,	data[3] & 0x40);
	input_report_key(dev, BTN_Y,	data[3] & 0x80);
	input_report_key(dev, BTN_TL,	data[3] & 0x01);
	input_report_key(dev, BTN_TR,	data[3] & 0x02);  
	input_report_key(dev, BTN_MODE,	data[3] & 0x04);
	 
    
	input_report_key(dev, KEY_F10,	data[14] & 0x10);
#if 0    
	if(data[14] & 0x10) {
        vibrator_level_left = 0xff;
        vibrator_level_right = 0xff;
        queue_work(vibrator_workqueue, &vibrator_work); 
    }else {
        vibrator_level_left = 0x00;
        vibrator_level_right = 0x00;
        queue_work(vibrator_workqueue, &vibrator_work); 
    }
#endif    
     
	if (!(zhixu_xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* left stick */
		input_report_abs(dev, ABS_X,
				 (__s16) le16_to_cpup((__le16 *)(data + 6)));
		input_report_abs(dev, ABS_Y,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 8)));

		/* right stick */
		input_report_abs(dev, ABS_RX,
				 (__s16) le16_to_cpup((__le16 *)(data + 10)));
		input_report_abs(dev, ABS_RY,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 12)));
	}

	/* triggers left/right */
	if (zhixu_xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		input_report_key(dev, BTN_TL2, data[4]);
		input_report_key(dev, BTN_TR2, data[5]);
	} else {
		input_report_abs(dev, ABS_Z, data[4]);
		input_report_abs(dev, ABS_RZ, data[5]);
	}

	input_sync(dev);
}

#if 0
static int max3420_thread(void *dev_id)
{
	struct max3420_udc *udc = dev_id;
	struct spi_device *spi = udc->spi;
	int i, loop_again = 1;
	unsigned long flags;

	while (!kthread_should_stop()) {
		if (!loop_again) {
			ktime_t kt = ns_to_ktime(1000 * 1000 * 250); /* 250ms */

			set_current_state(TASK_INTERRUPTIBLE);

			spin_lock_irqsave(&udc->lock, flags);
			if (udc->todo & ENABLE_IRQ) {
				enable_irq(spi->irq);
				udc->todo &= ~ENABLE_IRQ;
			}
			spin_unlock_irqrestore(&udc->lock, flags);

			schedule_hrtimeout(&kt, HRTIMER_MODE_REL);
		}
		loop_again = 0;

		mutex_lock(&udc->spi_bus_mutex);

		/* If bus-vbus_active and disconnected */
		if (!udc->vbus_active || !udc->softconnect)
			goto loop;

		if (max3420_start(udc)) {
			loop_again = 1;
			goto loop;
		}

		if (max3420_handle_irqs(udc)) {
			loop_again = 1;
			goto loop;
		}

		if (spi_max3420_rwkup(udc)) {
			loop_again = 1;
			goto loop;
		}

		max3420_do_data(udc, 0, 1); /* get done with the EP0 ZLP */

		for (i = 1; i < MAX3420_MAX_EPS; i++) {
			struct max3420_ep *ep = &udc->ep[i];

			if (spi_max3420_enable(ep))
				loop_again = 1;
			if (spi_max3420_stall(ep))
				loop_again = 1;
		}
loop:
		mutex_unlock(&udc->spi_bus_mutex);
	}

	set_current_state(TASK_RUNNING);
	dev_info(udc->dev, "SPI thread exiting\n");
	return 0;
}
#endif


//void fts_irq_handler(u8 value)
//{
//	tpd_flag = 1;
//	temp_value = value;
//	wake_up_interruptible(&waiter);
//	//return 0;
//}
//EXPORT_SYMBOL(fts_irq_handler); 

static void wakeup_vibrator_thread(struct usb_zhixu_xpad *zhixu_xpad)
{
	atomic_set(&zhixu_xpad->work_left_running, 1);
	atomic_set(&zhixu_xpad->work_right_running, 1);
	wake_up_interruptible(&wait_left);
	wake_up_interruptible(&wait_right);
}

static int vibrator_left_thread(void *data)
{
	struct usb_zhixu_xpad *zhixu_xpad = data;//(struct usb_zhixu_xpad *) data;
	//struct sched_param param = {.sched_priority = RTPM_PRIO_TPD};
	struct sched_param param = {.sched_priority = 4};

	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(wait_left, atomic_read(&zhixu_xpad->work_left_running) == 1);

		atomic_set(&zhixu_xpad->work_left_running, 0);
		set_current_state(TASK_RUNNING);
		
		//zhixu_xpad->vibrator_level_left = temp_value;
		share_set_vibrator_level_left(zhixu_xpad->vibrator_level_left);
		pr_info("%s %d\n",__func__,__LINE__);
	} while (!kthread_should_stop());

	return 0;
}

static int vibrator_right_thread(void *data)
{
	struct usb_zhixu_xpad *zhixu_xpad = data;//(struct usb_zhixu_xpad *) data;
	//struct sched_param param = {.sched_priority = RTPM_PRIO_TPD};
	struct sched_param param = {.sched_priority = 4};

	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(wait_right, atomic_read(&zhixu_xpad->work_right_running) == 1);

		atomic_set(&zhixu_xpad->work_right_running, 0);
		set_current_state(TASK_RUNNING);
		
		//zhixu_xpad->vibrator_level_left = temp_value;
		share_set_vibrator_level_right(zhixu_xpad->vibrator_level_right);
		pr_info("%s %d\n",__func__,__LINE__);
	} while (!kthread_should_stop());

	return 0;
}
 

#if 0
static void vibrator_left_do_work(struct work_struct *work)
{
	struct usb_zhixu_xpad *zhixu_xpad = container_of(work, struct usb_zhixu_xpad, vibrator_left_work);
	
	atomic_set(&zhixu_xpad->work_left_running, 1); // 设置工作项标志变量
	//pr_debug("%s %d\n", __func__, __LINE__);
    share_set_vibrator_level_left(zhixu_xpad->vibrator_level_left);
    atomic_set(&zhixu_xpad->work_left_running, 0); // 清除工作项标志变量 
}

static void vibrator_right_do_work(struct work_struct *work)
{
	struct usb_zhixu_xpad *zhixu_xpad = container_of(work, struct usb_zhixu_xpad, vibrator_right_work);
	
	atomic_set(&zhixu_xpad->work_right_running, 1); // 设置工作项标志变量
	//pr_debug("%s %d\n", __func__, __LINE__);
    share_set_vibrator_level_right(zhixu_xpad->vibrator_level_right);
    atomic_set(&zhixu_xpad->work_right_running, 0); // 清除工作项标志变量 
}
#endif 
static void zhixu_xpad_presence_work(struct work_struct *work)
{
	struct usb_zhixu_xpad *zhixu_xpad = container_of(work, struct usb_zhixu_xpad, work);
	int error;

	if (zhixu_xpad->pad_present) {
		error = zhixu_xpad_init_input(zhixu_xpad);
		if (error) {
			/* complain only, not much else we can do here */
			dev_err(&zhixu_xpad->dev->dev,
				"unable to init device: %d\n", error);
		} else {
			rcu_assign_pointer(zhixu_xpad->x360w_dev, zhixu_xpad->dev);
		}
	} else {
		RCU_INIT_POINTER(zhixu_xpad->x360w_dev, NULL);
		synchronize_rcu();
		/*
		 * Now that we are sure zhixu_xpad360w_process_packet is not
		 * using input device we can get rid of it.
		 */
		zhixu_xpad_deinit_input(zhixu_xpad);
	}
}

/*
 * zhixu_xpad360w_process_packet
 *
 * Completes a request by converting the data into events for the
 * input subsystem. It is version for xbox 360 wireless controller.
 *
 * Byte.Bit
 * 00.1 - Status change: The controller or headset has connected/disconnected
 *                       Bits 01.7 and 01.6 are valid
 * 01.7 - Controller present
 * 01.6 - Headset present
 * 01.1 - Pad state (Bytes 4+) valid
 *
 */
static void zhixu_xpad360w_process_packet(struct usb_zhixu_xpad *zhixu_xpad, u16 cmd, unsigned char *data)
{
	struct input_dev *dev;
	bool present;

	/* Presence change */
	if (data[0] & 0x08) {
		present = (data[1] & 0x80) != 0;

		if (zhixu_xpad->pad_present != present) {
			zhixu_xpad->pad_present = present;
			schedule_work(&zhixu_xpad->work);
		}
	}

	/* Valid pad data */
	if (data[1] != 0x1)
		return;

	rcu_read_lock();
	dev = rcu_dereference(zhixu_xpad->x360w_dev);
	if (dev)
		zhixu_xpad360_process_packet(zhixu_xpad, dev, cmd, &data[4]);
	rcu_read_unlock();
}

/*
 *	zhixu_xpadone_process_packet
 *
 *	Completes a request by converting the data into events for the
 *	input subsystem. This version is for the Xbox One controller.
 *
 *	The report format was gleaned from
 *	https://github.com/kylelemons/xbox/blob/master/xbox.go
 */
static void zhixu_xpadone_process_packet(struct usb_zhixu_xpad *zhixu_xpad, u16 cmd, unsigned char *data)
{
	struct input_dev *dev = zhixu_xpad->dev;

	/* the xbox button has its own special report */
	if (data[0] == 0X07) {
		/*
		 * The Xbox One S controller requires these reports to be
		 * acked otherwise it continues sending them forever and
		 * won't report further mode button events.
		 */
		if (data[1] == 0x30)
			zhixu_xpadone_ack_mode_report(zhixu_xpad, data[2]);

		input_report_key(dev, BTN_MODE, data[4] & 0x01);
		input_sync(dev);
		return;
	}
	/* check invalid packet */
	else if (data[0] != 0X20)
		return;

	/* menu/view buttons */
	input_report_key(dev, BTN_START,  data[4] & 0x04);
	input_report_key(dev, BTN_SELECT, data[4] & 0x08);

	/* buttons A,B,X,Y */
	input_report_key(dev, BTN_A,	data[4] & 0x10);
	input_report_key(dev, BTN_B,	data[4] & 0x20);
	input_report_key(dev, BTN_X,	data[4] & 0x40);
	input_report_key(dev, BTN_Y,	data[4] & 0x80);

	/* digital pad */
	if (zhixu_xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		/* dpad as buttons (left, right, up, down) */
		input_report_key(dev, BTN_TRIGGER_HAPPY1, data[5] & 0x04);
		input_report_key(dev, BTN_TRIGGER_HAPPY2, data[5] & 0x08);
		input_report_key(dev, BTN_TRIGGER_HAPPY3, data[5] & 0x01);
		input_report_key(dev, BTN_TRIGGER_HAPPY4, data[5] & 0x02);
	} else {
		input_report_abs(dev, ABS_HAT0X,
				 !!(data[5] & 0x08) - !!(data[5] & 0x04));
		input_report_abs(dev, ABS_HAT0Y,
				 !!(data[5] & 0x02) - !!(data[5] & 0x01));
	}

	/* TL/TR */
	input_report_key(dev, BTN_TL,	data[5] & 0x10);
	input_report_key(dev, BTN_TR,	data[5] & 0x20);

	/* stick press left/right */
	input_report_key(dev, BTN_THUMBL, data[5] & 0x40);
	input_report_key(dev, BTN_THUMBR, data[5] & 0x80);

	if (!(zhixu_xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* left stick */
		input_report_abs(dev, ABS_X,
				 (__s16) le16_to_cpup((__le16 *)(data + 10)));
		input_report_abs(dev, ABS_Y,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 12)));

		/* right stick */
		input_report_abs(dev, ABS_RX,
				 (__s16) le16_to_cpup((__le16 *)(data + 14)));
		input_report_abs(dev, ABS_RY,
				 ~(__s16) le16_to_cpup((__le16 *)(data + 16)));
	}

	/* triggers left/right */
	if (zhixu_xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		input_report_key(dev, BTN_TL2,
				 (__u16) le16_to_cpup((__le16 *)(data + 6)));
		input_report_key(dev, BTN_TR2,
				 (__u16) le16_to_cpup((__le16 *)(data + 8)));
	} else {
		input_report_abs(dev, ABS_Z,
				 (__u16) le16_to_cpup((__le16 *)(data + 6)));
		input_report_abs(dev, ABS_RZ,
				 (__u16) le16_to_cpup((__le16 *)(data + 8)));
	}

	input_sync(dev);
}

static void zhixu_xpad_irq_in(struct urb *urb)
{
	struct usb_zhixu_xpad *zhixu_xpad = urb->context;
	struct device *dev = &zhixu_xpad->intf->dev;
	int retval, status;

	status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

	switch (zhixu_xpad->xtype) {
	case XTYPE_XBOX360:
		zhixu_xpad360_process_packet(zhixu_xpad, zhixu_xpad->dev, 0, zhixu_xpad->idata);
		break;
	case XTYPE_XBOX360W:
		zhixu_xpad360w_process_packet(zhixu_xpad, 0, zhixu_xpad->idata);
		break;
	case XTYPE_XBOXONE:
		zhixu_xpadone_process_packet(zhixu_xpad, 0, zhixu_xpad->idata);
		break;
	default:
		zhixu_xpad_process_packet(zhixu_xpad, 0, zhixu_xpad->idata);
	}

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(dev, "%s - usb_submit_urb failed with result %d\n",
			__func__, retval);
}

/* Callers must hold zhixu_xpad->odata_lock spinlock */
static bool zhixu_xpad_prepare_next_init_packet(struct usb_zhixu_xpad *zhixu_xpad)
{
	const struct xboxone_init_packet *init_packet;

	if (zhixu_xpad->xtype != XTYPE_XBOXONE)
		return false;

	/* Perform initialization sequence for Xbox One pads that require it */
	while (zhixu_xpad->init_seq < ARRAY_SIZE(xboxone_init_packets)) {
		init_packet = &xboxone_init_packets[zhixu_xpad->init_seq++];

		if (init_packet->idVendor != 0 &&
		    init_packet->idVendor != zhixu_xpad->dev->id.vendor)
			continue;

		if (init_packet->idProduct != 0 &&
		    init_packet->idProduct != zhixu_xpad->dev->id.product)
			continue;

		/* This packet applies to our device, so prepare to send it */
		memcpy(zhixu_xpad->odata, init_packet->data, init_packet->len);
		zhixu_xpad->irq_out->transfer_buffer_length = init_packet->len;

		/* Update packet with current sequence number */
		zhixu_xpad->odata[2] = zhixu_xpad->odata_serial++;
		return true;
	}

	return false;
}

/* Callers must hold zhixu_xpad->odata_lock spinlock */
static bool zhixu_xpad_prepare_next_out_packet(struct usb_zhixu_xpad *zhixu_xpad)
{
	struct zhixu_xpad_output_packet *pkt, *packet = NULL;
	int i;

	/* We may have init packets to send before we can send user commands */
	if (zhixu_xpad_prepare_next_init_packet(zhixu_xpad))
		return true;

	for (i = 0; i < XPAD_NUM_OUT_PACKETS; i++) {
		if (++zhixu_xpad->last_out_packet >= XPAD_NUM_OUT_PACKETS)
			zhixu_xpad->last_out_packet = 0;

		pkt = &zhixu_xpad->out_packets[zhixu_xpad->last_out_packet];
		if (pkt->pending) {
			dev_dbg(&zhixu_xpad->intf->dev,
				"%s - found pending output packet %d\n",
				__func__, zhixu_xpad->last_out_packet);
			packet = pkt;
			break;
		}
	}

	if (packet) {
		memcpy(zhixu_xpad->odata, packet->data, packet->len);
		zhixu_xpad->irq_out->transfer_buffer_length = packet->len;
		packet->pending = false;
		return true;
	}

	return false;
}

/* Callers must hold zhixu_xpad->odata_lock spinlock */
static int zhixu_xpad_try_sending_next_out_packet(struct usb_zhixu_xpad *zhixu_xpad)
{
	int error;

	if (!zhixu_xpad->irq_out_active && zhixu_xpad_prepare_next_out_packet(zhixu_xpad)) {
		usb_anchor_urb(zhixu_xpad->irq_out, &zhixu_xpad->irq_out_anchor);
		error = usb_submit_urb(zhixu_xpad->irq_out, GFP_ATOMIC);
		if (error) {
			dev_err(&zhixu_xpad->intf->dev,
				"%s - usb_submit_urb failed with result %d\n",
				__func__, error);
			usb_unanchor_urb(zhixu_xpad->irq_out);
			return -EIO;
		}

		zhixu_xpad->irq_out_active = true;
	}

	return 0;
}

static void zhixu_xpad_irq_out(struct urb *urb)
{
	struct usb_zhixu_xpad *zhixu_xpad = urb->context;
	struct device *dev = &zhixu_xpad->intf->dev;
	int status = urb->status;
	int error;
	unsigned long flags;

	spin_lock_irqsave(&zhixu_xpad->odata_lock, flags);

	switch (status) {
	case 0:
		/* success */
		zhixu_xpad->irq_out_active = zhixu_xpad_prepare_next_out_packet(zhixu_xpad);
		break;

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		zhixu_xpad->irq_out_active = false;
		break;

	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n",
			__func__, status);
		break;
	}

	if (zhixu_xpad->irq_out_active) {
		usb_anchor_urb(urb, &zhixu_xpad->irq_out_anchor);
		error = usb_submit_urb(urb, GFP_ATOMIC);
		if (error) {
			dev_err(dev,
				"%s - usb_submit_urb failed with result %d\n",
				__func__, error);
			usb_unanchor_urb(urb);
			zhixu_xpad->irq_out_active = false;
		}
	}

	spin_unlock_irqrestore(&zhixu_xpad->odata_lock, flags);
}

static int zhixu_xpad_init_output(struct usb_interface *intf, struct usb_zhixu_xpad *zhixu_xpad,
			struct usb_endpoint_descriptor *ep_irq_out)
{
	int error;

	if (zhixu_xpad->xtype == XTYPE_UNKNOWN)
		return 0;

	init_usb_anchor(&zhixu_xpad->irq_out_anchor);

	zhixu_xpad->odata = usb_alloc_coherent(zhixu_xpad->udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &zhixu_xpad->odata_dma);
	if (!zhixu_xpad->odata)
		return -ENOMEM;

	spin_lock_init(&zhixu_xpad->odata_lock);

	zhixu_xpad->irq_out = usb_alloc_urb(0, GFP_KERNEL);
	if (!zhixu_xpad->irq_out) {
		error = -ENOMEM;
		goto err_free_coherent;
	}

	usb_fill_int_urb(zhixu_xpad->irq_out, zhixu_xpad->udev,
			 usb_sndintpipe(zhixu_xpad->udev, ep_irq_out->bEndpointAddress),
			 zhixu_xpad->odata, XPAD_PKT_LEN,
			 zhixu_xpad_irq_out, zhixu_xpad, ep_irq_out->bInterval);
	zhixu_xpad->irq_out->transfer_dma = zhixu_xpad->odata_dma;
	zhixu_xpad->irq_out->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return 0;

err_free_coherent:
	usb_free_coherent(zhixu_xpad->udev, XPAD_PKT_LEN, zhixu_xpad->odata, zhixu_xpad->odata_dma);
	return error;
}

static void zhixu_xpad_stop_output(struct usb_zhixu_xpad *zhixu_xpad)
{
	if (zhixu_xpad->xtype != XTYPE_UNKNOWN) {
		if (!usb_wait_anchor_empty_timeout(&zhixu_xpad->irq_out_anchor,
						   5000)) {
			dev_warn(&zhixu_xpad->intf->dev,
				 "timed out waiting for output URB to complete, killing\n");
			usb_kill_anchored_urbs(&zhixu_xpad->irq_out_anchor);
		}
	}
}

static void zhixu_xpad_deinit_output(struct usb_zhixu_xpad *zhixu_xpad)
{
	if (zhixu_xpad->xtype != XTYPE_UNKNOWN) {
		usb_free_urb(zhixu_xpad->irq_out);
		usb_free_coherent(zhixu_xpad->udev, XPAD_PKT_LEN,
				zhixu_xpad->odata, zhixu_xpad->odata_dma);
	}
}

static int zhixu_xpad_inquiry_pad_presence(struct usb_zhixu_xpad *zhixu_xpad)
{
	struct zhixu_xpad_output_packet *packet =
			&zhixu_xpad->out_packets[XPAD_OUT_CMD_IDX];
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&zhixu_xpad->odata_lock, flags);

	packet->data[0] = 0x08;
	packet->data[1] = 0x00;
	packet->data[2] = 0x0F;
	packet->data[3] = 0xC0;
	packet->data[4] = 0x00;
	packet->data[5] = 0x00;
	packet->data[6] = 0x00;
	packet->data[7] = 0x00;
	packet->data[8] = 0x00;
	packet->data[9] = 0x00;
	packet->data[10] = 0x00;
	packet->data[11] = 0x00;
	packet->len = 12;
	packet->pending = true;

	/* Reset the sequence so we send out presence first */
	zhixu_xpad->last_out_packet = -1;
	retval = zhixu_xpad_try_sending_next_out_packet(zhixu_xpad);

	spin_unlock_irqrestore(&zhixu_xpad->odata_lock, flags);

	return retval;
}

static int zhixu_xpad_start_xbox_one(struct usb_zhixu_xpad *zhixu_xpad)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&zhixu_xpad->odata_lock, flags);

	/*
	 * Begin the init sequence by attempting to send a packet.
	 * We will cycle through the init packet sequence before
	 * sending any packets from the output ring.
	 */
	zhixu_xpad->init_seq = 0;
	retval = zhixu_xpad_try_sending_next_out_packet(zhixu_xpad);

	spin_unlock_irqrestore(&zhixu_xpad->odata_lock, flags);

	return retval;
}

static void zhixu_xpadone_ack_mode_report(struct usb_zhixu_xpad *zhixu_xpad, u8 seq_num)
{
	unsigned long flags;
	struct zhixu_xpad_output_packet *packet =
			&zhixu_xpad->out_packets[XPAD_OUT_CMD_IDX];
	static const u8 mode_report_ack[] = {
		0x01, 0x20, 0x00, 0x09, 0x00, 0x07, 0x20, 0x02,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	spin_lock_irqsave(&zhixu_xpad->odata_lock, flags);

	packet->len = sizeof(mode_report_ack);
	memcpy(packet->data, mode_report_ack, packet->len);
	packet->data[2] = seq_num;
	packet->pending = true;

	/* Reset the sequence so we send out the ack now */
	zhixu_xpad->last_out_packet = -1;
	zhixu_xpad_try_sending_next_out_packet(zhixu_xpad);

	spin_unlock_irqrestore(&zhixu_xpad->odata_lock, flags);
}

#ifdef CONFIG_JOYSTICK_XPAD_FF
static int zhixu_xpad_play_effect(struct input_dev *dev, void *data, struct ff_effect *effect)
{
	struct usb_zhixu_xpad *zhixu_xpad = input_get_drvdata(dev);
#if 0	
	struct zhixu_xpad_output_packet *packet = &zhixu_xpad->out_packets[XPAD_OUT_FF_IDX];
#endif 
	__u16 strong;
	__u16 weak;
	int retval = 0;
	unsigned long flags;
	//static int count = 30; 

	if (effect->type != FF_RUMBLE){ 
		return 0;
	} 

	strong = effect->u.rumble.strong_magnitude;
	weak = effect->u.rumble.weak_magnitude;  
		
	spin_lock_irqsave(&zhixu_xpad->odata_lock, flags);	
	///Ysjie modify///
  
	zhixu_xpad->vibrator_level_left = strong / 256;
	zhixu_xpad->vibrator_level_right = weak / 256;
	
	wakeup_vibrator_thread(zhixu_xpad);
#if 0	
	if (atomic_read(&zhixu_xpad->work_left_running) == 1) {
		/* 工作项正在运行 */ 
		if(zhixu_xpad->vibrator_level_left == 0 && zhixu_xpad->vibrator_level_right == 0){
			pr_info("vibrator_level_left == 0 && vibrator_level_right == 0\n");
			retval = queue_work(zhixu_xpad->vibrator_left_workqueue, &zhixu_xpad->vibrator_left_work); 
			//retval = schedule_work(&zhixu_xpad->vibrator_work); 
			if (!retval)
				pr_err("xpad : vibrator_left_workqueue queue_work failed retval:%d\n", retval);
		}
	} else {
		/* 工作项未运行 */
		//pr_info("%s %d is not work_running\n", __func__, __LINE__);
		//retval = schedule_work(&zhixu_xpad->vibrator_work);
		retval = queue_work(zhixu_xpad->vibrator_left_workqueue, &zhixu_xpad->vibrator_left_work); 
		if (!retval)
			pr_err("xpad : vibrator_left_workqueue queue_work failed retval:%d\n", retval);
	}
	
	if (atomic_read(&zhixu_xpad->work_right_running) == 1) {
		/* 工作项正在运行 */ 
		if(zhixu_xpad->vibrator_level_left == 0 && zhixu_xpad->vibrator_level_right == 0){
			pr_info("vibrator_level_left == 0 && vibrator_level_right == 0\n");
			retval = queue_work(zhixu_xpad->vibrator_right_workqueue, &zhixu_xpad->vibrator_right_work); 
			//retval = schedule_work(&zhixu_xpad->vibrator_work); 
			if (!retval)
				pr_err("xpad : vibrator_right_workqueue queue_work failed retval:%d\n", retval);
		}
	} else {
		/* 工作项未运行 */
		//pr_info("%s %d is not work_running\n", __func__, __LINE__);
		//retval = schedule_work(&zhixu_xpad->vibrator_work);
		retval = queue_work(zhixu_xpad->vibrator_right_workqueue, &zhixu_xpad->vibrator_right_work); 
		if (!retval)
			pr_err("xpad : vibrator_right_workqueue queue_work failed retval:%d\n", retval);
	}
#endif 
    ///end modify///
#if 0 
	switch (zhixu_xpad->xtype) {
	case XTYPE_XBOX:
		packet->data[0] = 0x00;
		packet->data[1] = 0x06;
		packet->data[2] = 0x00;
		packet->data[3] = strong / 256;	/* left actuator */
		packet->data[4] = 0x00;
		packet->data[5] = weak / 256;	/* right actuator */
		packet->len = 6;
		packet->pending = true;
		break;

	case XTYPE_XBOX360:  
		packet->data[0] = 0x00;
		packet->data[1] = 0x08;
		packet->data[2] = 0x00;
		packet->data[3] = strong / 256;  /* left actuator? */
		packet->data[4] = weak / 256;	/* right actuator? */
		packet->data[5] = 0x00;
		packet->data[6] = 0x00;
		packet->data[7] = 0x00;
		packet->len = 8;
		packet->pending = true; 
		break;

	case XTYPE_XBOX360W:
		packet->data[0] = 0x00;
		packet->data[1] = 0x01;
		packet->data[2] = 0x0F;
		packet->data[3] = 0xC0;
		packet->data[4] = 0x00;
		packet->data[5] = strong / 256;
		packet->data[6] = weak / 256;
		packet->data[7] = 0x00;
		packet->data[8] = 0x00;
		packet->data[9] = 0x00;
		packet->data[10] = 0x00;
		packet->data[11] = 0x00;
		packet->len = 12;
		packet->pending = true;
		break;

	case XTYPE_XBOXONE:
		packet->data[0] = 0x09; /* activate rumble */
		packet->data[1] = 0x00;
		packet->data[2] = zhixu_xpad->odata_serial++;
		packet->data[3] = 0x09;
		packet->data[4] = 0x00;
		packet->data[5] = 0x0F;
		packet->data[6] = 0x00;
		packet->data[7] = 0x00;
		packet->data[8] = strong / 512;	/* left actuator */
		packet->data[9] = weak / 512;	/* right actuator */
		packet->data[10] = 0xFF; /* on period */
		packet->data[11] = 0x00; /* off period */
		packet->data[12] = 0xFF; /* repeat count */
		packet->len = 13;
		packet->pending = true;
		break;

	default:
		dev_dbg(&zhixu_xpad->dev->dev,
			"%s - rumble command sent to unsupported zhixu_xpad type: %d\n",
			__func__, zhixu_xpad->xtype);
		retval = -EINVAL;
		goto out;
	}

	retval = zhixu_xpad_try_sending_next_out_packet(zhixu_xpad);

out:
#endif 
	spin_unlock_irqrestore(&zhixu_xpad->odata_lock, flags);
	return retval;
}

static int zhixu_xpad_init_ff(struct usb_zhixu_xpad *zhixu_xpad)
{
	//int res = 0;
	if (zhixu_xpad->xtype == XTYPE_UNKNOWN)
		return 0;

	input_set_capability(zhixu_xpad->dev, EV_FF, FF_RUMBLE);
	
	//vibrator_workqueue = create_singlethread_workqueue("xbox-vibrator");
	//vibrator_workqueue = alloc_workqueue("vibrator_workqueue", WQ_UNBOUND, 1); 
	//set_workqueue_priority(vibrator_workqueue, WQ_CONGESTION_WAIT); 
	//INIT_WORK(&vibrator_work, vibrator_do_work);

	//res = queue_work(vibrator_workqueue, &vibrator_work);
	//if (!res)
	//	pr_err("xpad : vibrator_workqueue init failed res:%d\n", res);
	
	return input_ff_create_memless(zhixu_xpad->dev, NULL, zhixu_xpad_play_effect);
}

#else
static int zhixu_xpad_init_ff(struct usb_zhixu_xpad *zhixu_xpad) { return 0; }
#endif

#if defined(CONFIG_JOYSTICK_XPAD_LEDS)
#include <linux/leds.h>
#include <linux/idr.h>

static DEFINE_IDA(zhixu_xpad_pad_seq);

struct zhixu_xpad_led {
	char name[16];
	struct led_classdev led_cdev;
	struct usb_zhixu_xpad *zhixu_xpad;
};

/**
 * set the LEDs on Xbox360 / Wireless Controllers
 * @param command
 *  0: off
 *  1: all blink, then previous setting
 *  2: 1/top-left blink, then on
 *  3: 2/top-right blink, then on
 *  4: 3/bottom-left blink, then on
 *  5: 4/bottom-right blink, then on
 *  6: 1/top-left on
 *  7: 2/top-right on
 *  8: 3/bottom-left on
 *  9: 4/bottom-right on
 * 10: rotate
 * 11: blink, based on previous setting
 * 12: slow blink, based on previous setting
 * 13: rotate with two lights
 * 14: persistent slow all blink
 * 15: blink once, then previous setting
 */
static void zhixu_xpad_send_led_command(struct usb_zhixu_xpad *zhixu_xpad, int command)
{
	struct zhixu_xpad_output_packet *packet =
			&zhixu_xpad->out_packets[XPAD_OUT_LED_IDX];
	unsigned long flags;

	command %= 16;

	spin_lock_irqsave(&zhixu_xpad->odata_lock, flags);

	switch (zhixu_xpad->xtype) {
	case XTYPE_XBOX360:
		packet->data[0] = 0x01;
		packet->data[1] = 0x03;
		packet->data[2] = command;
		packet->len = 3;
		packet->pending = true;
		break;

	case XTYPE_XBOX360W:
		packet->data[0] = 0x00;
		packet->data[1] = 0x00;
		packet->data[2] = 0x08;
		packet->data[3] = 0x40 + command;
		packet->data[4] = 0x00;
		packet->data[5] = 0x00;
		packet->data[6] = 0x00;
		packet->data[7] = 0x00;
		packet->data[8] = 0x00;
		packet->data[9] = 0x00;
		packet->data[10] = 0x00;
		packet->data[11] = 0x00;
		packet->len = 12;
		packet->pending = true;
		break;
	}

	zhixu_xpad_try_sending_next_out_packet(zhixu_xpad);

	spin_unlock_irqrestore(&zhixu_xpad->odata_lock, flags);
}

/*
 * Light up the segment corresponding to the pad number on
 * Xbox 360 Controllers.
 */
static void zhixu_xpad_identify_controller(struct usb_zhixu_xpad *zhixu_xpad)
{
	led_set_brightness(&zhixu_xpad->led->led_cdev, (zhixu_xpad->pad_nr % 4) + 2);
}

static void zhixu_xpad_led_set(struct led_classdev *led_cdev,
			 enum led_brightness value)
{
	struct zhixu_xpad_led *zhixu_xpad_led = container_of(led_cdev,
						 struct zhixu_xpad_led, led_cdev);

	zhixu_xpad_send_led_command(zhixu_xpad_led->zhixu_xpad, value);
}

static int zhixu_xpad_led_probe(struct usb_zhixu_xpad *zhixu_xpad)
{
	struct zhixu_xpad_led *led;
	struct led_classdev *led_cdev;
	int error;

	if (zhixu_xpad->xtype != XTYPE_XBOX360 && zhixu_xpad->xtype != XTYPE_XBOX360W)
		return 0;

	zhixu_xpad->led = led = kzalloc(sizeof(struct zhixu_xpad_led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	zhixu_xpad->pad_nr = ida_simple_get(&zhixu_xpad_pad_seq, 0, 0, GFP_KERNEL);
	if (zhixu_xpad->pad_nr < 0) {
		error = zhixu_xpad->pad_nr;
		goto err_free_mem;
	}

	snprintf(led->name, sizeof(led->name), "zhixu_xpad%d", zhixu_xpad->pad_nr);
	led->zhixu_xpad = zhixu_xpad;

	led_cdev = &led->led_cdev;
	led_cdev->name = led->name;
	led_cdev->brightness_set = zhixu_xpad_led_set;
	led_cdev->flags = LED_CORE_SUSPENDRESUME;

	error = led_classdev_register(&zhixu_xpad->udev->dev, led_cdev);
	if (error)
		goto err_free_id;

	zhixu_xpad_identify_controller(zhixu_xpad);

	return 0;

err_free_id:
	ida_simple_remove(&zhixu_xpad_pad_seq, zhixu_xpad->pad_nr);
err_free_mem:
	kfree(led);
	zhixu_xpad->led = NULL;
	return error;
}

static void zhixu_xpad_led_disconnect(struct usb_zhixu_xpad *zhixu_xpad)
{
	struct zhixu_xpad_led *zhixu_xpad_led = zhixu_xpad->led;

	if (zhixu_xpad_led) {
		led_classdev_unregister(&zhixu_xpad_led->led_cdev);
		ida_simple_remove(&zhixu_xpad_pad_seq, zhixu_xpad->pad_nr);
		kfree(zhixu_xpad_led);
	}
}
#else
static int zhixu_xpad_led_probe(struct usb_zhixu_xpad *zhixu_xpad) { return 0; }
static void zhixu_xpad_led_disconnect(struct usb_zhixu_xpad *zhixu_xpad) { }
#endif

static int zhixu_xpad_start_input(struct usb_zhixu_xpad *zhixu_xpad)
{
	int error;

	if (usb_submit_urb(zhixu_xpad->irq_in, GFP_KERNEL))
		return -EIO;

	if (zhixu_xpad->xtype == XTYPE_XBOXONE) {
		error = zhixu_xpad_start_xbox_one(zhixu_xpad);
		if (error) {
			usb_kill_urb(zhixu_xpad->irq_in);
			return error;
		}
	}

	return 0;
}

static void zhixu_xpad_stop_input(struct usb_zhixu_xpad *zhixu_xpad)
{
	usb_kill_urb(zhixu_xpad->irq_in);
}

static void zhixu_xpad360w_poweroff_controller(struct usb_zhixu_xpad *zhixu_xpad)
{
	unsigned long flags;
	struct zhixu_xpad_output_packet *packet =
			&zhixu_xpad->out_packets[XPAD_OUT_CMD_IDX];

	spin_lock_irqsave(&zhixu_xpad->odata_lock, flags);

	packet->data[0] = 0x00;
	packet->data[1] = 0x00;
	packet->data[2] = 0x08;
	packet->data[3] = 0xC0;
	packet->data[4] = 0x00;
	packet->data[5] = 0x00;
	packet->data[6] = 0x00;
	packet->data[7] = 0x00;
	packet->data[8] = 0x00;
	packet->data[9] = 0x00;
	packet->data[10] = 0x00;
	packet->data[11] = 0x00;
	packet->len = 12;
	packet->pending = true;

	/* Reset the sequence so we send out poweroff now */
	zhixu_xpad->last_out_packet = -1;
	zhixu_xpad_try_sending_next_out_packet(zhixu_xpad);

	spin_unlock_irqrestore(&zhixu_xpad->odata_lock, flags);
}

static int zhixu_xpad360w_start_input(struct usb_zhixu_xpad *zhixu_xpad)
{
	int error;

	error = usb_submit_urb(zhixu_xpad->irq_in, GFP_KERNEL);
	if (error)
		return -EIO;

	/*
	 * Send presence packet.
	 * This will force the controller to resend connection packets.
	 * This is useful in the case we activate the module after the
	 * adapter has been plugged in, as it won't automatically
	 * send us info about the controllers.
	 */
	error = zhixu_xpad_inquiry_pad_presence(zhixu_xpad);
	if (error) {
		usb_kill_urb(zhixu_xpad->irq_in);
		return error;
	}

	return 0;
}

static void zhixu_xpad360w_stop_input(struct usb_zhixu_xpad *zhixu_xpad)
{
	usb_kill_urb(zhixu_xpad->irq_in);

	/* Make sure we are done with presence work if it was scheduled */
	flush_work(&zhixu_xpad->work);
}

static int zhixu_xpad_open(struct input_dev *dev)
{
	struct usb_zhixu_xpad *zhixu_xpad = input_get_drvdata(dev);

	return zhixu_xpad_start_input(zhixu_xpad);
}

static void zhixu_xpad_close(struct input_dev *dev)
{
	struct usb_zhixu_xpad *zhixu_xpad = input_get_drvdata(dev);

	zhixu_xpad_stop_input(zhixu_xpad);
}

static void zhixu_xpad_set_up_abs(struct input_dev *input_dev, signed short abs)
{
	struct usb_zhixu_xpad *zhixu_xpad = input_get_drvdata(input_dev);

	switch (abs) {
	case ABS_X:
	case ABS_Y:
	case ABS_RX:
	case ABS_RY:	/* the two sticks */
		input_set_abs_params(input_dev, abs, -32768, 32767, 16, 128);
		break;
	case ABS_Z:
	case ABS_RZ:	/* the triggers (if mapped to axes) */
		if (zhixu_xpad->xtype == XTYPE_XBOXONE)
			input_set_abs_params(input_dev, abs, 0, 1023, 0, 0);
		else
			input_set_abs_params(input_dev, abs, 0, 255, 0, 0);
		break;
	case ABS_HAT0X:
	case ABS_HAT0Y:	/* the d-pad (only if dpad is mapped to axes */
		input_set_abs_params(input_dev, abs, -1, 1, 0, 0);
		break;
	default:
		input_set_abs_params(input_dev, abs, 0, 0, 0, 0);
		break;
	}
}

static void zhixu_xpad_deinit_input(struct usb_zhixu_xpad *zhixu_xpad)
{
	if (zhixu_xpad->input_created) {
		zhixu_xpad->input_created = false;
		zhixu_xpad_led_disconnect(zhixu_xpad);
		input_unregister_device(zhixu_xpad->dev);
	}
}

static int zhixu_xpad_init_input(struct usb_zhixu_xpad *zhixu_xpad)
{
	struct input_dev *input_dev;
	int i, error;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	zhixu_xpad->dev = input_dev;
	input_dev->name = zhixu_xpad->name;
	input_dev->phys = zhixu_xpad->phys;
	usb_to_input_id(zhixu_xpad->udev, &input_dev->id);

	if (zhixu_xpad->xtype == XTYPE_XBOX360W) {
		/* x360w controllers and the receiver have different ids */
		input_dev->id.product = 0x02a1;
	}

	input_dev->dev.parent = &zhixu_xpad->intf->dev;

	input_set_drvdata(input_dev, zhixu_xpad);

	if (zhixu_xpad->xtype != XTYPE_XBOX360W) {
		input_dev->open = zhixu_xpad_open;
		input_dev->close = zhixu_xpad_close;
	}

	if (!(zhixu_xpad->mapping & MAP_STICKS_TO_NULL)) {
		/* set up axes */
		for (i = 0; zhixu_xpad_abs[i] >= 0; i++)
			zhixu_xpad_set_up_abs(input_dev, zhixu_xpad_abs[i]);
	}

	/* set up standard buttons */
	for (i = 0; zhixu_xpad_common_btn[i] >= 0; i++)
		input_set_capability(input_dev, EV_KEY, zhixu_xpad_common_btn[i]);

	/* set up model-specific ones */
	if (zhixu_xpad->xtype == XTYPE_XBOX360 || zhixu_xpad->xtype == XTYPE_XBOX360W ||
	    zhixu_xpad->xtype == XTYPE_XBOXONE) {
		for (i = 0; zhixu_xpad360_btn[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY, zhixu_xpad360_btn[i]);
	} else {
		for (i = 0; zhixu_xpad_btn[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY, zhixu_xpad_btn[i]);
	}

	if (zhixu_xpad->mapping & MAP_DPAD_TO_BUTTONS) {
		for (i = 0; zhixu_xpad_btn_pad[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY,
					     zhixu_xpad_btn_pad[i]);
	}

	/*
	 * This should be a simple else block. However historically
	 * xbox360w has mapped DPAD to buttons while xbox360 did not. This
	 * made no sense, but now we can not just switch back and have to
	 * support both behaviors.
	 */
	if (!(zhixu_xpad->mapping & MAP_DPAD_TO_BUTTONS) ||
	    zhixu_xpad->xtype == XTYPE_XBOX360W) {
		for (i = 0; zhixu_xpad_abs_pad[i] >= 0; i++)
			zhixu_xpad_set_up_abs(input_dev, zhixu_xpad_abs_pad[i]);
	}

	if (zhixu_xpad->mapping & MAP_TRIGGERS_TO_BUTTONS) {
		for (i = 0; zhixu_xpad_btn_triggers[i] >= 0; i++)
			input_set_capability(input_dev, EV_KEY,
					     zhixu_xpad_btn_triggers[i]);
	} else {
		for (i = 0; zhixu_xpad_abs_triggers[i] >= 0; i++)
			zhixu_xpad_set_up_abs(input_dev, zhixu_xpad_abs_triggers[i]);
	}

	error = zhixu_xpad_init_ff(zhixu_xpad);
	if (error)
		goto err_free_input;

	error = zhixu_xpad_led_probe(zhixu_xpad);
	if (error)
		goto err_destroy_ff;

	error = input_register_device(zhixu_xpad->dev);
	if (error)
		goto err_disconnect_led;

	zhixu_xpad->input_created = true;
	return 0;

err_disconnect_led:
	zhixu_xpad_led_disconnect(zhixu_xpad);
err_destroy_ff:
	input_ff_destroy(input_dev);
err_free_input:
	input_free_device(input_dev);
	return error;
}

static int zhixu_xpad_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_zhixu_xpad *zhixu_xpad;
	struct usb_endpoint_descriptor *ep_irq_in, *ep_irq_out;
	int i, error;
	int ret;

	pr_info("%s %d\n",__func__,__LINE__);
	if (intf->cur_altsetting->desc.bNumEndpoints != 2)
		return -ENODEV;

	for (i = 0; zhixu_xpad_device[i].idVendor; i++) {
		if ((le16_to_cpu(udev->descriptor.idVendor) == zhixu_xpad_device[i].idVendor) &&
		    (le16_to_cpu(udev->descriptor.idProduct) == zhixu_xpad_device[i].idProduct))
			break;
	}

	zhixu_xpad = kzalloc(sizeof(struct usb_zhixu_xpad), GFP_KERNEL);
	if (!zhixu_xpad)
		return -ENOMEM;

	usb_make_path(udev, zhixu_xpad->phys, sizeof(zhixu_xpad->phys));
	strlcat(zhixu_xpad->phys, "/input0", sizeof(zhixu_xpad->phys));

	zhixu_xpad->idata = usb_alloc_coherent(udev, XPAD_PKT_LEN,
					 GFP_KERNEL, &zhixu_xpad->idata_dma);
	if (!zhixu_xpad->idata) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	zhixu_xpad->irq_in = usb_alloc_urb(0, GFP_KERNEL);
	if (!zhixu_xpad->irq_in) {
		error = -ENOMEM;
		goto err_free_idata;
	}

	zhixu_xpad->udev = udev;
	zhixu_xpad->intf = intf;
	zhixu_xpad->mapping = zhixu_xpad_device[i].mapping;
	zhixu_xpad->xtype = zhixu_xpad_device[i].xtype;
	zhixu_xpad->name = zhixu_xpad_device[i].name;
	
	/*Ysjie modify*/   
	atomic_set(&zhixu_xpad->work_left_running, 0); 
	atomic_set(&zhixu_xpad->work_right_running, 0); 
	zhixu_xpad->vibrator_level_left = 0;
	zhixu_xpad->vibrator_level_right = 0;
	//init_waitqueue_head(&zhixu_xpad->wait_left);
	//init_waitqueue_head(&zhixu_xpad->wait_right);
	zhixu_xpad->thread_vibrator_left = kthread_run(vibrator_left_thread, zhixu_xpad, "xbox-vibrator-left-thread");
	if (IS_ERR_OR_NULL(zhixu_xpad->thread_vibrator_left)) {
		ret = PTR_ERR(zhixu_xpad->thread_vibrator_left);
		pr_err("create kernel thread_vibrator_left fail,ret:%d", ret);
		zhixu_xpad->thread_vibrator_left = NULL;
		error = -ENODEV;
		goto err_deinit_output;
	} 
	
	zhixu_xpad->thread_vibrator_right = kthread_run(vibrator_right_thread, zhixu_xpad, "xbox-vibrator-right-thread");
	if (IS_ERR_OR_NULL(zhixu_xpad->thread_vibrator_right)) {
		ret = PTR_ERR(zhixu_xpad->thread_vibrator_right);
		pr_err("create kernel thread_vibrator_right fail,ret:%d", ret);
		zhixu_xpad->thread_vibrator_right = NULL;
		error = -ENODEV;
		goto err_deinit_output;
	} 
	
	/*end modify*/
	
	/*
	zhixu_xpad->vibrator_left_workqueue = alloc_workqueue("vibrator_left_workqueue", WQ_UNBOUND, 1); 
	zhixu_xpad->vibrator_right_workqueue = alloc_workqueue("vibrator_right_workqueue", WQ_UNBOUND, 1);  
	INIT_WORK(&zhixu_xpad->vibrator_left_work, vibrator_left_do_work);
	INIT_WORK(&zhixu_xpad->vibrator_right_work, vibrator_right_do_work);
	*/
	INIT_WORK(&zhixu_xpad->work, zhixu_xpad_presence_work); 

	if (zhixu_xpad->xtype == XTYPE_UNKNOWN) {
		if (intf->cur_altsetting->desc.bInterfaceClass == USB_CLASS_VENDOR_SPEC) {
			if (intf->cur_altsetting->desc.bInterfaceProtocol == 129)
				zhixu_xpad->xtype = XTYPE_XBOX360W;
			else if (intf->cur_altsetting->desc.bInterfaceProtocol == 208)
				zhixu_xpad->xtype = XTYPE_XBOXONE;
			else
				zhixu_xpad->xtype = XTYPE_XBOX360;
		} else {
			zhixu_xpad->xtype = XTYPE_XBOX;
		}

		if (dpad_to_buttons)
			zhixu_xpad->mapping |= MAP_DPAD_TO_BUTTONS;
		if (triggers_to_buttons)
			zhixu_xpad->mapping |= MAP_TRIGGERS_TO_BUTTONS;
		if (sticks_to_null)
			zhixu_xpad->mapping |= MAP_STICKS_TO_NULL;
	}

	if (zhixu_xpad->xtype == XTYPE_XBOXONE &&
	    intf->cur_altsetting->desc.bInterfaceNumber != 0) {
		/*
		 * The Xbox One controller lists three interfaces all with the
		 * same interface class, subclass and protocol. Differentiate by
		 * interface number.
		 */
		error = -ENODEV;
		goto err_free_in_urb;
	}

	ep_irq_in = ep_irq_out = NULL;

	for (i = 0; i < 2; i++) {
		struct usb_endpoint_descriptor *ep =
				&intf->cur_altsetting->endpoint[i].desc;

		if (usb_endpoint_xfer_int(ep)) {
			if (usb_endpoint_dir_in(ep))
				ep_irq_in = ep;
			else
				ep_irq_out = ep;
		}
	}

	if (!ep_irq_in || !ep_irq_out) {
		error = -ENODEV;
		goto err_free_in_urb;
	}

	error = zhixu_xpad_init_output(intf, zhixu_xpad, ep_irq_out);
	if (error)
		goto err_free_in_urb;

	usb_fill_int_urb(zhixu_xpad->irq_in, udev,
			 usb_rcvintpipe(udev, ep_irq_in->bEndpointAddress),
			 zhixu_xpad->idata, XPAD_PKT_LEN, zhixu_xpad_irq_in,
			 zhixu_xpad, ep_irq_in->bInterval);
	zhixu_xpad->irq_in->transfer_dma = zhixu_xpad->idata_dma;
	zhixu_xpad->irq_in->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_set_intfdata(intf, zhixu_xpad);

	if (zhixu_xpad->xtype == XTYPE_XBOX360W) {
		/*
		 * Submit the int URB immediately rather than waiting for open
		 * because we get status messages from the device whether
		 * or not any controllers are attached.  In fact, it's
		 * exactly the message that a controller has arrived that
		 * we're waiting for.
		 */
		error = zhixu_xpad360w_start_input(zhixu_xpad);
		if (error)
			goto err_deinit_output;
		/*
		 * Wireless controllers require RESET_RESUME to work properly
		 * after suspend. Ideally this quirk should be in usb core
		 * quirk list, but we have too many vendors producing these
		 * controllers and we'd need to maintain 2 identical lists
		 * here in this driver and in usb core.
		 */
		udev->quirks |= USB_QUIRK_RESET_RESUME;
	} else {
		error = zhixu_xpad_init_input(zhixu_xpad);
		if (error)
			goto err_deinit_output;
	}
	return 0;

err_deinit_output:
	zhixu_xpad_deinit_output(zhixu_xpad);
err_free_in_urb:
	usb_free_urb(zhixu_xpad->irq_in);
err_free_idata:
	usb_free_coherent(udev, XPAD_PKT_LEN, zhixu_xpad->idata, zhixu_xpad->idata_dma);
err_free_mem:
	kfree(zhixu_xpad);
	return error;
}

static void zhixu_xpad_disconnect(struct usb_interface *intf)
{
	struct usb_zhixu_xpad *zhixu_xpad = usb_get_intfdata(intf);

	pr_info("%s %d\n",__func__,__LINE__); 
 
	if (zhixu_xpad->xtype == XTYPE_XBOX360W)
		zhixu_xpad360w_stop_input(zhixu_xpad); 
	
	zhixu_xpad_deinit_input(zhixu_xpad);

	/*
	 * Now that both input device and LED device are gone we can
	 * stop output URB.
	 */
	zhixu_xpad_stop_output(zhixu_xpad);

	zhixu_xpad_deinit_output(zhixu_xpad);
	
	if (!IS_ERR_OR_NULL(zhixu_xpad->thread_vibrator_left)) {
		pr_info("%s %d\n",__func__,__LINE__); 
		kthread_stop(zhixu_xpad->thread_vibrator_left);
		zhixu_xpad->thread_vibrator_left = NULL;
	}
	
	pr_info("%s %d\n",__func__,__LINE__); 
	
	if (!IS_ERR_OR_NULL(zhixu_xpad->thread_vibrator_right)) {
		pr_info("%s %d\n",__func__,__LINE__); 
		kthread_stop(zhixu_xpad->thread_vibrator_right);
		zhixu_xpad->thread_vibrator_right = NULL;
	} 
	
#if 0
	/*Ysjie modify*/
	//flush_work(&zhixu_xpad->vibrator_work); 
	/* 等待工作完成并删除它们 */
	flush_workqueue(zhixu_xpad->vibrator_left_workqueue);
	flush_workqueue(zhixu_xpad->vibrator_right_workqueue);
	
	/* 取消和销毁workqueue */
	//spin_lock(&vibrator_workqueue->lock);
	cancel_work_sync(&zhixu_xpad->vibrator_left_work);  // 在禁用队列之前取消工作项
	cancel_work_sync(&zhixu_xpad->vibrator_right_work);  // 在禁用队列之前取消工作项
	//spin_unlock(&vibrator_workqueue->lock);
	
	flush_workqueue(zhixu_xpad->vibrator_left_workqueue);  // 再次等待工作项完成并从队列中删除
	destroy_workqueue(zhixu_xpad->vibrator_left_workqueue);
	
	flush_workqueue(zhixu_xpad->vibrator_right_workqueue);  // 再次等待工作项完成并从队列中删除
	destroy_workqueue(zhixu_xpad->vibrator_right_workqueue);
	/*end modify*/
#endif	
	usb_free_urb(zhixu_xpad->irq_in);
	usb_free_coherent(zhixu_xpad->udev, XPAD_PKT_LEN,
			zhixu_xpad->idata, zhixu_xpad->idata_dma);

	kfree(zhixu_xpad);

	usb_set_intfdata(intf, NULL);
	
	 
}

static int zhixu_xpad_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_zhixu_xpad *zhixu_xpad = usb_get_intfdata(intf);
	struct input_dev *input = zhixu_xpad->dev;

	if (zhixu_xpad->xtype == XTYPE_XBOX360W) {
		/*
		 * Wireless controllers always listen to input so
		 * they are notified when controller shows up
		 * or goes away.
		 */
		zhixu_xpad360w_stop_input(zhixu_xpad);

		/*
		 * The wireless adapter is going off now, so the
		 * gamepads are going to become disconnected.
		 * Unless explicitly disabled, power them down
		 * so they don't just sit there flashing.
		 */
		if (auto_poweroff && zhixu_xpad->pad_present)
			zhixu_xpad360w_poweroff_controller(zhixu_xpad);
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			zhixu_xpad_stop_input(zhixu_xpad);
		mutex_unlock(&input->mutex);
	}

	zhixu_xpad_stop_output(zhixu_xpad);

	return 0;
}

static int zhixu_xpad_resume(struct usb_interface *intf)
{
	struct usb_zhixu_xpad *zhixu_xpad = usb_get_intfdata(intf);
	struct input_dev *input = zhixu_xpad->dev;
	int retval = 0;

	if (zhixu_xpad->xtype == XTYPE_XBOX360W) {
		retval = zhixu_xpad360w_start_input(zhixu_xpad);
	} else {
		mutex_lock(&input->mutex);
		if (input->users) {
			retval = zhixu_xpad_start_input(zhixu_xpad);
		} else if (zhixu_xpad->xtype == XTYPE_XBOXONE) {
			/*
			 * Even if there are no users, we'll send Xbox One pads
			 * the startup sequence so they don't sit there and
			 * blink until somebody opens the input device again.
			 */
			retval = zhixu_xpad_start_xbox_one(zhixu_xpad);
		}
		mutex_unlock(&input->mutex);
	}

	return retval;
}

static struct usb_driver zhixu_xpad_driver = {
	.name		= "zhixu_xpad",
	.probe		= zhixu_xpad_probe,
	.disconnect	= zhixu_xpad_disconnect,
	.suspend	= zhixu_xpad_suspend,
	.resume		= zhixu_xpad_resume,
	.reset_resume	= zhixu_xpad_resume,
	.id_table	= zhixu_xpad_table,
};

module_usb_driver(zhixu_xpad_driver);

MODULE_AUTHOR("Marko Friedemann <mfr@bmx-chemnitz.de>");
MODULE_DESCRIPTION("X-Box pad driver");
MODULE_LICENSE("GPL");
