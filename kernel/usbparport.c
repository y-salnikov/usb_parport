/*****************************************************************************/

/*	usbparport -- at90usb82 parport device.
 * 	based on
 *	uss720.c  --  uss720 USB Parport Cable.
 *
 *	Copyright (C) 1999, 2005, 2010
 *	    Thomas Sailer (t.sailer@alumni.ethz.ch)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  Based on parport_pc.c
 *
 *  History:
 *   0.1  04.08.1999  Created
 *   0.2  07.08.1999  Some fixes mainly suggested by Tim Waugh
 *		      Interrupt handling currently disabled because
 *		      usb_request_irq crashes somewhere within ohci.c
 *		      for no apparent reason (that is for me, anyway)
 *		      ECP currently untested
 *   0.3  10.08.1999  fixing merge errors
 *   0.4  13.08.1999  Added Vendor/Product ID of Brad Hard's cable
 *   0.5  20.09.1999  usb_control_msg wrapper used
 *        Nov01.2000  usb_device_table support by Adam J. Richter
 *        08.04.2001  Identify version on module load.  gb
 *   0.6  02.09.2005  Fix "scheduling in interrupt" problem by making save/restore
 *                    context asynchronous
 *
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/socket.h>
#include <linux/parport.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/kref.h>
#include <linux/slab.h>

/*
 * Version Information
 */
#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "y.salnikov"
#define DRIVER_DESC "USB Parport Cable driver for at90usb82"

/* --------------------------------------------------------------------- */

struct parport_at90_private {
	struct usb_device *usbdev;
	struct parport *pp;
	struct kref ref_count;
	__u16	reg;
	__u8	data_reg,ctrl_reg,st_reg;
	struct list_head asynclist;
	spinlock_t asynclock;
};

struct at90_async_request {
	struct parport_at90_private *priv;
	struct kref ref_count;
	struct list_head asynclist;
	struct completion compl;
	struct urb *urb;
	struct usb_ctrlrequest *dr;
	__u16	reg;
};

/* --------------------------------------------------------------------- */

static void destroy_priv(struct kref *kref)
{
	struct parport_at90_private *priv = container_of(kref, struct parport_at90_private, ref_count);

	dev_dbg(&priv->usbdev->dev, "destroying priv datastructure\n");
	usb_put_dev(priv->usbdev);
	kfree(priv);
}

static void destroy_async(struct kref *kref)
{
	struct at90_async_request *rq = container_of(kref, struct at90_async_request, ref_count);
	struct parport_at90_private *priv = rq->priv;
	unsigned long flags;

	if (likely(rq->urb))
		usb_free_urb(rq->urb);
	kfree(rq->dr);
	spin_lock_irqsave(&priv->asynclock, flags);
	list_del_init(&rq->asynclist);
	spin_unlock_irqrestore(&priv->asynclock, flags);
	kfree(rq);
	kref_put(&priv->ref_count, destroy_priv);
}

/* --------------------------------------------------------------------- */

static void async_complete(struct urb *urb)
{
	struct at90_async_request *rq;
	struct parport *pp;
	struct parport_at90_private *priv;
	int status = urb->status;

	rq = urb->context;
	priv = rq->priv;
	pp = priv->pp;
	if (status) {
		dev_err(&urb->dev->dev, "async_complete: urb error %d\n",
			status);
	} else if (rq->dr->bRequest == 0xe1) {
		priv->reg=rq->reg;
		/* if nAck interrupts are enabled and we have an interrupt, call the interrupt procedure */
		// not implemented
	}
	complete(&rq->compl);
	kref_put(&rq->ref_count, destroy_async);
}

static struct at90_async_request *submit_async_request(struct parport_at90_private *priv,
							 __u8 request, __u8 requesttype, __u16 value, __u16 index,
							 gfp_t mem_flags)
{
	struct usb_device *usbdev;
	struct at90_async_request *rq;
	unsigned long flags;
	int ret;

	if (!priv)
		return NULL;
	usbdev = priv->usbdev;
	if (!usbdev)
		return NULL;
	rq = kzalloc(sizeof(struct at90_async_request), mem_flags);
	if (!rq) {
		dev_err(&usbdev->dev, "submit_async_request out of memory\n");
		return NULL;
	}
	kref_init(&rq->ref_count);
	INIT_LIST_HEAD(&rq->asynclist);
	init_completion(&rq->compl);
	kref_get(&priv->ref_count);
	rq->priv = priv;
	rq->urb = usb_alloc_urb(0, mem_flags);
	if (!rq->urb) {
		kref_put(&rq->ref_count, destroy_async);
		dev_err(&usbdev->dev, "submit_async_request out of memory\n");
		return NULL;
	}
	rq->dr = kmalloc(sizeof(*rq->dr), mem_flags);
	if (!rq->dr) {
		kref_put(&rq->ref_count, destroy_async);
		return NULL;
	}
	rq->dr->bRequestType = requesttype;
	rq->dr->bRequest = request;
	rq->dr->wValue = cpu_to_le16(value);
	rq->dr->wIndex = cpu_to_le16(index);
	rq->dr->wLength = cpu_to_le16(((requesttype & 0x80) && request==0xe1) ? sizeof(rq->reg) : 0);
	usb_fill_control_urb(rq->urb, usbdev, (requesttype & 0x80) ? usb_rcvctrlpipe(usbdev, 0) : usb_sndctrlpipe(usbdev, 0),
			     (unsigned char *)rq->dr,
			      ((requesttype & 0x80) && (request==0xe1)) ? &rq->reg : NULL, ((requesttype & 0x80) && (request==0xe1)) ? 2 : 0, async_complete, rq);
	/* rq->urb->transfer_flags |= URB_ASYNC_UNLINK; */
	spin_lock_irqsave(&priv->asynclock, flags);
	list_add_tail(&rq->asynclist, &priv->asynclist);
	spin_unlock_irqrestore(&priv->asynclock, flags);
	kref_get(&rq->ref_count);
	ret = usb_submit_urb(rq->urb, mem_flags);
	if (!ret)
		return rq;
	destroy_async(&rq->ref_count);
	dev_err(&usbdev->dev, "submit_async_request submit_urb failed with %d\n", ret);
	return NULL;
}

static unsigned int kill_all_async_requests_priv(struct parport_at90_private *priv)
{
	struct at90_async_request *rq;
	unsigned long flags;
	unsigned int ret = 0;

	spin_lock_irqsave(&priv->asynclock, flags);
	list_for_each_entry(rq, &priv->asynclist, asynclist) {
		usb_unlink_urb(rq->urb);
		ret++;
	}
	spin_unlock_irqrestore(&priv->asynclock, flags);
	return ret;
}

/* --------------------------------------------------------------------- */

static int get_1284_register(struct parport *pp, unsigned char reg, unsigned char *val, gfp_t mem_flags)
{
	struct parport_at90_private *priv;
	struct at90_async_request *rq;
	
	int ret;

	if (!pp)
		return -EIO;
	priv = pp->private_data;
	rq = submit_async_request(priv, 0xe1, 0xc0, 0, ((unsigned int)reg), mem_flags);
	if (!rq) {
		dev_err(&priv->usbdev->dev, "get_1284_register(%u) failed",
			(unsigned int)reg);
		return -EIO;
	}
	if (!val) {
		kref_put(&rq->ref_count, destroy_async);
		return 0;
	}
	if (wait_for_completion_timeout(&rq->compl, HZ)) {
		ret = rq->urb->status;
		*val = priv->reg;
		if (ret)
			printk(KERN_WARNING "get_1284_register: "
			       "usb error %d\n", ret);
		kref_put(&rq->ref_count, destroy_async);
		return ret;
	}
	printk(KERN_WARNING "get_1284_register timeout\n");
	kill_all_async_requests_priv(priv);
	return -EIO;
}

static int set_1284_register(struct parport *pp, unsigned char reg, unsigned char val, gfp_t mem_flags)
{
	struct parport_at90_private *priv;
	struct at90_async_request *rq;

	if (!pp)
		return -EIO;
	priv = pp->private_data;
	rq = submit_async_request(priv, 0xe1, 0x40, val, (((unsigned int)reg)), mem_flags);
	if (!rq) {
		dev_err(&priv->usbdev->dev, "set_1284_register(%u,%u) failed",
			(unsigned int)reg, (unsigned int)val);
		return -EIO;
	}
	kref_put(&rq->ref_count, destroy_async);
	return 0;
}


static int at90_init_request(struct parport *pp, gfp_t mem_flags)
{
	struct parport_at90_private *priv;
	struct at90_async_request *rq;

	if (!pp)
		return -EIO;
	priv = pp->private_data;
	rq = submit_async_request(priv, 0xe0, 0x40, 0, 0, mem_flags);
	if (!rq) {
		dev_err(&priv->usbdev->dev, "at90usb82 init request failed");
		return -EIO;
	}
	kref_put(&rq->ref_count, destroy_async);
	return 0;
}


/* --------------------------------------------------------------------- */

/* ECR modes */
#define ECR_SPP 00
#define ECR_PS2 01
#define ECR_PPF 02
#define ECR_ECP 03
#define ECR_EPP 04



/*
 * Access functions.
 */


static void parport_at90_write_data(struct parport *pp, unsigned char d)
{
	set_1284_register(pp, 0, d, GFP_KERNEL);
}

static unsigned char parport_at90_read_data(struct parport *pp)
{
	unsigned char ret;

	if (get_1284_register(pp, 0, &ret, GFP_KERNEL))
		return 0;
	return ret;
}

static void parport_at90_write_control(struct parport *pp, unsigned char d)
{

	d = (d & 0xf) ^ 0x0b;		//invert 0,1 & 3 bits
	set_1284_register(pp, 2, d, GFP_KERNEL);
}

static unsigned char parport_at90_read_control(struct parport *pp)
{
	unsigned char reg;
	
	if(get_1284_register(pp,2,&reg, GFP_KERNEL))
		return 0;
	else return reg;
}

static unsigned char parport_at90_frob_control(struct parport *pp, unsigned char mask, unsigned char val)
{
	unsigned char d;

	mask &= 0x0f;
	val &= 0x0f;
	get_1284_register(pp,2,&d, GFP_KERNEL);
	d=((d & (~mask)) ^ val) ^ 0x0b;
	if (set_1284_register(pp, 2, d, GFP_KERNEL))
		return 0;
	return d & 0xf;
}

static unsigned char parport_at90_read_status(struct parport *pp)
{
	unsigned char ret;

	if (get_1284_register(pp, 1, &ret, GFP_KERNEL))
		return 0;
	return (ret & 0xf8) ^ 0x80;    // invert bit 7
}

static void parport_at90_disable_irq(struct parport *pp)
{
//	struct parport_at90_private *priv = pp->private_data;	
//	
//	dev_err(&priv->usbdev->dev, "IRQ not implemented");
	return;
}

static void parport_at90_enable_irq(struct parport *pp)
{
//	struct parport_at90_private *priv = pp->private_data;	
//	dev_err(&priv->usbdev->dev, "IRQ not implemented");
	return;
}

static void parport_at90_data_forward (struct parport *pp)
{
//	struct parport_at90_private *priv = pp->private_data;	
//	unsigned char d;
	//not implemented
}

static void parport_at90_data_reverse (struct parport *pp)
{
//	struct parport_at90_private *priv = pp->private_data;	
//	unsigned char d;

//	d = priv->reg[1] | 0x20;
//	if (set_1284_register(pp, 2, d, GFP_KERNEL))
//		return;
//	priv->reg[1] = d;
}

static void parport_at90_init_state(struct pardevice *dev, struct parport_state *s)
{
	s->u.pc.ctr = 0xc | (dev->irq_func ? 0x10 : 0x0);
	s->u.pc.ecr = 0x24;
	
}

static void parport_at90_save_state(struct parport *pp, struct parport_state *s)
{
	unsigned char d;
	get_1284_register(pp,2,&d,GFP_KERNEL);
	s->u.pc.ctr = d;
	s->u.pc.ecr = 0;
}

static void parport_at90_restore_state(struct parport *pp, struct parport_state *s)
{
	set_1284_register(pp, 2, s->u.pc.ctr, GFP_ATOMIC);
	get_1284_register(pp, 2, NULL, GFP_ATOMIC);
}

static size_t parport_at90_epp_read_data(struct parport *pp, void *buf, size_t length, int flags)
{
	struct parport_at90_private *priv = pp->private_data;	
	dev_err(&priv->usbdev->dev, "EPP mode not implemented");
	return 0;
}

static size_t parport_at90_epp_write_data(struct parport *pp, const void *buf, size_t length, int flags)
{

	struct parport_at90_private *priv = pp->private_data;
	dev_err(&priv->usbdev->dev, "EPP mode not implemented");
	return 0;
}

static size_t parport_at90_epp_read_addr(struct parport *pp, void *buf, size_t length, int flags)
{
	struct parport_at90_private *priv = pp->private_data;	
	dev_err(&priv->usbdev->dev, "EPP mode not implemented");
	return 0;
}

static size_t parport_at90_epp_write_addr(struct parport *pp, const void *buf, size_t length, int flags)
{
	struct parport_at90_private *priv = pp->private_data;	
	dev_err(&priv->usbdev->dev, "EPP mode not implemented");
	return 0;
}

static size_t parport_at90_ecp_write_data(struct parport *pp, const void *buffer, size_t len, int flags)
{
	struct parport_at90_private *priv = pp->private_data;
	dev_err(&priv->usbdev->dev, "ECP mode not implemented");
	return 0;
}

static size_t parport_at90_ecp_read_data(struct parport *pp, void *buffer, size_t len, int flags)
{
	struct parport_at90_private *priv = pp->private_data;
	dev_err(&priv->usbdev->dev, "ECP mode not implemented");
	return 0;
}

static size_t parport_at90_ecp_write_addr(struct parport *pp, const void *buffer, size_t len, int flags)
{
	struct parport_at90_private *priv = pp->private_data;
	dev_err(&priv->usbdev->dev, "ECP mode not implemented");
	return 0;
}

static size_t parport_at90_write_compat(struct parport *pp, const void *buffer, size_t len, int flags)
{
	struct parport_at90_private *priv = pp->private_data;
	dev_err(&priv->usbdev->dev, "COMPAT mode not implemented");
	return 0;
}

/* --------------------------------------------------------------------- */

static struct parport_operations parport_at90_ops = 
{
	.owner =		THIS_MODULE,
	.write_data =		parport_at90_write_data,
	.read_data =		parport_at90_read_data,

	.write_control =	parport_at90_write_control,
	.read_control =		parport_at90_read_control,
	.frob_control =		parport_at90_frob_control,

	.read_status =		parport_at90_read_status,

	.enable_irq =		parport_at90_enable_irq,
	.disable_irq =		parport_at90_disable_irq,

	.data_forward =		parport_at90_data_forward,
	.data_reverse =		parport_at90_data_reverse,

	.init_state =		parport_at90_init_state,
	.save_state =		parport_at90_save_state,
	.restore_state =	parport_at90_restore_state,

	.epp_write_data =	parport_at90_epp_write_data,
	.epp_read_data =	parport_at90_epp_read_data,
	.epp_write_addr =	parport_at90_epp_write_addr,
	.epp_read_addr =	parport_at90_epp_read_addr,

	.ecp_write_data =	parport_at90_ecp_write_data,
	.ecp_read_data =	parport_at90_ecp_read_data,
	.ecp_write_addr =	parport_at90_ecp_write_addr,

	.compat_write_data =	parport_at90_write_compat,
	.nibble_read_data =	parport_ieee1284_read_nibble,
	.byte_read_data =	parport_ieee1284_read_byte,
};

/* --------------------------------------------------------------------- */

static int at90_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct usb_device *usbdev = usb_get_dev(interface_to_usbdev(intf));
	struct usb_host_interface *interface;
	struct usb_host_endpoint *endpoint;
	struct parport_at90_private *priv;
	struct parport *pp;
	unsigned char reg;
	int i;

	dev_dbg(&intf->dev, "probe: vendor id 0x%x, device id 0x%x\n",
		le16_to_cpu(usbdev->descriptor.idVendor),
		le16_to_cpu(usbdev->descriptor.idProduct));
	i = usb_set_interface(usbdev, intf->altsetting->desc.bInterfaceNumber, 0);
	dev_dbg(&intf->dev, "set interface result %d\n", i);
	interface = intf->cur_altsetting;
	/*
	 * Allocate parport interface 
	 */
	priv = kzalloc(sizeof(struct parport_at90_private), GFP_KERNEL);
	if (!priv) {
		usb_put_dev(usbdev);
		return -ENOMEM;
	}
	priv->pp = NULL;
	priv->usbdev = usbdev;
	kref_init(&priv->ref_count);
	spin_lock_init(&priv->asynclock);
	INIT_LIST_HEAD(&priv->asynclist);
	pp = parport_register_port(0, PARPORT_IRQ_NONE, PARPORT_DMA_NONE, &parport_at90_ops);
	if (!pp) {
		printk(KERN_WARNING "at90: could not register parport\n");
		goto probe_abort;
	}

	priv->pp = pp;
	pp->private_data = priv;
	pp->modes = PARPORT_MODE_PCSPP ;

	at90_init_request(pp,GFP_KERNEL);

	/* debugging */
	get_1284_register(pp, 0, &reg, GFP_KERNEL);
	dev_dbg(&intf->dev, "reg: %7ph\n", &priv->reg);

	endpoint = &interface->endpoint[2];
	dev_dbg(&intf->dev, "epaddr %d interval %d\n",
		endpoint->desc.bEndpointAddress, endpoint->desc.bInterval);
	parport_announce_port(pp);

	usb_set_intfdata(intf, pp);
	return 0;

probe_abort:
	kill_all_async_requests_priv(priv);
	kref_put(&priv->ref_count, destroy_priv);
	return -ENODEV;
}

static void at90_disconnect(struct usb_interface *intf)
{
	struct parport *pp = usb_get_intfdata(intf);
	struct parport_at90_private *priv;
	struct usb_device *usbdev;

	dev_dbg(&intf->dev, "disconnect\n");
	usb_set_intfdata(intf, NULL);
	if (pp) {
		priv = pp->private_data;
		usbdev = priv->usbdev;
		priv->usbdev = NULL;
		priv->pp = NULL;
		dev_dbg(&intf->dev, "parport_remove_port\n");
		parport_remove_port(pp);
		parport_put_port(pp);
		kill_all_async_requests_priv(priv);
		kref_put(&priv->ref_count, destroy_priv);
	}
	dev_dbg(&intf->dev, "disconnect done\n");
}

/* table of cables that work through this driver */
static const struct usb_device_id at90_table[] = {
	{ USB_DEVICE(0xFFFF, 0x1026) },
	{ }						/* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, at90_table);


static struct usb_driver at90_driver = {
	.name =		"at90usb_parport",
	.probe =	at90_probe,
	.disconnect =	at90_disconnect,
	.id_table =	at90_table,
};

/* --------------------------------------------------------------------- */

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static int __init at90_init(void)
{
	int retval;
	retval = usb_register(&at90_driver);
	if (retval)
		goto out;

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");
	printk(KERN_INFO KBUILD_MODNAME ": NOTE: this is a special purpose "
	       "driver to allow nonstandard\n");
	printk(KERN_INFO KBUILD_MODNAME ": protocols (eg. bitbang) over "
	       "at90 usb to parallel cables\n");
	printk(KERN_INFO KBUILD_MODNAME ": If you just want to connect to a "
	       "printer, use usblp instead\n");
out:
	return retval;
}

static void __exit at90_cleanup(void)
{
	usb_deregister(&at90_driver);
}

module_init(at90_init);
module_exit(at90_cleanup);

/* --------------------------------------------------------------------- */
