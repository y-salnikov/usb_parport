#!/usr/bin/env python
import usb,sys,os,time

def get_devices(vid,pid) :
		devices=[]
		buses = usb.busses()
		for bus in buses :
			for device in bus.devices :
				if device.idVendor == vid:
					if device.idProduct == pid :
						devices.append(device)
		return devices


def main():
	vid=0xFFFF
	pid=0x1026
	devs=get_devices(vid,pid)
	if len(devs)==0:
		print "No device"
		exit (3)
	device=devs[0]
	try:
		handle = device.open()
		handle.claimInterface(0)
	except usb.USBError, err:
			print >> sys.stderr, err
	else:
			pass
	handle.controlMsg(usb.TYPE_VENDOR | usb.RECIP_DEVICE | usb.ENDPOINT_OUT, 0xE0, [], value=0, index=0, timeout=100)
	handle.controlMsg(usb.TYPE_VENDOR | usb.RECIP_DEVICE | usb.ENDPOINT_OUT, 0xE1, [], value=0, index=0, timeout=100)
	handle.controlMsg(usb.TYPE_VENDOR | usb.RECIP_DEVICE | usb.ENDPOINT_OUT, 0xE2, [], value=0, index=0, timeout=100)
	while 1:
		print handle.controlMsg(usb.TYPE_VENDOR | usb.RECIP_DEVICE | usb.ENDPOINT_IN, 0xE1, 2, value=0, index=1, timeout=100)
		time.sleep(1)
	try:
		handle.reset()
		handle.releaseInterface()
	except Exception, err:
			print >> sys.stderr, err
	handle, device = None, None

main()



