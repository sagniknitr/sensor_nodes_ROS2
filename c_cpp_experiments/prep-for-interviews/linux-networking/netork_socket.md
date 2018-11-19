*   Like TCP/IP, you first need to open a socket for communicating over a
  CAN network. Since SocketCAN implements a new protocol family, you
  need to pass PF_CAN as the first argument to the socket(2) system
  call. Currently, there are two CAN protocols to choose from, the raw
  socket protocol and the broadcast manager (BCM). So to open a socket,
  you would write

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  and

    s = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);


* Device drivers have an associated major and minor number. For example, /dev/ram0 and /dev/null are associated with a driver with major number 1, and /dev/tty0 and /dev/ttyS0 are associated with a driver with major number 4. The major number is used by the kernel to identify the correct device driver when the device is accessed. The role of the minor number is device dependent, and is handled internally within the driver. You can see the major/minor number pair for each device if you perform a listing in the /dev directory.


* The Linux kernel provides a virtual file system called sysfs. By providing virtual files, sysfs is able to export information about various kernel sub-systems, hardware devices and associated device drivers from the kernels device model to user space.


