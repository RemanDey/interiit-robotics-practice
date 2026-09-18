analyzing hte problem the main INVARIANT here is the PHYSICAL LOCATION OF THE USB CONNECTION: whatever happens the PHYSICAL POSITION OF THE USB A Port doesnt change--so use it to control the other devices connected
My target: somehow identify the physical
1) Make a Udev rule:
at first check the info of the device that we are connecting

```udevadm info -a -n /dev/ttyUSB0```

Look for properties under parent device blocks, such as ATTRS{idVendor}, ATTRS{idProduct}, ATTRS{serial}, or the kernel physical path KERNELS.

  NOW we need to create the udev rule by:

  ```sudo gedit /etc/udev/rules.d/99-robot-serial.rules``` or use nano if doing ssh or something else or whichever text editor u are using

  NOW we need to write the assignment logic, here i plan to use the physical location of the ports
  ```SUBSYSTEM=="tty", KERNELS=="1-2.1:1.0", SYMLINK+="robot/port_1_2", MODE="0666"```

  now we just need to update the usb devrules once 
```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
```
VERIFY ONCE BY TYPING 
```bash
ls -l /dev/robot/
```