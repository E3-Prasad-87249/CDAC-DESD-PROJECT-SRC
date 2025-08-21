#include <linux/module.h>
#include <linux/usb.h>
#include <linux/mutex.h>

#include "cdc_ioctl.h"
#include <linux/string.h>

/* Vendor and product ID of the STM32 USB CDC device */ 
#define VENDOR_ID       0x0483                              // STMicroelectronics (default)
#define PRODUCT_ID      0x5740                              // Default for STM32 USB CDC

// Endpoint addresses
#define BULK_IN_EP      0x81     // Endpoint 0x81 = EP1 IN (device to host)
#define BULK_OUT_EP     0x01     // Endpoint 0x01 = EP1 OUT (host to device)

#define DEVICE_NAME     "stm32cdc"  // Name for device node (/dev/stm32cdc)

// Global pointers to represent device, interface, and endpoints
static struct usb_device *device_addr;
static struct usb_interface *interface_addr;
static struct usb_endpoint_descriptor *ep_in_addr, *ep_out_addr;
static int major;
static struct class *cdc_class_addr;

static struct mutex lock;

// Open operation for device file
static int cdc_open(struct inode *inode_addr, struct file *file_addr)
{
    pr_info("%s: cdc_open() is called\n", THIS_MODULE->name);

    return 0;
}

// Release operation for device file
static int cdc_release(struct inode *inode_addr, struct file *file_addr)
{
    pr_info("%s: cdc_release() is called\n", THIS_MODULE->name);

    return 0;
}

// Read operation from USB device to user-space (via usb device's IN_EP)
// e.g. reading Die Temp senosor value here sent by our custom cdc device
static ssize_t cdc_read(struct file *file_addr, char __user *user_buf_addr, size_t len, loff_t *off_addr)
{
    int retval;
    int actual_len;
    uint8_t *local_buf_addr;

    pr_info("%s: cdc_read() is called\n", THIS_MODULE->name);

    // Check if device and endpoint are valid
    if (!device_addr || !ep_in_addr)
        return -ENODEV;

    // start of critical section of code
    // lock(mutex);
    // retval = mutex_lock_interruptible(&lock);
    // if(retval != 0) {
    //     pr_err("%s: in cdc_write() process unblocked due to sigal\n", THIS_MODULE->name);
    //     return retval;
    // }
    mutex_lock(&lock);
    pr_info("%s: mutex lock is acquired\n", THIS_MODULE->name);

    // Allocate dynamic buffer to read data from device
    local_buf_addr = kmalloc(len, GFP_KERNEL);
    if (!local_buf_addr)
        return -ENOMEM;

    // Perform bulk read from usb device's IN endpoint
    retval = usb_bulk_msg(device_addr,
                          usb_rcvbulkpipe(device_addr, ep_in_addr->bEndpointAddress),
                          local_buf_addr,
                          len,
                          &actual_len,
                          5000);  // Timeout in milliseconds
    pr_info("%s: local_buf content = %s\n", THIS_MODULE->name, local_buf_addr);

    if (retval) {
        kfree(local_buf_addr);
        pr_err("%s: usb_bulk_msg read failed with %d\n", THIS_MODULE->name, retval);
        return retval;
    }

    // Copy kernel buffer to user space
    if (copy_to_user(user_buf_addr, local_buf_addr, actual_len)) {
        kfree(local_buf_addr);
        return -EFAULT;
    }
    

    kfree(local_buf_addr);

    // end of critical section of code
     // unlock(mutex);
    mutex_unlock(&lock);
    pr_info("%s: mutex lock is released\n", THIS_MODULE->name);

    pr_info("%s: cdc_read() is done successfully\n", THIS_MODULE->name);
    return actual_len; // Return number of bytes read
}

// Write operation from user-space to USB device (via usb device's OUT_EP)
static ssize_t cdc_write(struct file *file_addr, const char __user *user_buf_addr, size_t len, loff_t *off_addr)
{
    int retval;
    int actual_len;
    uint8_t *local_buf_addr;

    pr_info("%s: cdc_write() is called\n", THIS_MODULE->name);

    // Check if device and endpoint are valid
    if (!device_addr || !ep_out_addr)
        return -ENODEV;

    // critical section of code
    // lock(mutex);
    //retval = mutex_lock_interruptible(&lock);
    // if(retval != 0) {
    //     pr_err("%s: in cdc_write() process unblocked due to sigal\n", THIS_MODULE->name);
    //     return retval;
    // }
    mutex_lock(&lock);
    pr_info("%s: mutex lock is acquired\n", THIS_MODULE->name);

    // Allocate buffer of size user_buf to write data
    local_buf_addr = kmalloc(len, GFP_KERNEL);
    if (!local_buf_addr)
        return -ENOMEM;

    // Copy data from user space to kernel space local buffer
    if (copy_from_user(local_buf_addr, user_buf_addr, len)) {
        kfree(local_buf_addr);
        return -EFAULT;
    }
    pr_info("%s: user_buf content = %s\n", THIS_MODULE->name, local_buf_addr);

    // Perform bulk write to usb device's OUT endpoint
    retval = usb_bulk_msg(device_addr,
                          usb_sndbulkpipe(device_addr, ep_out_addr->bEndpointAddress),
                          local_buf_addr,
                          len,
                          &actual_len,
                          1000);  // Timeout in milliseconds
    
    kfree(local_buf_addr);
    if (retval) {
        pr_err("%s: usb_bulk_msg write failed with %d\n", THIS_MODULE->name, retval);
        return retval;
    }

    // end of critical section
     // unlock(mutex);
    mutex_unlock(&lock);
    pr_info("%s: mutex lock is released\n", THIS_MODULE->name);

    pr_info("%s: cdc_write() is done successfully\n", THIS_MODULE->name);
    return actual_len; // Return number of bytes written
}

static long cdc_ioctl(struct file *pfile, unsigned int cmd, unsigned long param) 
{
    pr_info("%s: cdc_ioctl() is called\n", THIS_MODULE->name);
    devinfo_t info;
    int ret;
    //int actual_len;
    //uint8_t *tx_local_buf_addr = "L";
    //uint8_t *rx_local_buf_addr;
    //char rxBuf[10];

    // Allocate dynamic buffer to read data from device
    // rx_local_buf_addr = kmalloc(4, GFP_KERNEL);
    // if (!rx_local_buf_addr)
    //     return -ENOMEM;

    switch (cmd)
    {
    case DEV_GETINFO:
        pr_info("%s: cdc_ioctl() DEV_GETINFO CMD is triggered\n", THIS_MODULE->name);
        info.sGreenLed = 1;
        info.sOrangeLed = 1;
        info.sRedLed = 0;
        info.sBlueLed = 0;
         // Perform bulk write to OUT endpoint
        // ret = usb_bulk_msg(device_addr,
        //                   usb_sndbulkpipe(device_addr, ep_out_addr->bEndpointAddress),
        //                   tx_local_buf_addr,
        //                   strlen(tx_local_buf_addr),
        //                   &actual_len,
        //                   1000);  // Timeout in milliseconds
            
        // kfree(tx_local_buf_addr);
        // if (ret) {
        //     pr_err("%s: usb_bulk_msg write failed with %d\n", THIS_MODULE->name, ret);
        //     return ret;
        // }

        // // Perform bulk read from IN endpoint
        // ret = usb_bulk_msg(device_addr,
        //                   usb_rcvbulkpipe(device_addr, ep_in_addr->bEndpointAddress),
        //                   rxBuf,
        //                   strlen(rxBuf),
        //                   &actual_len,
        //                   5000);  // Timeout in milliseconds
        // pr_info("%s: rx_local_buf_addr = %s %d\n",THIS_MODULE->name, rxBuf, actual_len);
        // info.sGreenLed = rxBuf[0];
        // info.sOrangeLed = rxBuf[1];
        // info.sRedLed = rxBuf[2];
        // info.sBlueLed = rxBuf[3];
    
        ret = copy_to_user((void*)param, &info, sizeof(info));
        if(ret < 0) {
            pr_err("%s: copy_to_user() failed in pchar_ioctl().\n", THIS_MODULE->name);
            return ret;
        }
        pr_info("%s: pchar_ioctl() read dev current led stattus info.\n", THIS_MODULE->name);
        return 0;
    }

    pr_info("%s: invalid command in pchar_ioctl().\n", THIS_MODULE->name);    
    return -EINVAL; // invalid command
}


// File operations structure
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = cdc_open,
    .release = cdc_release,
    .read = cdc_read,
    .write = cdc_write,
    .unlocked_ioctl = cdc_ioctl,
};

// Called when USB device is plugged in and matched
static int cdc_probe(struct usb_interface *intf_addr, const struct usb_device_id *id_addr)
{
    int i;
    struct usb_host_interface *iface_desc_addr;

    pr_info("%s: cdc_probe() is called\n", THIS_MODULE->name);

    // Get device and interface pointer
    device_addr = interface_to_usbdev(intf_addr);
    interface_addr = intf_addr;
    iface_desc_addr = intf_addr->cur_altsetting;

    // Make sure this is a CDC Data interface
    if (iface_desc_addr->desc.bInterfaceClass != 0x0A) {
        pr_info("%s: skipping interface %d (not CDC Data)\n", THIS_MODULE->name, iface_desc_addr->desc.bInterfaceNumber);
        return -ENODEV;
    }
    // Search for bulk IN and OUT endpoints
    for (i = 0; i < iface_desc_addr->desc.bNumEndpoints; ++i) 
    {
        struct usb_endpoint_descriptor *ep_addr = &iface_desc_addr->endpoint[i].desc;

        if (usb_endpoint_is_bulk_in(ep_addr))
            ep_in_addr = ep_addr;
        else if (usb_endpoint_is_bulk_out(ep_addr))
            ep_out_addr = ep_addr;
    }

    // Ensure both endpoints were found
    if (!ep_in_addr || !ep_out_addr) {
        pr_err("%s: bulk endpoints not found\n", THIS_MODULE->name);
        return -ENODEV;
    }

    // Register char device
    major = register_chrdev(0, DEVICE_NAME, &fops);

    // Create device node under /dev/
    cdc_class_addr = class_create(DEVICE_NAME);
    device_create(cdc_class_addr, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);

    pr_info("%s: driver bound to Interface %d\n", THIS_MODULE->name, iface_desc_addr->desc.bInterfaceNumber);

    mutex_init(&lock);
    pr_info("%s: mutex_init() created mutex lock for stm32cdc device\n", THIS_MODULE->name);

    pr_info("%s: cdc_probe() is done successfully.\n", THIS_MODULE->name);

    return 0;
}

// Called when USB device is removed/unplugged
static void cdc_disconnect(struct usb_interface *intf_addr)
{
    pr_info("%s: cdc_disconnect() is called\n", THIS_MODULE->name);

    // Remove device node and unregister class/driver
    mutex_destroy(&lock);
    pr_info("%s: mutex_destroy() destroyed mutex lock for stm32cdc device\n", THIS_MODULE->name);

    device_destroy(cdc_class_addr, MKDEV(major, 0));
    class_destroy(cdc_class_addr);
    unregister_chrdev(major, DEVICE_NAME);
    pr_info("%s: cdc_disconnect() is done\n", THIS_MODULE->name);
}

// USB device ID table to match against
static struct usb_device_id cdc_table[] = {
    { USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
    {}  // Terminating entry
};
// It exports the device ID table (cdc_table) to the user space via the kernel's modinfo mechanism.
// This allows tools like udev and the kernel's module auto-loading system to know which devices your module supports, 
// so your driver can be automatically loaded when the device is plugged in.
MODULE_DEVICE_TABLE(usb, cdc_table);

// USB driver definition structure
static struct usb_driver cdc_driver = {
    .name = "stm32cdc_driver",
    .id_table = cdc_table,
    .probe = cdc_probe,
    .disconnect = cdc_disconnect,
};

// Register the driver with the USB core
module_usb_driver(cdc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Prasad Jadhav <prasad.jadhav@gmail.com>");
MODULE_DESCRIPTION("Custom Linux USB CDC Driver for STM32 CDC device");











// #include <linux/module.h>
// #include <linux/usb.h>
// #include <linux/fs.h>
// #include <linux/cdev.h>
// #include <linux/uaccess.h>
// #include <linux/slab.h>

// #define VENDOR_ID       0x0483   // STMicroelectronics (default)
// #define PRODUCT_ID      0x5740   // Default for STM32 USB CDC
// #define BULK_IN_EP      0x81    // Endpoint 0x81 = EP1 IN (device to host)
// #define BULK_OUT_EP     0x01    // Endpoint 0x01 = EP1 OUT (host to device)
// //#define BUF_SIZE        512
// #define DEVICE_NAME     "stm32cdc"

// static struct usb_device *device;
// static struct usb_interface *interface;
// static struct usb_endpoint_descriptor *ep_in, *ep_out;
// static int major;
// static struct class *cdc_class;

// static int cdc_open(struct inode *inode, struct file *file)
// {
//     pr_info("%s: cdc_open() is called\n", THIS_MODULE->name);
//     return 0;
// }

// static int cdc_release(struct inode *inode, struct file *file)
// {
//     pr_info("%s: cdc_release() is called\n", THIS_MODULE->name);
//     return 0;
// }

// static ssize_t cdc_read(struct file *file, char __user *user_buf, size_t len, loff_t *off)
// {
//     int retval;
//     int actual_len;
//     uint8_t *buf;

//     pr_info("%s: cdc_read() is called\n", THIS_MODULE->name);

//     if (!device || !ep_in)
//         return -ENODEV;

//     buf = kmalloc(len, GFP_KERNEL);
//     if (!buf)
//         return -ENOMEM;

//     retval = usb_bulk_msg(device,
//                           usb_rcvbulkpipe(device, ep_in->bEndpointAddress),
//                           buf, 
//                           len, 
//                           &actual_len, 
//                           5000);  // 1000ms timeout

//     if (retval) {
//         kfree(buf);
//         pr_err("%s: usb_bulk_msg read failed with %d\n", THIS_MODULE->name, retval); // usb_bulk_msg read failed with -110
//         return retval;
//     }

//     if (copy_to_user(user_buf, buf, actual_len)) {
//         kfree(buf);
//         return -EFAULT;
//     }

//     kfree(buf);
//     pr_info("%s: cdc_read() is exiting...\n", THIS_MODULE->name);
//     return actual_len;
// }

// static ssize_t cdc_write(struct file *file, const char __user *user_buf, size_t len, loff_t *off)
// {
//     int retval;
//     int actual_len;
//     uint8_t *buf;

//     pr_info("%s: cdc_write() is called\n", THIS_MODULE->name);

//     if (!device || !ep_out)
//         return -ENODEV;

//     buf = kmalloc(len, GFP_KERNEL);
//     if (!buf)
//         return -ENOMEM;

//     if (copy_from_user(buf, user_buf, len)) {
//         kfree(buf);
//         return -EFAULT;
//     }

//     retval = usb_bulk_msg(device,
//                           usb_sndbulkpipe(device, ep_out->bEndpointAddress),
//                           buf, 
//                           len, 
//                           &actual_len, 
//                           1000);  // 1000ms timeout

//     kfree(buf);
//     if (retval) {
//         pr_err("%s: usb_bulk_msg write failed with %d\n", THIS_MODULE->name, retval);
//         return retval;
//     }
//     pr_info("%s: cdc_write() is exiting...\n", THIS_MODULE->name);
//     return actual_len;
// }


// static struct file_operations fops = {
//     .owner = THIS_MODULE,
//     .open = cdc_open,
//     .release = cdc_release,
//     .read = cdc_read,
//     .write = cdc_write,
// };

// static int cdc_probe(struct usb_interface *intf, const struct usb_device_id *id)
// {
//     int i;
//     struct usb_host_interface *iface_desc;
//     pr_info("%s: cdc_probe() is called\n", THIS_MODULE->name);

//     device = interface_to_usbdev(intf);
//     interface = intf;

//     iface_desc = intf->cur_altsetting;

//     if (iface_desc->desc.bInterfaceClass != 0x0A) {
//         pr_info("STM32 CDC: skipping interface %d (not CDC Data)\n",iface_desc->desc.bInterfaceNumber);
//         return -ENODEV;
//     }

//     for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) 
//     {
//         struct usb_endpoint_descriptor *ep = &iface_desc->endpoint[i].desc;

//         if (usb_endpoint_is_bulk_in(ep))
//             ep_in = ep;
//         else if (usb_endpoint_is_bulk_out(ep))
//             ep_out = ep;
//     }

//     if (!ep_in || !ep_out) {
//         pr_err("STM32 CDC: bulk endpoints not found\n");
//         return -ENODEV;
//     }

//     major = register_chrdev(0, DEVICE_NAME, &fops);
//     cdc_class = class_create(DEVICE_NAME);
//     device_create(cdc_class, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);

//     pr_info("STM32 CDC: driver bound to Interface %d\n",iface_desc->desc.bInterfaceNumber);

//     pr_info("%s: cdc_probe() is exiting...\n", THIS_MODULE->name);
//     return 0;
// }

// static void cdc_disconnect(struct usb_interface *intf)
// {
//     pr_info("%s: cdc_disconnect() is called\n", THIS_MODULE->name);

//     device_destroy(cdc_class, MKDEV(major, 0));
//     class_destroy(cdc_class);
//     unregister_chrdev(major, DEVICE_NAME);
//     pr_info("STM32 CDC: driver disconnected\n");

//     pr_info("%s: cdc_disconnect() is exiting...\n", THIS_MODULE->name);
// }

// static struct usb_device_id cdc_table[] = {
//     { USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
//     {}
// };
// MODULE_DEVICE_TABLE(usb, cdc_table);

// static struct usb_driver cdc_driver = {
//     .name = "stm32cdc_driver",
//     .id_table = cdc_table,
//     .probe = cdc_probe,
//     .disconnect = cdc_disconnect,
// };
// module_usb_driver(cdc_driver);

// MODULE_LICENSE("GPL");
// MODULE_AUTHOR("You");
// MODULE_DESCRIPTION("Custom Linux USB CDC Driver for STM32");