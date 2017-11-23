## uvc-gadget

usb video class testing app

## Build  

- host:  
    make
- Cross compile:  
    make ARCH=arch CROSS_COMPILE=cross_compiler  
    eg:  
    make ARCH=arm CROSS_COMPILE=arm-hisiv600-linux-  
- or:  
    set ARCH, CROSS_COMPILE, KERNEL_DIR in Makefile

## Documentation

- [gadget-testing.txt](https://github.com/torvalds/linux/blob/master/Documentation/usb/gadget-testing.txt)
- [more doc](https://github.com/torvalds/linux/tree/master/Documentation/usb)

## Change log

- Add Readme/.gitignore and documentations  
  Copy linux-3.18.y/drivers/usb/gadget/function/uvc.h into repository, change include path for build

### Initial

- Fork(copy) from [uvc-gadget.git](http://git.ideasonboard.org/uvc-gadget.git)