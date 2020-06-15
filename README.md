# rcraid-dkms
AMD RAIDXpert driver as DKMS package

Foreword:
===========

Many AMD mainboards for the AM4 socket based on the following chipsets come with RAID support:
 * X370
 * X399
 * X470
 * X570

But this RAID mode, which needs to be set in the BIOS, requires a specific driver for each OS.
There is a driver for Windows, but for Linux AMD provides either a binary blob or the sources.
When following the instructions, you will need to recompile the driver and install the kernel module on each kernel update and/or upgrade.
Since we are in the 21 century and we have software like DKMS, we don't need to do this manually, but let it happen automatically.

Goal
====
Therefore we try here to keep the code alive for many kernel versions as possible and deliver it within a PPA for Ubuntu as a DKMS package.

Installation
============
## Debian/Ubuntu
  * Driver package for Ubuntu: https://launchpad.net/~thopiekar/+archive/ubuntu/rcraid 
    ```bash
    sudo add-apt-repository ppa:thopiekar/rcraid
    sudo apt-get update
    sudo apt-get install rcraid-dkms
    ```
  * Switching to RAID mode:
    * Boot Linux in AHCI mode.
    * Append `modprobe.blacklist=ahci` to GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub
    * Run `sudo update-grub`
    * Restart
    * Switch to RAID mode
    * Boot your Linux installation from a RAID disk
    
## Arch/Manjaro
* Download and unpack zip or:

```bash
  git clone https://github.com/thopiekar/rcraid-dkms.git
```

* From inside "arch" directory:

```bash
  makepkg -si
```

* This will compile sources and install package for current running kernel. If you need 
to make package for different kernel then you can use KVERS option.
After installing edit /etc/mkinitcpio.conf and add
  
```bash
  MODULES=(rcraid?)
```

* Then rebuild your initrd f.e.:

```bash
  mkinitcpio -p linux56
```

**Note:** If you just planning on switching to rcraid, my advice is: don't. 
Support from AMD for Promontory raid on Linux is pretty much non-existent. 
You will be way better off sticking with mdadm, zfs or lvm. 

If you REALLY want this to dual-boot Linux/Win, also don't. Setting up virtualization 
with KVM with GPU Passtrough today is really simple, and you will not need to 
boot Windows directly on your hardware ever again.

**Sidenote:** If you decide to use rcraid regardless you might want to also install RAIDXpert. 
Web interface isn't working properly on Manjaro, but rcadm is more or less is.

**Sidenote 2:** If you ever decide to switch from rcraid to something else, you may find it useful 
to know that deleting arrays only removes metadata. Partitions are still there, and can be
recovered with gdisk, gpart etc. I found out that offset for partitions is 1069056 sectors (512B).
So if you have partition starting at usual 2048 it will be at 1071104 after array deletion.

Further info
============
  * https://thopiekar.eu/other/amd-raidxpress/

Thanks go to..
==============
  * To AMD to hand out the driver as source code at least. (Some history: Just think about Intel Poulsbo..)
    * https://www.amd.com/en/support/chipsets/amd-socket-am4/x370 - section Linux
  * To Martin Weber (@martinkarlweber) for sharing patches to make the driver work on Linux >= 4.15.x
    * https://github.com/martinkarlweber/rcraid-patches
