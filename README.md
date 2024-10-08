# shake_test


## Setup for Jetpack 6 to include serial drivers
https://forums.developer.nvidia.com/t/no-ttyusb-ttyacm-shown-after-installed-jetpack6-0/299191/13

```bash
head -n 1 /etc/nv_tegra_release
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v3.0/sources/public_sources.tbz2 # Or download https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v3.0/sources/public_sources.tbz2

cd ~/Downloads
tar xf public_sources.tbz2
cd Linux_for_Tegra/source
tar xf kernel_src.tbz2
cd kernel/kernel-jammy-src

cd ~/Downloads/Linux_for_Tegra/source/kernel/kernel-jammy-src
make defconfig
gedit .config

# Search for the CH341, uncomment it and enable it as module
# CONFIG_USB_SERIAL_CH341=m
# Save the file after finish editing.

make -j$(nproc) modules

cd ~/Downloads/Linux_for_Tegra/source/kernel/kernel-jammy-src/drivers/usb/serial/
sudo cp ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
sudo depmod -a

reboot

systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
systemctl stop brltty.service
systemctl disable brltty.service
sudo apt remove brltty
```
