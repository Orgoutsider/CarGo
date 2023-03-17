echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002",MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001",MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar.rules
# echo 'KERNEL=="video*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07",MODE:="0777", GROUP:="dialout", SYMLINK+="d435"' >/etc/udev/rules.d/d435.rules
echo 'KERNEL=="video*", ATTR{index}=="0", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="6368",MODE:="0777", GROUP:="dialout", SYMLINK+="usb_cam"' >/etc/udev/rules.d/usb_cam.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/fdilink_ahrs.rules
service udev reload
sleep 2
service udev restart
