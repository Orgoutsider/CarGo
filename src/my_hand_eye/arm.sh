echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="ft_servo"' >/etc/udev/rules.d/ft_servo.rules
echo 'KERNEL=="video*", ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9230", MODE:="0777", GROUP:="dialout", SYMLINK+="eye"' >/etc/udev/rules.d/eye.rules
echo 'KERNEL=="video*", ATTR{index}=="0", ATTRS{idVendor}=="0601", ATTRS{idProduct}=="0405", ATTRS{serial}=="200901010001", MODE:="0777", GROUP:="dialout", SYMLINK+="front_camera"' >/etc/udev/rules.d/front_camera.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/fdilink_ahrs.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", GROUP:="dialout", SYMLINK+="c_board"' >/etc/udev/rules.d/c_board.rules
service udev reload
sleep 2
service udev restart