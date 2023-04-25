echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="ft_servo"' >/etc/udev/rules.d/ft_servo.rules
echo 'KERNEL=="video*", ATTR{index}=="0", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="6368", MODE:="0777", GROUP:="dialout", SYMLINK+="eye"' >/etc/udev/rules.d/eye.rules
service udev reload
sleep 2
service udev restart