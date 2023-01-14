echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="ft_servo"' >/etc/udev/rules.d/ft_servo.rules
echo 'KERNEL=="video*", ATTRS{idVendor}=="0801", ATTRS{idProduct}=="0101", ATTRS{serial}=="200901010001", MODE:="0777", GROUP:="dialout", SYMLINK+="eye"' >/etc/udev/rules.d/eye.rules
service udev reload
sleep 2
service udev restart