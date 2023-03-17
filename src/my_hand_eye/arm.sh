echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="ft_servo"' >/etc/udev/rules.d/ft_servo.rules
echo 'KERNEL=="video*", ATTR{index}=="0", ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9230", MODE:="0777", GROUP:="dialout", SYMLINK+="eye"' >/etc/udev/rules.d/eye.rules
service udev reload
sleep 2
service udev restart