#CH9102，虚拟串口设置别名为wheeltec_mic_virtual
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2208", ATTRS{idProduct}=="0001",MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_mic_virtual"' >>/etc/udev/rules.d/wheeltec_mic.rules
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="2208", ATTRS{idProduct}=="0001",MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_mic_virtual"' >>/etc/udev/rules.d/wheeltec_mic.rules

service udev reload
sleep 2
service udev restart
