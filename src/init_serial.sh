#!/bin/bash


if [ `whoami` = "root" ];then
  echo "当前是root用户, 开始设置串口访问权限并添加串口绑定规则"
else
  echo "非root用户, 请输入用户密码后重新执行脚本"
  sudo su
fi

while true
do
  echo -n "输入计算机用户名 ： "
  read user_name
  echo -n "用户名为 ： $user_name 确认回复： y ,重新输入回复 ： n ， 退出： alt+c -> "
  read comfirm
  if [ "$comfirm" == "y" ];then
    echo "开始设置"
    break
  elif [ "$comfirm" == "n" ];then
    echo -n "重新输入:"
  fi
done


sudo usermod -aG dialout $user_name

sudo echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666" GROUP:="dialout", SYMLINK+="px4_01"'>/etc/udev/rules.d/px4_01.rules

# sudo echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="uwb_02"'>/etc/udev/rules.d/uwb_02.rules

sudo echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="uwb_01"'>/etc/udev/rules.d/uwb_01.rules

echo '添加完成！重启服务'

service udev reload
sleep 2
service udev restart
sudo udevadm trigger 


echo '重启完成，配置成功,开始测试,请[重新拔插串口设备]'
while true
do
  echo -n "开始测试输入： y , 退出： alt+c -> "
  read comfirm
  if [ "$comfirm" == "y" ];then
    break
  fi
done

ls -l /dev/px4_01 ; ls -l /dev/uwb_01; #ls -l /dev/uwb_02
echo '只显示插入的设备，如果找不到对应设备，检查是否插入对应设备，或尝试重启'

exit 0

