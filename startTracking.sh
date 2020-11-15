# 开启在usb连接的情况无人机可以起飞
cd /home/dji/WorkSpace/m210-2020-match/
./M210ConfigTool --usb-port /dev/ttyACM0 --config-file UserConfig.txt --usb-connected-flight on
# ./M210ConfigTool --usb-port /dev/ttyACM0 --config-file UserConfig.txt --simulation on --latitude 36.66835186138794 --longitude -117.1376226464041
# 切换到build目录下，执行程序
cd /home/dji/WorkSpace/m210-2020-match/
./build/bin/m210-2020-match UserConfig.txt /dev/ttyACM0
