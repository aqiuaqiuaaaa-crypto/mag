import serial
import serial.tools.list_ports
import time
# 获取所有串口设备实例。
# 如果没找到串口设备，则输出：“无串口设备。”
# 如果找到串口设备，则依次输出每个设备对应的串口号和描述信息。
ports_list = list(serial.tools.list_ports.comports())
if len(ports_list) <= 0:
    print("无串口设备。")
else:
    print("可用的串口设备如下：")
    for comport in ports_list:
        print(list(comport)[0], list(comport)[1])
    try:
        ser = serial.Serial('COM6'
                            , 115200, 8, 'N', 1)
        # data = ser.readline().decode().strip()
        while(True):
            #                            range(0-50)
            write_len = ser.write("a0:10,a1:00,a2:00,a3:00,a4:00,a5:00\r\n".encode('utf-8'))  # real_fre= fre//2
            time.sleep(1)
            data = ser.read_all()
            if data:
                rec_str = data.decode('utf-8')
                print(rec_str)
                a0 = 0.1 * (10 * int(rec_str[0]) + int(rec_str[1]))
                a1 = 0.1 * (10 * int(rec_str[2]) + int(rec_str[3]))
                a2 = 0.1 * (10 * int(rec_str[4]) + int(rec_str[5]))
                a3 = 0.1 * (10 * int(rec_str[6]) + int(rec_str[7]))
                a4 = 0.1 * (10 * int(rec_str[8]) + int(rec_str[9]))
                a5 = 0.1 * (10 * int(rec_str[10]) + int(rec_str[11]))
                print("a0:{1},a1:{1},a2:{1},a3:{1},a4:{1},a5:{1}".format(a0, a1, a2, a3, a4, a5))
            else:
                print("no data")
                break
    except:
        print("w")
