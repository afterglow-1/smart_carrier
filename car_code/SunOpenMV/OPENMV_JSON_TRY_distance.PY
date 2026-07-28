#请确认激光测距传感器已通电，需打开扩展版开关。否者报如下错误：OSError: [Error 110] ETIMEDOUT 最好try
from machine import I2C,Pin,Timer,UART #从 machine 模块导入 I2C、Pin子模块
from vl53l1x import VL53L1X #从 vl53l1x 模块导入 VL53L1X子模块
import json
import time
i2c = I2C(sda=Pin("P1"),scl=Pin("P9"),freq=400000)#I2C 初始化：sda--> P0, scl --> P2,频率 40MHz
distance = VL53L1X(i2c) #测距传感器初始化

#python字典类型
obj = {
    "qrCode":"123-321",
    "color" :[1,2,3],
    "distance":0
}

# UART 3, and baudrate.
uart = UART(3, 115200)
outQRFlag=True
while(True):
    obj["distance"] = distance.read()
    if(outQRFlag):
        output_str =json.dumps(obj)
        uart.write(output_str+'\n')
        print('I send:',output_str)
        if (uart.any()):
            data = uart.read()  #将串口3读取的数据存入data
            print('you send:',data)
            try:
                #数据能否解析为json
                dictData=json.loads(data)
            except  ValueError as e:
                print(e)
            else:
                print("没有出现异常")
                print(dictData)
                #if(dictData["qrCode"]==obj["qrCode"]):
                    #outQRFlag=False
    time.sleep_ms(1000)
