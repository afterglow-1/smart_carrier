import json
import time
from pyb import UART
#python字典类型
obj = {
    "qrCode":"123-321",
    "color" :[1,2,3]
}

# UART 3, and baudrate.
uart = UART(3, 115200)
outQRFlag=True
while(True):
    if(outQRFlag):
        output_str =json.dumps(obj)
        uart.write(output_str+'\n')
        print('I send:',output_str)
        if (uart.any()):
            data = uart.read()  #将串口3读取的数据存入data
            print('you send:',data)
            dictData=json.loads(data)
            print(dictData)
            if(dictData["qrCode"]==obj["qrCode"]):
                outQRFlag=False
    time.sleep_ms(1000)
