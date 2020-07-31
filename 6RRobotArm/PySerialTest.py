import serial
import time

serialcomm = serial.Serial('COM7', 115200)
serialcomm.bytesize = serial.EIGHTBITS
serialcomm.parity = serial.PARITY_NONE
serialcomm.stopbits = serial.STOPBITS_ONE
serialcomm.timeout = 1
time.sleep(2)

'''
while True:
    i = input("input(on/off): ").strip()
    if i == 'done':
        print('finished program')
        break
    serialcomm.write((i+'\n').encode())
    print((i + '\n').encode())
    time.sleep(0.2)
    print(serialcomm.readline().decode('ascii'))
serialcomm.close()
'''
t = (360 * 7680) / 360
print(t)
print(str(int(round(t))))
def mvmt():
    M6 = '#MTR6' + str(int(((360 * 7680) / 360)))
    serialcomm.write((M6 + '\n').encode())

    print(serialcomm.readline().decode('ascii'))

    print(serialcomm.readline().decode('ascii'))

    print(serialcomm.readline().decode('ascii'))


    i = '#EXEC'
    serialcomm.write((i + '\n').encode())

mvmt()