import serial
import struct
import sys
import keyboard

def print_with_precision(values, precision):
    print("Received values:", [round(value, precision) for value in values])

port = 'COM3' 
baud_rate = 250_000



try:
    ser = serial.Serial(port, baud_rate)
    print(f"Успешно подключено к порту {port} на скорости {baud_rate} бод")
except serial.SerialException as e:
    print(f"Ошибка подключения к порту {port}: {e}")
    exit()


# main loop
while True:
    raw_data = ser.read(20)
    
    if len(raw_data) != 20:
        print("IGNORE RECEIVE")
        continue
    
    floats = struct.unpack('fffff', raw_data)
    print_with_precision(floats, 5)
    if keyboard.is_pressed('q'):
        ser.close()  # Закрываем соединение с портом
        print("Программа завершена")
        sys.exit()