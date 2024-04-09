import serial
import struct
import sys
import keyboard
import csv
import os
import time
from datetime import datetime

SAVE_DIRECTORY = r'C:\ITMO_2024_WORK\ITMO_Qt_arduino\PlayGround_Sketches\Test_Serial_port_connection\test_send_receive_py\logs'
port = 'COM3' 
baud_rate = 250_000

PWM_INDEX = 2
ANGLE_INDEX = 0
ANGLE_FILT_INDEX = 1
DT_INDEX = 4
INTER_INDEX = 3 

# Инициализация массивов для каждого значения
pwmData = []
angleData = []
angle_filtData = []
dt_Data = []
inter_Data = []

def print_with_precision(values, precision):
    print("Rec:", [round(value, precision) for value in values])

def send_command(command, ser):
    ser.write(command.encode())

def stop_receive(ser):
    ser.close()
    print("Программа завершена")
    save_data_to_csv()
    sys.exit()

def save_data_to_csv():
    current_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"data_{current_datetime}.csv"
    filename = os.path.join(SAVE_DIRECTORY, filename)

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["pwmData", "angleData", "angle_filtData", "dt_Data", "inter_Data"])
        for i in range(len(pwmData)):
            writer.writerow([pwmData[i], angleData[i], angle_filtData[i], dt_Data[i], inter_Data[i]])


PANIC_BUTTON1 = '9'
PANIC_BUTTON2 = '0'
START_BUTTON = '2'
STOP_BUTTON = '1'
RESET_PID_BUTTON = '3'
QUIT_BUTTON = 'q'

if __name__ == "__main__":
    try:
        ser = serial.Serial(port, baud_rate, write_timeout = 1)
        print(f"Успешно подключено к порту {port} на скорости {baud_rate} бод")
    except serial.SerialException as e:
        print(f"Ошибка подключения к порту {port}: {e}")
        exit()

    
    while True:
        if ser.in_waiting:
            raw_data = ser.read(20)
            if len(raw_data) != 20:
                print("--- wrong receive ---")
                continue

            floats = struct.unpack('5f', raw_data)
            pwmData.append(floats[PWM_INDEX])
            angleData.append(floats[ANGLE_INDEX])
            angle_filtData.append(floats[ANGLE_FILT_INDEX])
            dt_Data.append(floats[DT_INDEX])
            inter_Data.append(floats[INTER_INDEX])
            print_with_precision(floats, 2)

        # Обработка нажатия клавиши
        if keyboard.is_pressed(STOP_BUTTON):
            print("Stop button")
            ser.write(b'1')
            time.sleep(0.5)
        elif keyboard.is_pressed(START_BUTTON):
            print("Start")
            ser.write(START_BUTTON.encode())
            time.sleep(0.5)
        elif keyboard.is_pressed('3'):
            send_command('3', ser)
        elif keyboard.is_pressed('4'):
            send_command('4', ser)
        elif keyboard.is_pressed('0'):
            send_command('0', ser)
        elif keyboard.is_pressed('9'):
            send_command('9', ser)
        elif keyboard.is_pressed(QUIT_BUTTON):
            stop_receive(ser)
            
        