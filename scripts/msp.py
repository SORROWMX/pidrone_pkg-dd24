#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import serial
import struct
import time
import sys
import termios
import tty
import select
import os

# MSP команды
MSP_SET_RAW_RC = 200
MSP_STATUS = 101
MSP_BOXNAMES = 116
MSP_RC = 105
MSP_ANALOG = 110

# Настройка порта
ser = None

def init_serial(port='/dev/ttyACM0', baud=115200):
    global ser
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print("Подключено к {0} на скорости {1}".format(port, baud))
        return True
    except Exception as e:
        print("Ошибка подключения: {0}".format(e))
        return False

def msp_send(cmd, data=None):
    if data is None:
        data = []
    size = len(data)
    checksum = 0
    
    # Заголовок
    packet = b'$M<'
    
    # Размер данных
    packet += struct.pack('<B', size)
    checksum ^= size
    
    # Команда
    packet += struct.pack('<B', cmd)
    checksum ^= cmd
    
    # Данные
    for i in range(size):
        packet += struct.pack('<B', data[i])
        checksum ^= data[i]
    
    # Контрольная сумма
    packet += struct.pack('<B', checksum)
    
    # Отправка
    ser.write(packet)
    
    # Чтение ответа
    header = ser.read(3)
    if len(header) != 3 or header != b'$M>':
        return None
    
    dsize = ord(ser.read(1))
    cmd_resp = ord(ser.read(1))
    data_resp = ser.read(dsize)
    checksum_resp = ord(ser.read(1))
    
    return (cmd_resp, data_resp)

def set_rc_channels(roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4):
    # Преобразование значений в байты
    data = struct.pack('<8H', roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4)
    data = list(bytearray(data))  # Преобразование в список байтов для Python 2
    return msp_send(MSP_SET_RAW_RC, data)

def get_status():
    resp = msp_send(MSP_STATUS)
    if resp and resp[0] == MSP_STATUS:
        data = resp[1]
        if len(data) >= 11:
            cycle_time = struct.unpack('<H', data[0:2])[0]
            i2c_errors = struct.unpack('<H', data[2:4])[0]
            sensors = struct.unpack('<H', data[4:6])[0]
            flags = struct.unpack('<I', data[6:10])[0]
            return {
                'cycle_time': cycle_time,
                'i2c_errors': i2c_errors,
                'sensors': sensors,
                'flags': flags
            }
    return None

def get_box_names():
    resp = msp_send(MSP_BOXNAMES)
    if resp and resp[0] == MSP_BOXNAMES:
        names = resp[1].decode('ascii').rstrip('\0').split(';')
        return names
    return []

def get_rc_channels():
    resp = msp_send(MSP_RC)
    if resp and resp[0] == MSP_RC:
        data = resp[1]
        channels = []
        for i in range(0, len(data), 2):
            if i+1 < len(data):
                channels.append(struct.unpack('<H', data[i:i+2])[0])
        return channels
    return []

def getch_nb():
    """Non-blocking getch"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/dev/ttyACM0'
    
    if not init_serial(port):
        return
    
    try:
        # Получение информации о полетном контроллере
        box_names = get_box_names()
        print("Доступные режимы: {0}".format(box_names))
        
        # Начальные значения каналов
        roll = 1500
        pitch = 1500
        yaw = 1500
        throttle = 1000
        aux1 = 1000  # ARM канал
        aux2 = 1000
        aux3 = 1000
        aux4 = 1000
        
        print("\nУправление:")
        print("A/a - Арм/Дизарм")
        print("Q/q - Выход с дизармингом")
        print("F - Выход без дизарминга (failsafe)")
        print("+/- - Увеличение/уменьшение газа на 25мкс")
        print("W/S - Увеличение/уменьшение питча")
        print("D/A - Увеличение/уменьшение крена")
        print("E/Q - Увеличение/уменьшение рыскания")
        
        armed = False
        running = True
        
        while running:
            # Отправка команд
            set_rc_channels(roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4)
            
            # Получение статуса
            status = get_status()
            
            # Очистка строки
            sys.stdout.write("\r" + " " * 80)
            sys.stdout.flush()
            
            # Вывод информации
            status_str = "\rСтатус: "
            if status:
                flags = status['flags']
                armed_status = "Armed" if flags & 1 else "Disarmed"
                status_str += "{0}, ".format(armed_status)
            
            status_str += "R:{0} P:{1} Y:{2} T:{3} A1:{4} A2:{5} A3:{6} A4:{7}".format(
                roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4)
            sys.stdout.write(status_str)
            sys.stdout.flush()
            
            # Обработка клавиш
            ch = getch_nb()
            if ch:
                if ch in ['Q', 'q']:
                    print("\nВыход с дизармингом...")
                    # Дизарм перед выходом
                    aux1 = 1000
                    set_rc_channels(roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4)
                    time.sleep(0.5)  # Даем время на дизарм
                    running = False
                elif ch == 'F':
                    print("\nВыход без дизарминга (failsafe)...")
                    running = False
                elif ch in ['A', 'a']:
                    armed = not armed
                    aux1 = 1800 if armed else 1000
                    print("\n{0}".format("Арм" if armed else "Дизарм"))
                elif ch == '+':
                    throttle = min(throttle + 25, 2000)
                elif ch == '-':
                    throttle = max(throttle - 25, 1000)
                elif ch == 'W':
                    pitch = min(pitch + 50, 2000)
                elif ch == 'S':
                    pitch = max(pitch - 50, 1000)
                elif ch == 'D':
                    roll = min(roll + 50, 2000)
                elif ch == 'A':
                    roll = max(roll - 50, 1000)
                elif ch == 'E':
                    yaw = min(yaw + 50, 2000)
                elif ch == 'Q':
                    yaw = max(yaw - 50, 1000)
            
            time.sleep(0.1)  # 10Hz обновление
    
    except KeyboardInterrupt:
        print("\nПрервано пользователем")
    finally:
        if ser:
            ser.close()

if __name__ == "__main__":
    main()