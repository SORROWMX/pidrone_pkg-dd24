# Изменения для совместимости с Python 2.7 для ROS Kinetic

## Выполненные изменения

Для обеспечения совместимости с Python 2.7, используемым в ROS Kinetic, были выполнены следующие изменения:

1. Заменены все f-строки (форматированные строки с префиксом `f`) на формат `.format()`:
   - В файле `h2rMultiWii.py`: 
     ```python
     # Было
     print(f"Failed to get data on attempt {retry+1}, retrying...")
     
     # Стало
     print("Failed to get data on attempt {}, retrying...".format(retry+1))
     ```
     
   - В файле `flight_controller_node.py`: 
     ```python
     # Было
     print(f"Error updating IMU message: {e}")
     
     # Стало
     print("Error updating IMU message: {}".format(e))
     ```

2. Заменены все %-форматирование с f-строками на метод `.format()`:
   ```python
   # Было
   print("Timeout on receiveDataPacket after %d attempts" % max_attempts)
   
   # Стало
   print("Timeout on receiveDataPacket after {} attempts".format(max_attempts))
   ```

## Почему это было необходимо

f-строки были введены только в Python 3.6 и не поддерживаются в Python 2.7. Поскольку ROS Kinetic работает на Python 2.7, использование f-строк приводило к синтаксическим ошибкам при запуске скриптов:

```
SyntaxError: invalid syntax
```

## Затронутые файлы

- `scripts/h2rMultiWii.py`
- `scripts/flight_controller_node.py`

## Проверка файлов на совместимость с Python 2.7

Для проверки совместимости файлов с Python 2.7 можно использовать следующие команды:

```bash
python -m py_compile scripts/h2rMultiWii.py
python -m py_compile scripts/flight_controller_node.py
python -m py_compile scripts/msp_offboard.py
```

Если команда завершается без ошибок, файл совместим с Python 2.7. 