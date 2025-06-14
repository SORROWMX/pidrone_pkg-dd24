# Инструменты отладки для дрона PiDrone

В этом документе описаны инструменты отладки для системы управления дроном PiDrone, использующей протокол MSP (MultiWii Serial Protocol). Эти инструменты помогут вам диагностировать проблемы в системе управления, проверить правильность отправки команд и анализировать поведение полетного контроллера.

## Общий обзор инструментов

1. **debug_msp_commands.py** - Инструмент для отправки и тестирования MSP команд
2. **debug_command_order.py** - Тестирование порядка команд для определения правильного маппинга каналов
3. **inspect_command_values.py** - Проверка и модификация значений команд в файле command_values.py
4. **debug_msp_protocol.py** - Низкоуровневый анализ протокола MSP
5. **debug_flight_controller.py** - Отладка взаимодействия с полетным контроллером
6. **debug_msp_offboard.py** - Тестирование системы MSP Offboard

Все инструменты поддерживают:
- Вывод данных в ROS топики
- Сохранение логов в файлы
- Экспорт данных в JSON формате

## Установка и подготовка

Перед использованием инструментов убедитесь, что:
1. У вас установлен ROS и пакет pidrone_pkg
2. Полетный контроллер подключен к Raspberry Pi через USB
3. Создана директория для логов

```bash
# Создание директории для логов
mkdir -p ~/logs
```

## 1. debug_msp_commands.py

Этот инструмент позволяет отправлять команды на полетный контроллер и мониторить его ответы.

### Основные функции:
- Отправка предустановленных команд (arm, disarm, idle)
- Отправка пользовательских команд с указанными значениями
- Мониторинг RC каналов и данных о положении дрона
- Публикация данных в ROS топики

### Использование:

```bash
# Отправка команды arm и мониторинг RC каналов
python scripts/debug_msp_commands.py --command arm --monitor --log-file --ros

# Отправка пользовательской команды
python scripts/debug_msp_commands.py --command custom --roll 1700 --pitch 1500 --yaw 1500 --throttle 1500 --monitor

# Экспорт логов в файл
python scripts/debug_msp_commands.py --command idle --monitor --export msp_commands_log.json
```

### ROS топики:
- `/pidrone/msp_debug_data` - Данные отладки в формате JSON
- `/pidrone/msp_debug_log` - Сообщения о логировании

## 2. debug_command_order.py

Этот инструмент помогает определить правильный порядок каналов управления (roll, pitch, yaw, throttle).

### Основные функции:
- Тестирование стандартного порядка каналов
- Тестирование с перестановкой yaw и throttle
- Тестирование с перестановкой roll и pitch
- Тестирование всех перестановок каналов

### Использование:

```bash
# Тестирование стандартного порядка каналов
python scripts/debug_command_order.py --test standard

# Тестирование с перестановкой yaw и throttle
python scripts/debug_command_order.py --test swap_yaw_throttle

# Тестирование всех перестановок
python scripts/debug_command_order.py --test all_permutations
```

## 3. inspect_command_values.py

Этот инструмент позволяет проверять и модифицировать значения команд в файле command_values.py.

### Основные функции:
- Отображение текущих значений команд
- Перестановка каналов yaw и throttle
- Восстановление из резервной копии
- Обновление порядка каналов

### Использование:

```bash
# Отображение текущих значений команд
python scripts/inspect_command_values.py --action show

# Перестановка каналов yaw и throttle
python scripts/inspect_command_values.py --action swap_yt

# Восстановление из резервной копии
python scripts/inspect_command_values.py --action restore

# Обновление порядка каналов
python scripts/inspect_command_values.py --action update --order throttle yaw roll pitch
```

## 4. debug_msp_protocol.py

Низкоуровневый инструмент для анализа протокола MSP.

### Основные функции:
- Мониторинг сырого обмена данными между Raspberry Pi и полетным контроллером
- Анализ пакетов MSP
- Тестирование порядка команд
- Публикация данных в ROS топики

### Использование:

```bash
# Мониторинг коммуникации
python scripts/debug_msp_protocol.py --action monitor --duration 30 --log-file --ros

# Тестирование порядка команд
python scripts/debug_msp_protocol.py --action test_order --log-file

# Выполнение всех действий
python scripts/debug_msp_protocol.py --action all --duration 20 --export msp_protocol_log.json
```

### ROS топики:
- `/pidrone/msp_protocol_debug` - Данные отладки протокола
- `/pidrone/msp_protocol_log` - Сообщения о логировании
- `/pidrone/msp_packets` - Данные о пакетах MSP

## 5. debug_flight_controller.py

Инструмент для отладки взаимодействия с полетным контроллером.

### Основные функции:
- Мониторинг данных от полетного контроллера
- Проверка порядка команд
- Тестирование каналов управления
- Создание моста ROS для отладки

### Использование:

```bash
# Мониторинг полетного контроллера
python scripts/debug_flight_controller.py --action monitor --duration 30 --log-file

# Проверка порядка команд
python scripts/debug_flight_controller.py --action verify_order --log-file

# Тестирование каналов
python scripts/debug_flight_controller.py --action test_channels --log-file

# Создание моста ROS для отладки
python scripts/debug_flight_controller.py --action ros_bridge --log-file
```

### ROS топики:
- `/pidrone/imu_debug` - Данные IMU
- `/pidrone/mode_debug` - Информация о режиме
- `/pidrone/rc_debug` - Данные RC каналов
- `/pidrone/debug_data` - Данные отладки
- `/pidrone/debug_log` - Сообщения о логировании

## 6. debug_msp_offboard.py

Инструмент для тестирования системы MSP Offboard.

### Основные функции:
- Тестирование взлета и посадки
- Тестирование полета по квадрату
- Тестирование управления скоростью
- Публикация данных в ROS топики

### Использование:

```bash
# Тестирование взлета и посадки
python scripts/debug_msp_offboard.py --test takeoff_land --log-file

# Тестирование полета по квадрату
python scripts/debug_msp_offboard.py --test square --height 0.3 --side 0.5 --log-file

# Тестирование управления скоростью
python scripts/debug_msp_offboard.py --test velocity --speed 0.2 --duration 2.0 --log-file

# Выполнение всех тестов
python scripts/debug_msp_offboard.py --test all --log-file --export offboard_log.json
```

### ROS топики:
- `/pidrone/offboard_debug` - Данные отладки Offboard
- `/pidrone/offboard_log` - Сообщения о логировании

## Анализ логов

Все инструменты могут сохранять логи в файлы в формате JSON или в специальном формате для последующего анализа.

### Структура лог-файла:
```
# Flight Controller Debug Log
# Started: 2023-06-15 14:30:45
# Format: timestamp,type,data
1686837045.123,attitude,{"roll":0.5,"pitch":-0.2,"yaw":45.3}
1686837045.456,command,{"command":[1500,1500,1500,1500,1900,1000,1000,1000],"description":"IDLE"}
```

### Просмотр логов:

```bash
# Просмотр последних записей лога
tail -n 20 logs/fc_debug_20230615-143045.log

# Извлечение данных о командах из лога
grep "command" logs/fc_debug_20230615-143045.log

# Конвертация лога в JSON формат (требует jq)
cat logs/fc_debug_20230615-143045.log | grep -v "^#" | jq -s 'map({timestamp: .[0], type: .[1], data: .[2] | fromjson})'
```

## Решение проблем с порядком команд

Если вы обнаружили, что каналы yaw и throttle перепутаны, вы можете исправить это следующим образом:

1. Проверьте текущий порядок команд:
```bash
python scripts/debug_command_order.py --test all_permutations
```

2. Проверьте текущие значения в command_values.py:
```bash
python scripts/inspect_command_values.py --action show
```

3. Поменяйте местами каналы yaw и throttle:
```bash
python scripts/inspect_command_values.py --action swap_yt
```

4. Проверьте, что изменения применились:
```bash
python scripts/inspect_command_values.py --action show
```

5. Протестируйте новый порядок команд:
```bash
python scripts/debug_command_order.py --test standard
```

6. Если что-то пошло не так, восстановите из резервной копии:
```bash
python scripts/inspect_command_values.py --action restore
```

## Примеры использования

### Пример 1: Отладка проблемы с управлением

```bash
# 1. Проверьте порядок команд
python scripts/debug_command_order.py --test all_permutations --log-file

# 2. Проанализируйте протокол MSP
python scripts/debug_msp_protocol.py --action monitor --duration 20 --log-file --ros

# 3. Проверьте значения в command_values.py
python scripts/inspect_command_values.py --action show

# 4. Если необходимо, обновите порядок каналов
python scripts/inspect_command_values.py --action update --order roll pitch throttle yaw

# 5. Протестируйте изменения
python scripts/debug_msp_commands.py --command custom --roll 1700 --pitch 1500 --yaw 1500 --throttle 1500 --monitor --log-file
```

### Пример 2: Тестирование полетной системы

```bash
# 1. Запустите мост ROS для отладки
python scripts/debug_flight_controller.py --action ros_bridge --log-file

# 2. В другом терминале запустите тест взлета и посадки
python scripts/debug_msp_offboard.py --test takeoff_land --log-file

# 3. Проанализируйте логи
cat logs/fc_debug_*.log | grep "attitude" > attitude_data.txt
cat logs/msp_offboard_*.log | grep "command" > command_data.txt
```

## Дополнительные ресурсы

- [Документация по протоколу MSP](https://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol)
- [Документация по ROS](http://wiki.ros.org/)
- [Репозиторий pidrone_pkg](https://github.com/h2r/pidrone_pkg)

## Устранение неполадок

### Не удается подключиться к полетному контроллеру

- Проверьте, что полетный контроллер подключен к Raspberry Pi через USB
- Убедитесь, что у вас есть права на доступ к устройству:
  ```bash
  sudo chmod 666 /dev/ttyACM0
  ```
- Проверьте, что устройство определяется системой:
  ```bash
  ls -l /dev/ttyACM*
  ```

### Ошибки ROS

- Убедитесь, что roscore запущен:
  ```bash
  roscore
  ```
- Проверьте, что переменные окружения ROS настроены правильно:
  ```bash
  echo $ROS_MASTER_URI
  echo $ROS_IP
  ```

### Проблемы с логированием

- Убедитесь, что директория для логов существует:
  ```bash
  mkdir -p logs
  ```
- Проверьте права доступа к директории:
  ```bash
  chmod 755 logs
  ``` 