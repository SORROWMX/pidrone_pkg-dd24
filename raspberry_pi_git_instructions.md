# Инструкция по работе с репозиторием на Raspberry Pi

## Первоначальная настройка

1. Подключитесь к Raspberry Pi через SSH:
   ```bash
   ssh duckiesky@192.168.42.1
   ```
   или
   ```bash
   ssh duckiesky@<nameofdrone>.local
   ```

2. Клонируйте репозиторий:
   ```bash
   cd ~
   git clone https://github.com/SORROWMX/pidrone_pkg-dd24.git
   ```

3. Перейдите в директорию проекта:
   ```bash
   cd pidrone_pkg-dd24
   ```

## Обновление кода на Raspberry Pi

### Вариант 1: Если вы редактируете код напрямую на Raspberry Pi

1. Внесите изменения в код
2. Добавьте изменения в индекс:
   ```bash
   git add .
   ```
3. Создайте коммит:
   ```bash
   git commit -m "Описание ваших изменений"
   ```
4. Отправьте изменения в репозиторий:
   ```bash
   git push origin main
   ```

### Вариант 2: Если вы редактируете код на другом компьютере

1. На вашем компьютере внесите изменения и отправьте их в репозиторий:
   ```bash
   git add .
   git commit -m "Описание ваших изменений"
   git push origin main
   ```

2. На Raspberry Pi получите последние изменения:
   ```bash
   cd ~/pidrone_pkg-dd24
   git pull
   ```

## Быстрое обновление и перезапуск дрона

1. Получите последние изменения:
   ```bash
   cd ~/pidrone_pkg-dd24
   git pull
   ```

2. Если вы изменили файл `rigid_transform_node.py` и хотите его перезапустить:
   ```bash
   # Найдите PID процесса
   ps aux | grep rigid_transform_node.py
   
   # Остановите процесс
   kill <PID>
   
   # Перезапустите процесс
   cd ~/pidrone_pkg-dd24/scripts
   python rigid_transform_node.py
   ```

3. Для перезапуска всех процессов:
   ```bash
   # Остановите текущую screen-сессию
   screen -X -S pidrone quit
   
   # Запустите заново
   cd ~/pidrone_pkg-dd24
   ./start_pidrone_code.sh
   ```

## Важно: исправление проблемы с взлетом

Если `rigid_transform_node.py` не запускается, дрон может улететь в потолок при взлете. Этот узел отвечает за определение позиции дрона с помощью визуальной одометрии.

1. Проверьте, что узел запущен:
   ```bash
   ps aux | grep rigid_transform_node
   ```

2. Если узел не запущен, запустите его вручную:
   ```bash
   cd ~/pidrone_pkg-dd24/scripts
   python rigid_transform_node.py
   ```

3. Исправьте файл `pi.screenrc`, добавив символ новой строки `\n` после команды запуска:
   ```bash
   # Откройте файл для редактирования
   nano ~/pidrone_pkg-dd24/pi.screenrc
   
   # Найдите строку:
   # stuff "python rigid_transform_node.py"
   
   # Замените на:
   # stuff "python rigid_transform_node.py\n"
   ```

4. Сохраните изменения и перезапустите screen-сессию.

## Проверка перед взлетом

Перед взлетом убедитесь, что:

1. Все необходимые узлы запущены
2. Топик `/pidrone/picamera/pose` активен:
   ```bash
   rostopic echo /pidrone/picamera/pose -n 1
   ```

3. Режим позиционного управления работает корректно:
   ```bash
   rostopic echo /pidrone/position_control -n 1
   ```

Если все проверки пройдены, дрон должен взлетать и удерживать позицию корректно. 