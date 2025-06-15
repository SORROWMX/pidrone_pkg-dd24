# Инструкции по установке прав на исполнение для скриптов

Для добавления прав на исполнение для отдельных скриптов:

```bash
# Перейдите в директорию с пакетом
cd ~/catkin_ws/src/pidrone_pkg/scripts

# Добавьте права на исполнение для каждого файла
chmod +x state_estimator.py
chmod +x pid_controller.py
chmod +x optical_flow_node.py
chmod +x rigid_transform_node.py
chmod +x msp_offboard.py
chmod +x flight_node_controller.py
```

Или вы можете сделать все Python-скрипты в директории исполняемыми одной командой:

```bash
# Добавление прав на исполнение для всех Python-файлов в директории scripts
chmod +x ~/catkin_ws/src/pidrone_pkg/scripts/*.py
```

После установки прав на исполнение, необходимо пересобрать пакет:

```bash
# Перейдите в корневую директорию рабочего пространства catkin
cd ~/catkin_ws

# Соберите пакеты
catkin_make

# Применить изменения в окружении
source devel/setup.bash
```

После этих действий ROS должен корректно находить и запускать ваши узлы. 
git checkout -- scripts/flight_controller_node.py scripts/msp_offboard.py