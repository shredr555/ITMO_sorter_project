# __ROS line follower__

В данном проекте, мы реализовали четырех колесного робота с камерой, для отслеживания линии.


## __Подготовка__
____
### __Необходимое программное обеспечение__

- ROS
- python

### __Установка__

Скопируйте наш репозиторий в свой `_catkin_ws/src_`

```
git clone https://github.com/shredr555/ITMO_sorter_project.git
```

## __Запуск__
____

Для запуска симуляции перейдите в `_catkin_ws/src_` и введите следующее

```
cd sorter_description/launch
roslaunch gazebo.launch
```

Для запуска алгоритма следования по линии также перейдите в `_catkin_ws/src_` и введите следующее

```
rosrun sorter_control simple_move.py
```


