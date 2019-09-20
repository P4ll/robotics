# Лабораторная работа 2

## Техническое задание
1. В среде V-REP необходимо реализовать построение карты окружающего робота пространства при помощи SLAM. Требуется использовать карту, полученную в результате работы № 1.
2. Необходимо разработать программу, с помощью которой робот может передвигаться до точки, назначенной пользователем, по готовой карте.
## Реализация
В работе использовалась библиотека BreezySlam, которая реализует алгоритм RMHC SLAM (Random-Mutation Hill-Climbing SLAM). Для вывода карты на экран использовался RoboViz.
1) Подготовка сцены и робота
Была скопирована сцена из работы № 1. На крышу робота был добавлен лидар Hokuyo 04LX UG01.
![Pioneer с лидаром Hokuyo](/pics/pio.png)
Лидар через определенное время собирает данные о близлежащих объектах (684 значения - расстояния до препятствий). Данные обрабатываются прямо в среде V-REP, запаковываются и отправляются как строка. Для большей точности алгоритма, есть возможность дополнить расстояния одометрией. В данной ситуации одометрия рассматривается как угол поворота колеса в радианах.
2) Описание взаимодействия с BreezySlam
Скрипт Python получает данные по имени сигнала, затем обрабатывает их (необходимо перевести значения расстояний из метров в сантиметры) и передает объекту SLAM. объект делает изменения на карте, а также вычисляет новые координаты <img src="svgs/84a4068373fea90ea32ef2b2bc897de2.svg?invert_in_darkmode" align=middle width=54.686445000000006pt height=24.56552999999997pt/>, где <img src="svgs/0acac2a2d5d05a8394e21a70a71041b4.svg?invert_in_darkmode" align=middle width=25.265625000000004pt height=14.102549999999994pt/> – координаты робота на плоскости, а <img src="svgs/11c596de17c342edeed29f489aa4b274.svg?invert_in_darkmode" align=middle width=9.388665000000001pt height=14.102549999999994pt/> – угол поворота на ней. Необходимо отметить, что <img src="svgs/11c596de17c342edeed29f489aa4b274.svg?invert_in_darkmode" align=middle width=9.388665000000001pt height=14.102549999999994pt/> – величина абсолютная и выражается в градусах. Это значит, что вращение против часовой стрелки после одного оборота будет возвращать значения <img src="svgs/1f7dc45e6508460721f9769ebed420fc.svg?invert_in_darkmode" align=middle width=41.869245pt height=21.10812pt/>.
Полученная карта и координаты новой позиции передаются объекту RoboViz для вывода на экран.
3) Реализация езды по точкам
Рассмотрим карту, созданную в процессе проезда комнаты роботом. Предположим, что точка А - текущая позиция робота, а B - точка назначения робота. Идея проста - поворачивать до тех пор, пока робот не встанет на курс (прямую AB), затем двигаться по нему, пока это возможно. В случае препятствия передавать управления алгоритму из первой работы. Необходимо отметить, что прямая перестраивается каждый раз с получением данных от сенсоров. BreezySLAM третьей координатой задает угол поворота робота, удобно найти угол между AB и осью абсцисс, затем использовать разницу между этим углом и углом робота для ПИД-регулятора.
![Предполагаемый курс движения робота к точке](/pics/path.png)
Для каждой точки известны координаты <img src="svgs/002256562817956f3233d16995d0e10a.svg?invert_in_darkmode" align=middle width=64.44735pt height=24.56552999999997pt/> и <img src="svgs/1bc0deb405a9cd3947b9b7c888a65eaf.svg?invert_in_darkmode" align=middle width=65.411445pt height=24.56552999999997pt/>, тогда уравнение прямой, проходящей через две точки будет следующим:<p align="center"><img src="svgs/96f8fc02922419d5b7fd94a54fa40d40.svg?invert_in_darkmode" align=middle width=256.79609999999997pt height=34.415535pt/></p>Уравнение для оси абсцисс можно построить аналогично (1) (например для <img src="svgs/6c5b383c97125d6712f47ab816025976.svg?invert_in_darkmode" align=middle width=36.403620000000004pt height=24.56552999999997pt/> и <img src="svgs/00306c9c29c6ff4349788ae30e389617.svg?invert_in_darkmode" align=middle width=42.772455pt height=24.56552999999997pt/> <img src="svgs/88434522a9c464c6b2182f622b7529fb.svg?invert_in_darkmode" align=middle width=115.44637499999999pt height=21.10812pt/>. Тангенс угла между прямыми можно найти как: <p align="center"><img src="svgs/865df3036e40790bea5bf805d9e65210.svg?invert_in_darkmode" align=middle width=168.7422pt height=39.41553pt/></p>
где <img src="svgs/aa90653a26bc63b138fb304972d81589.svg?invert_in_darkmode" align=middle width=15.05394pt height=22.745910000000016pt/> и <img src="svgs/a8ebf8c468236800b8ed78d42ddbfa57.svg?invert_in_darkmode" align=middle width=15.05394pt height=22.745910000000016pt/> - угловые коэффициенты прямых. <img src="svgs/102dfcf556f1840bbd47ebcd236f89db.svg?invert_in_darkmode" align=middle width=144.9195pt height=24.56552999999997pt/>, тогда <img src="svgs/c2253e510a89add3170e0b982198dcd6.svg?invert_in_darkmode" align=middle width=109.99609499999998pt height=24.56552999999997pt/> Следом можно узнать значение <img src="svgs/7d886a3a956ac3266c364699efce077e.svg?invert_in_darkmode" align=middle width=64.62538500000001pt height=24.56552999999997pt/>, поняв, в какой четверти находится точка назначения.
После нахождения требуемого угла, найдем разницу <img src="svgs/546ae4a13cb4385890fad0c015e81be3.svg?invert_in_darkmode" align=middle width=75.080445pt height=22.745910000000016pt/>, где <img src="svgs/11c596de17c342edeed29f489aa4b274.svg?invert_in_darkmode" align=middle width=9.388665000000001pt height=14.102549999999994pt/>- текущий угол поворота робота. Для того, чтобы робот следовал курсу можно воспользоваться ПИД-регулятором из первой работы. К существующему алгоритму необходимо добавить несколько условий:
    - если передний и боковой сенсоры не регистрируют препятствий, тогда используем прямую AB как целевую, то есть в регулятор будет передаваться значение <img src="svgs/91543b476af9f170afea06e6f12155a1.svg?invert_in_darkmode" align=middle width=67.52031pt height=22.381919999999983pt/>, где <img src="svgs/6dec54c48a0438a5fcde6053bdb9d712.svg?invert_in_darkmode" align=middle width=8.467140000000004pt height=14.102549999999994pt/> - требуемая дистанция справа из первой работы, <img src="svgs/07617f9d8fe48b4a7b3f523d6730eef0.svg?invert_in_darkmode" align=middle width=9.867990000000004pt height=14.102549999999994pt/> - коэффициент регулирования для поддержания требуемого угла (было выбрано значение <img src="svgs/2c6d2c92561d4eb0752000e383451528.svg?invert_in_darkmode" align=middle width=45.491655pt height=21.10812pt/>);
    - если препятствие есть спереди, тогда по прямой AB робот уже двигаться не может то, в какую сторону повернет робот будет зависеть от значения <img src="svgs/07617f9d8fe48b4a7b3f523d6730eef0.svg?invert_in_darkmode" align=middle width=9.867990000000004pt height=14.102549999999994pt/>, если оно большое, тогда минимальное отклонение заставит робота возвращаться на прямую и только очень близкое препятствие спереди заставит его свернуть, а если оно будет слишком маленькое, тогда робот будет возвращаться на прямую только после внушительного поворота;
    - если есть данные со всех сенсоров, тогда регуляция будет производится для минимума из их значений.