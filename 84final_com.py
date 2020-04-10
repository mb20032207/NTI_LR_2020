# coding: utf8
import rospy
from clever import srv
import cv2
import numpy as np
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from clever.srv import SetLEDEffect
from sensor_msgs.msg import Image
import pyzbar
from pyzbar.pyzbar import decode
import math
from  itertools import permutations
from numpy import median
rospy.init_node('flight')
import time

index = 0
res = {x:[] for x in range(10)}

aft = ''
bir = np.zeros((500, 500, 3), np.uint8)
bir[:, 0:500] = (255, 0, 0)
bir = cv2.cvtColor(bir, cv2.COLOR_BGR2RGB)


big = np.zeros((500, 500, 3), np.uint8)
big[:, 0:500] = (0, 255, 0)
big = cv2.cvtColor(big, cv2.COLOR_BGR2RGB)


biy = np.zeros((500, 500, 3), np.uint8)
biy[:, 0:500] = (255, 255, 0)
biy = cv2.cvtColor(biy, cv2.COLOR_BGR2RGB)
#res - itog otchet
#index

#connect to proxy
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
#navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
#set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
#set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
#set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
#set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

bridge = CvBridge() #подключение "моста" для переброски изображения с камеры дрона в cv2
image_pub = rospy.Publisher('~image', Image, queue_size=1)



list_zar = set() #Список зараженных
con = ""
innoth = []
#innoth - массив нездоровых людей
#con - переменная отвечающая за qr код (строка)
last_time = time.time()
last_time_QR = time.time()
last_five= ['','','','','']



def flight():   # мигание
    set_effect(r=192, g=5, b=248)
    print("Good light")
    rospy.sleep(5)



def get_color(img):  # определяем медианный цвет в центральном прямоугольнике
    height, width, channels = img.shape
    h1 = height // 2
    h2 = height // 20
    w1 = width // 2
    w2 = width // 20
    h2 = w2 = min(h2, w2)
    crop_img = img[h1 - h2:h1 + h2, w1 - w2:w1 + w2]  # вырезаем центральный прямоугольник
    # cv2.imshow("cropped", crop_img)
    pixs = [list(map(int, list(crop_img[x]))) for x in permutations(range(h2), 2)]  # превращаем его в массив пикселей

    return (int(median([x[0] for x in pixs])), int(median([x[1] for x in pixs])), int(median([x[2] for x in pixs])))  # по каждому каналу берем медиану - самое устойчивое определение

def color_dist(col1, col2): # функция определения расстояния от одного цвета до другого
    return int(sum([(x[0]-x[1])**2 for x in zip(col1, col2)])**0.5)

def which_color(img):  # функция, определяющая до какого из трех нужных нам цветом, наиболее близок тот цвет, что мы послали функции на вход
    red = [179, 74, 87][::-1]  # red
    yellow = [143, 122, 70][::-1]  # yellow
    green = [65, 120, 93][::-1]  # green
    white = [255, 255, 255][::-1]  # white
    black = [0, 0, 0]  # black
    img_color = get_color(img)
    print (img_color, time.time())  # убрать
    slovar = {}

    slovar[color_dist(red, img_color)] = 'Red'
    slovar[color_dist(yellow, img_color)] = 'Yellow'
    slovar[color_dist(green, img_color)] = 'Green'
    slovar[color_dist(white, img_color)] = ''
    slovar[color_dist(black, img_color)] = ''

    return slovar[min(slovar.keys())]



def image_callback(data):  # функция обрабатывает поступающие изображения с камеры
    global list_zar, res, innoth, aft, last_time, last_five, Flag
    if time.time()-last_time<0.5:   # если функция запускалась менее чем полсекунды назад, то ничего не делаем
        return
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # получаем opencv изображение
    # print("here we go callbakcs")
    str1 = which_color(img)   # определяем наиболее близкий цвет
    aft = str1
    last_five.pop()  # список содержит пять последних опрделенных цветов, т.е. за последние 2,5 секунды
    last_five = [str1]+last_five   # предполагалось из этого списка брать статитсическую моду, для надежности, но некогда было протестировать :(
    if Flag:
        print("here image show:", last_five, time.time())
    last_time = time.time()


def do_mission(coord):   # функция вызывается, когда прилетам на точку и надо определить ее цвет
    a, b = coord
    global aft, index, list_zar, last_five, res
    print ('start mission:', coord, last_five)
    print(aft)    # строковое значение последнего определенного цвета.   надо было тут конечно брать статистическую моду из последних 5 значений
    if len(aft) == 0:
        print("we havent go")
    elif aft[0] == 'Y':  #Проверка
        list_zar.add(coord)
        res[index].append('?')
        innoth.append(index)
        print ('Yellow detected')
        print("Sbrosheno")
        image_pub.publish(bridge.cv2_to_imgmsg(biy, 'bgr8'))  # публикуем желтый сигнал (желтую картинку)
        flight()  # мигаем
    elif aft[0] == 'R':
        list_zar.add(coord)
        res[index].append('+')
        innoth.append(index)
        print ('Red detected')
        print("Sbrosheno")
        image_pub.publish(bridge.cv2_to_imgmsg(bir, 'bgr8'))  # публикуем красный сигнал (красную картинку)
        flight()
    else:
        res[index].append('-')
        res[index].append('Healthy')
        print ('Green detected')
        image_pub.publish(bridge.cv2_to_imgmsg(big, 'bgr8')) # публикуем зеленый сигнал (зеленую картинку)
    res[index] = [a, b]
    index += 1



def image_callback_qr(data):  # функция вызывается, когда прилетам на точку и надо определить QR
    global con, last_time, last_time_QR, image_pub
    if time.time()-last_time_QR<2:
        return
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # получаем opencv изображение
    #test1 = decode(image)
    test1 = decode(img)
    last_time_QR = time.time()
    if len(str(test1)) != 0:
        con = str(test1)
        print(con)
        res[innoth[index]].append(con)
        if con[0] == 'c' or con[0] == 'C':
            set_effect(r=255, g=0, b=0)
            print ('Covid-19 detected :(')
            image_pub.publish(bridge.cv2_to_imgmsg(bir, 'bgr8'))
        else:
            print('Covid-19 NOT detected :)')
            image_pub.publish(bridge.cv2_to_imgmsg(big, 'bgr8'))
    else:
        con = ""


Flag = True
#patient coordinates
d = {1:(0.295 , 0.295), 2:(0.885, 0.295), 3:(0.295,0.885), 4:(0.885, 0.885), 5:(0.295, 1.475), 6:(0.885, 1.475), 7:(0.295, 2.065), 8:(0.885, 2.065), 9:(0.59, 2.655), 10:(0, 0)}


navigate(x=0, y=0, z=0.6, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(20)
print("takeoff succesfull")


img_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback,  queue_size = 1)
for tochka in sorted(d):   # основной цикл по первому облету
    a, b = d[tochka]
    navigate(x = a, y = b, z = 0.6, speed = 0.5, frame_id = 'aruco_map')  #go to the point

    rospy.sleep(12)
    if tochka==10:
        rospy.sleep(3)
        break

    print("we are here:", d[tochka])

    do_mission((a,b))

    rospy.sleep(1)

Flag = False

list_zar = sorted(list_zar)
print(list_zar)
print (len(list_zar))
print ('preparing to landing')
land()
print("we have landed")
print("we're sleeping for 120")

rospy.sleep(120)
print("starting...")
index = 0
navigate(x=0, y=0, z=0.6, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(5)

if len(list_zar) == 0:
    list_zar = [(0.295 , 0.295), (0.885, 0.295), (0.295,0.885), (0.885, 0.885), (0.295, 1.475), (0.885, 1.475), (0.295, 2.065), (0.885, 2.065), (0.59, 2.655)]

# if (len(list_zar) == 0):
#     for x2 in sorted(d):
#         list_zar.append((d[x2][0], d[x2][1]))
img_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback_qr,  queue_size = 1)  # Вызов функции
for (x, y) in list_zar:     # основной цикл по второму облету
    navigate(x=x, y=y, z=0.6, speed=0.5, frame_id='aruco_map')
    rospy.sleep(11)

print ('going back')
navigate(x = 0, y = 0, z = 0.6, speed = 0.5, frame_id = 'aruco_map')     #Летим домой
rospy.sleep(8)
land()
print(res)
rospy.sleep(100)