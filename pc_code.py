import numpy as np
import cv2
import json
import serial, time

np.set_printoptions(suppress=True, precision=8)

def detect_and_refine_corners(gray, max_corners=200, quality=0.5, min_dist=5, win_size=(5,5), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 15, 0.03)):
    rough = cv2.goodFeaturesToTrack(gray, max_corners, quality, min_dist) #Находим углы
    # rough = rough.astype(np.float32)  #Программа ожидает переменные типа float32, поэтому мы их переводим в них
    # precise = cv2.cornerSubPix(gray, rough, winSize=win_size, zeroZone=(-1,-1), criteria=criteria) #Уточняем местоположение уже найденных углов с помощью goodFeaturesToTrack

    return rough.squeeze()

def send_to_data (data_list):
    # для каждой точки шлём x как 'a', y как 'b', z как 'c'
    for row in data_list:  # Берем по очереди 10 строк
        for key, val in zip(['a', 'b'],row):  # Берем из одной строки по каждому элементу. zip(['a','b','c'] означает, что мы делаем три итерации. в одном цикле сразу два вектора, что значительно проще.
            msg = f"{key}{val:.2f}\n"  # К элементу спереди крепим идентификатор-букву
            uart.write(msg.encode())  # с помощью encode() кодируем данные в UTF-8 и отправляем Ардуино
            # ждём строку "OK" от Arduino
            resp = uart.readline().decode().strip()  # ser.readline() читает данные от Ардуино. decode() декодирует UTF-8 в Python-строку.strip() убирает начальные и концевые пробелы.
            print(f"Sent {msg.strip():6s} → Arduino: {resp}")
            time.sleep(0.1)  # небольшая задержка между байтами

    return 0

with open('calibration_data.json') as f:
    cal = json.load(f)

fx = cal['fx']
fy = cal['fy']
cx = cal['cx']
cy = cal['cy']
z  = cal['z']

cam  = np.array(cal['camera_matrix'])
dist = np.array(cal['distortion'])
R    = np.array(cal['R'])
t    = np.array(cal['t'])

cam_coord = []
world_coord = []
int_coord = []

last_x = 0
last_y = 0

img = cv2.imread('Cubes-3.jpg')
h, w = img.shape[:2] #Получаем размер изображения

newK, _ = cv2.getOptimalNewCameraMatrix(cam, dist, (w, h), 1, (w, h)) #Получаем новую интринсик матрицу камеры за счет вектора значений дисторсии камеры
dst = cv2.undistort(img, cam, dist, None, newK) #Проводим андисторсию, чтобы избавиться от бочкообразности изображения

fx, fy = newK[0,0], newK[1,1] #Передаем новые значения фокусного расстояния переменным (после андисторсии)
cx, cy = newK[0,2], newK[1,2] #Передаем новые значения центра обьекта переменным (после андисторсии)

gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY) #Теперь мы передаем изображение без дисторсии
corners =detect_and_refine_corners(gray)

for i, (u, v) in enumerate(corners):
    cv2.circle(img, (int(round(u)), int(round(v))), 5, (255, 0, 0), cv2.FILLED)
    x_c = (z / fx) * (u - cx)
    y_c = (z / fy) * (v - cy)
    cam_pt = np.array([x_c, y_c, z])
    cam_coord.append(cam_pt)

    # camera coords → world coords
    pt = R.T.dot(cam_pt - t)
    world_coord.append(pt)

for pt in world_coord:
    if pt[0] < 0:
        pt[0] = 0.0
    if pt[1] < 0:
        pt[1] = 0.0

for pt in world_coord:
    int_coord.append([int(pt[0]), int(pt[1])])

# 1) Формируем множество уникальных точек, но только тех, у которых
#    x % 5 == 0 и y % 5 == 0
unique_points = set() #храним координаты в хеш таблице
for x, y in int_coord:
    if x % 5 == 0 and y % 5 == 0:
        unique_points.add((x, y))

# Теперь у нас в unique_points ровно то, что нам нужно, без дубликатов.
result = []
# 2) Группируем по x:
groups = {}  # словарь: { x_val: [ список y_val ] }
for x, y in unique_points:
    groups.setdefault(x, []).append(y)

# 3) В каждой группе сортируем y:
for x_val in groups:
    groups[x_val].sort(reverse=True)

# 4) Перебираем x в возрастании, а внутри каждой – y в возрастании:
for x_val in sorted(groups.keys()):
    for y_val in groups[x_val]:
        result.append([x_val, y_val])


# 5) Выводим:
print("Отобранные векторы в порядке возрастания x→y:")
for v in result:
    print(v)

port = "COM3"
uart = serial.Serial(port, 9600, timeout=2)
time.sleep(2)
uart.reset_input_buffer()

data = send_to_data(result)
uart.write(b"p\n")

print("\n=== MATRIX ON ARDUINO ===")
while True: #ждем пока Ардуино вышлет обратно весь полученный массив.
    line = uart.readline().decode('ascii').strip() #читаем полученные данные от Ардуино.decode('ascii') чуть проще и быстрее, потому что ASCII-декодер намного легче UTF-8-декодера.
    if not line:
        break        # таймаут или пустая строка — выходим
    print(line)

uart.close()

# print("Отобранные векторы (целые координаты):")
# print(int_coord)

# print("Сырые:")
# print(world_coord)

cv2.imshow('Frame', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

