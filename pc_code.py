import numpy as np
import cv2
import json
import serial, time

def main():

    with open('calibration_data.json') as f:
        cal = json.load(f)

    cam = np.array(cal['camera_matrix'])
    dist = np.array(cal['distortion'])
    R = np.array(cal['R'])
    t = np.array(cal['t'])
    z = cal['z']

    img = cv2.imread('Cubes-4.jpg')
    h, w = img.shape[:2]

    newK, _ = cv2.getOptimalNewCameraMatrix(cam, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, cam, dist, None, newK)

    fx, fy = newK[0, 0], newK[1, 1]
    cx, cy = newK[0, 2], newK[1, 2]

    gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
    rough = cv2.goodFeaturesToTrack(gray, 50, 0.5, 5)
    corners = rough.squeeze()

    convert_coord = camera_to_world_coord(corners, img, fx, fy, cx, cy, R, t, z)
    result = split_clear_coord(convert_coord)

    # 5) Выводим:
    print("Coordinates: ")
    for v in result:
        print(v)

    port = "COM3"
    uart = serial.Serial(port, 9600, timeout=2)
    time.sleep(2)
    uart.reset_input_buffer()

    data = send_to_data(result)
    uart.write(b"p\n")

    print("Coords have been sent successfully")
    uart.close()

    cv2.imshow('Frame', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def send_to_data (coord):

    for row in coord:
        for key, val in zip(['a', 'b'],row):
            msg = f"{key}{val}\n"
            uart.write(msg.encode())
            resp = uart.readline().decode().strip()
            print(f"Sent {msg.strip():6s} → Arduino: {resp}")
            time.sleep(0.1)

    return 0


def camera_to_world_coord(coord, img, fx, fy, cx, cy, R, t, z):

    """
 :param coord: input image coord list
 :param fx, fy, cx, cy: camera intrinsic parameters
 :param R, t:  camera extrinsic parameters
 :param z: height (in cm)
 :return: world coord (x, y, z)
    """

    world_coord = []
    int_coord = []

    # circle founded corners on the image
    for u, v in coord:
        cv2.circle(img, (int(round(u)), int(round(v))), 5, (255, 0, 0), cv2.FILLED)

        # image coords -> camera coord
        x_c = (z / fx) * (u - cx)
        y_c = (z / fy) * (v - cy)
        cam_pt = np.array([x_c, y_c, z])

        # camera coords -> world coords
        pt = R.T.dot(cam_pt - t)
        world_coord.append(pt)

    # discard all negative coords
    for pt in world_coord:
        if pt[0] < 0:
            pt[0] = 0.0
        if pt[1] < 0:
            pt[1] = 0.0

    # make coords integers
    for pt in world_coord:
        int_coord.append([int(pt[0]), int(pt[1])])

    return int_coord

def split_clear_coord(coord):

    """
    :param coord: world coord (x, y, z)
    :return: a list of coordinates sorted in descending order, containing only the top-left corner of each cube.
    """

    result = []
    groups = {}

    # choose x % 5 == 0 and y % 5 == 0 only and add them to a dictionary
    for x, y in coord:
        if x % 5 == 0 and y % 5 == 0:
            groups.setdefault(x, []).append(y)

    # sort y's inside every x key
    for x_val in groups:
        groups[x_val].sort(reverse=True)

    # append sorted coords to a list
    for x_val in sorted(groups.keys(), reverse=True):
        for y_val in groups[x_val]:
            result.append([x_val, y_val])

    return result

if __name__ == "__main__":
    main()


