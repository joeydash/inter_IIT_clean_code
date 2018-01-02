import cv2, math, numpy as np, multiprocessing


def get_coordinates_in_cm(cam_height, cap):
    constant = 557
    lower_colour = np.array([20, 45, 80])
    upper_colour = np.array([46, 130, 220])
    point_to_displace = np.zeros(2)

    _, frame = cap.read()

    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(hsv, (15, 15), 0)
    mask = cv2.inRange(blur, lower_colour, upper_colour)
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours):
        filter(lambda a: cv2.contourArea(a) > 500, contours)
        c = sorted(contours, key=cv2.contourArea)[::-1]
        for contour in c[0:1:]:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, ((2 * x + w) / 2, (2 * y + h) / 2), 2, (0, 0, 255), -1)
            cv2.circle(frame, (width / 2, height / 2), 2, (0, 0, 255), -1)
            cv2.imshow("frame", frame)
            x_displacement_in_pixels = ((2 * x + w) / 2) - (width / 2)
            y_displacement_in_pixels = (height / 2) - ((2 * y + h) / 2)
            point_to_displace = [(x_displacement_in_pixels * cam_height) / constant,
                                 (y_displacement_in_pixels * cam_height) / constant]
            return point_to_displace
    else:
        return point_to_displace
    cv2.imshow("frame", frame)


def set_point_to_displace(lock, height_queue, points_to_displace_queue):
    cap = cv2.VideoCapture()
    while True:
        lock.acquire()
        cam_height = height_queue.get()
        lock.release()
        point_to_displace = get_coordinates_in_cm(cam_height, cap)
        lock.acquire()
        if points_to_displace_queue.full():
            points_to_displace_queue.get()
            points_to_displace_queue.set(point_to_displace)
        else:
            points_to_displace_queue.set(point_to_displace)
        lock.release()


def do_something():
    pass


def main():
    height_queue = multiprocessing.Queue(5)
    points_to_displace_queue = multiprocessing.Queue(5)
    lock = multiprocessing.Lock()
    set_point_to_displace_process = multiprocessing.Process(target=set_point_to_displace(),
                                                            args=(lock, height_queue, points_to_displace_queue))
    do_something_process = multiprocessing.Process(target=do_something, args=(lock, height_queue))
    set_point_to_displace_process.start()
    do_something_process.start()
    set_point_to_displace_process.join()
    do_something_process.join()


if __name__ == "__main__":
    main()
