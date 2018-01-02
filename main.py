import multiprocessing
from joeydash import set_point_to_displace
from alt_control import control_main


def main():
    height_queue = multiprocessing.Queue(1)
    points_to_displace_queue = multiprocessing.Queue(1)
    lock = multiprocessing.Lock()
    set_point_to_displace_process = multiprocessing.Process(target=set_point_to_displace, args=(lock, height_queue, points_to_displace_queue))
    control_main_processing = multiprocessing.Process(target=control_main, args=(lock, height_queue, points_to_displace_queue))
    set_point_to_displace_process.start()
    control_main_processing.start()
    set_point_to_displace_process.join()
    control_main_processing.join()


if __name__ == "__main__":
    main()
