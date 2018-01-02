import multiprocessing
from joeydash import set_point_to_displace




def main():
    height_queue = multiprocessing.Queue(5)
    points_to_displace_queue = multiprocessing.Queue(5)
    lock = multiprocessing.Lock()
    set_point_to_displace_process = multiprocessing.Process(target=set_point_to_displace,
                                                            args=(lock, height_queue, points_to_displace_queue))
    do_something_process = multiprocessing.Process(target=do_something, args=(lock, height_queue))
    set_point_to_displace_process.start()
    do_something_process.start()
    set_point_to_displace_process.join()
    do_something_process.join()



if __name__ == "__main__":
    main()
