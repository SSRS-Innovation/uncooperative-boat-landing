from multiprocessing import Process
# from threading import Thread

import positioner_gazebo_box
import Test_1




if __name__ == '__main__':
    Process(target=Test_1.tester).start()
    Process(target=positioner_gazebo_box.positioner).start()

