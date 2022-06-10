from multiprocessing import Process
import os
import time

def info(title):
    print(title)
    print('module name:', __name__)
    print('parent process:', os.getppid())
    print('process id:', os.getpid())

def f(name):
    time.sleep(3)
    info('function f')
    print('hello', name)

if __name__ == '__main__': # important. or else it won't work.
    info('main line')
    p = Process(target=f, args=('bob',))
    p.start()
    while(p.is_alive()):
        print(f"still alive {os.getppid()}")
        time.sleep(0.1)
    # p.join()