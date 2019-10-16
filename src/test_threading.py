#/usr/bin/env python


import logging
import threading
import time



def thread_function(arr, name):
    time.sleep(1)
    arr[name] = "kek_%d"%(name)

if __name__ == "__main__":

    arr= [None, None, None] 
    threads = list()
    for index in range(3):
        x = threading.Thread(target=thread_function, args=(arr, index,))
        threads.append(x)
        x.start()

    for index, thread in enumerate(threads):
        thread.join()
        
    print(arr)
