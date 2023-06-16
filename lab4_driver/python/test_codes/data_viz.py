import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np


def data_csv(b):

    csvfiles = []
    for t in b.topics:
        data = b.message_by_topic(t)
        csvfiles.append(data)

    print(csvfiles[0])
    data = pd.read_csv(csvfiles[0])
    return data

if __name__ == '__main__':
    
    #read the data
    # b = bagreader('/home/marley/catkin_ws/src/gps_driver/bag_files/2022-09-21-17-20-38.bag')
    b = bagreader("/home/marley/catkin_ws/src/gps_driver/bag_files/stationary_data/2022-09-21-16-43-54.bag")
    csv_data = data_csv(b)



