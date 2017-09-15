
from matplotlib import pyplot as plt


def pop(img_arr, time, kwargs):
    plt.imshow(img_arr, **kwargs)
    plt.pause(time)
