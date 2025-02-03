import os

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""

def create(folder):
    """
    Documentation
    Creates new subdirectories for the generated plots

    Input:
    folder:         path to the directory specified by user

    Output:

    """

    if not os.path.isdir(folder):
        os.makedirs(folder)

    # create parameter directory
    name = '/params'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)

    # create data directory
    name = '/data'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)

    # create plots directory
    name = '/plots'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)
