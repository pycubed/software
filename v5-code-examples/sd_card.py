"""
Routine that prints the names of all the directories and files on the SD card.
"""
import time, os, storage
from lib.pycubed import cubesat 


def print_directory(path, tabs = 0):
    for file in os.listdir(path):
        stats = os.stat(path + "/" + file)
        filesize = stats[6]
        isdir = stats[0] & 0x4000
 
        if filesize < 1000:
            sizestr = str(filesize) + " by"
        elif filesize < 1000000:
            sizestr = "%0.1f KB" % (filesize / 1000)
        else:
            sizestr = "%0.1f MB" % (filesize / 1000000)
 
        prettyprintname = ""
        for _ in range(tabs):
            prettyprintname += "   "
        prettyprintname += file
        if isdir:
            prettyprintname += "/"
        print(f"{prettyprintname:<40} Size: {sizestr:>10}")
 
        # recursively print directory contents
        if isdir:
            print_directory(path + "/" + file, tabs + 1)


######################### MAIN PORTION #########################
if cubesat.hardware["SDcard"]:
    print("Files on system:")
    print("="*20)
    print_directory("/sd")
else:
    cubesat.RGB = (255, 0, 0)
    print("[ERROR]: No SD card detected. Press Ctrl+C to exit.")
    while True:
        time.sleep(1)