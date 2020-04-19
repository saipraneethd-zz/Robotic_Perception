import glob
from PIL import Image
import os
# filepaths
fp_in = "../data/straight_line/raw_images/img_*.png"
fp_out = "../assets/straight_line.gif"

# https://pillow.readthedocs.io/en/stable/handbook/image-file-formats.html#gif
files = glob.glob(fp_in)
files.sort(key=os.path.getmtime)
img, *imgs = [Image.open(f) for f in files]
img.save(fp=fp_out, format='GIF', append_images=imgs,
         save_all=True, duration=200, loop=0)
