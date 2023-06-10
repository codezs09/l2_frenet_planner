import imageio
import os

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG_DIR = os.path.join(PROJECT_DIR, "img")

path = PROJECT_DIR + '/img/frames'
image_files = [f for f in os.listdir(path) if f.endswith('.jpg')]
image_files.sort(key=lambda x: int(x.split('.')[0]))

images = []
for image_file in image_files:
    image_path = os.path.join(path, image_file)
    images.append(imageio.imread(image_path))

imageio.mimsave(IMG_DIR + '/animation.gif', images, duration=100)  # Set the path to your output file and desired fps
