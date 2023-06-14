import imageio
import os

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG_DIR = os.path.join(PROJECT_DIR, "img")

def make_gif():
    path = PROJECT_DIR + '/img/frames'
    image_files = [f for f in os.listdir(path) if f.endswith('.jpg')]
    image_files.sort(key=lambda x: int(x.split('.')[0]))

    images = []
    for image_file in image_files:
        image_path = os.path.join(path, image_file)
        images.append(imageio.imread(image_path))

    export_path = IMG_DIR + '/animation.gif'
    imageio.mimsave(export_path, images, duration=100, loop=0)  # Set the path to your output file and desired fps

    print("GIF exported successfully to: " + export_path + "\n")

if __name__ == "__main__":
    make_gif()