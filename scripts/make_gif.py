import imageio
import os

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG_DIR = os.path.join(PROJECT_DIR, "img")

def make_gif_helper(frames_dir, export_path):
    image_files = [f for f in os.listdir(frames_dir) if f.endswith('.jpg')]
    image_files.sort(key=lambda x: int(x.split('.')[0]))

    images = []
    for image_file in image_files:
        image_path = os.path.join(frames_dir, image_file)
        images.append(imageio.imread(image_path))

    imageio.mimsave(export_path, images, duration=100, loop=0)  # Set the path to your output file and desired fps

    print("GIF exported successfully to: " + export_path + "\n")

def make_gif():
    frames_dir = os.path.join(IMG_DIR, 'frames')
    export_path = IMG_DIR + '/animation.gif'
    make_gif_helper(frames_dir, export_path)

    frames_local_dir = os.path.join(IMG_DIR, 'frames_local')
    export_path_local = IMG_DIR + '/animation_local.gif'
    make_gif_helper(frames_local_dir, export_path_local)

if __name__ == "__main__":
    make_gif()