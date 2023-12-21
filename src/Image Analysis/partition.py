from PIL import Image
import cv2
import os

def partition_image(image_path,partition_size_height, partition_size_width):
    image = Image.open(image_path)
    new_pic = (0,0,204,96)
    image1 = image.crop(new_pic)
    image1.show()
    image_width, image_height = image1.size

    for i in range(0, image_width, partition_size_width): #204/12 and 144/12
        for j in range(0, image_height, partition_size_height):
            box = (i, j, i + partition_size_width, j + partition_size_height)
            yield image.crop(box)

def main():
    folder = 'partitioned_images'
    os.makedirs(folder, exist_ok=True)

    for index, partition in enumerate(partition_image('data/picture_0.jpg', 17,12)):

        # Zoom the partition to fit the screen.
        zoomed_partition = partition.resize((int(partition.width * 12), int(partition.height * 12)))
         # Display the zoomed partition.
        zoomed_partition.show()
        #partition.show()
        user_input = input(f"Do you see a robot in image partition {index}? (y/n): ")
        if user_input.lower() == 'y':
            partition.save(f"partitioned_images/robot_{index}.jpg")
        else:
            partition.save(f"partitioned_images/non_{index}.jpg")

if __name__ == "__main__":
    main()

