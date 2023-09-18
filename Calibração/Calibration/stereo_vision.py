"""
MIT License

Copyright (c) 2020 Pramod Anantharam

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import cv2
from PIL import Image
import numpy as np
from tqdm import *
import matplotlib.pyplot as plt

"""
Implementation using ideas from:
http://mccormickml.com/2014/01/10/stereo-vision-tutorial-part-i/

"""
BLOCK_SIZE = 7
SEARCH_BLOCK_SIZE = 56


def _read_left_right_image_pair(left_image_path="Testesdevisãoestéreo\assets\images\left\all_light_left.png",
                                right_image_path="Testesdevisãoestéreo\assets\images\right\all_light_right.png"):

    left_im = cv2.imread(left_image_path, 0)
    right_im = cv2.imread(right_image_path, 0)

    left_im_array = np.asarray(left_im)
    right_im_array = np.asarray(right_im)
    print(left_im_array.shape)
    print(right_im_array.shape)

    return left_im_array, right_im_array


def display_image(img, window_name='img'):
    cv2.imshow(window_name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def sum_of_abs_diff(pixel_vals_1, pixel_vals_2):

    if pixel_vals_1.shape != pixel_vals_2.shape:
        return -1

    return np.sum(abs(pixel_vals_1 - pixel_vals_2))


def compare_blocks(y, x, block_left, right_array, block_size=5):
    # Get search range for the right image
    x_min = max(0, x - SEARCH_BLOCK_SIZE)
    x_max = min(right_array.shape[1], x + SEARCH_BLOCK_SIZE)
    # print(f'search bounding box: ({y, x_min}, ({y, x_max}))')
    first = True
    min_sad = None
    min_index = None
    for x in range(x_min, x_max):
        block_right = right_array[y: y+block_size,
                                  x: x+block_size]
        sad = sum_of_abs_diff(block_left, block_right)
        # print(f'sad: {sad}, {y, x}')
        if first:
            min_sad = sad
            min_index = (y, x)
            first = False
        else:
            if sad < min_sad:
                min_sad = sad
                min_index = (y, x)

    return min_index


def right_image_block(x, y):
    left_array, right_array = _read_left_right_image_pair()
    right_im = cv2.imread(
        "Testesdevisãoestéreo\assets\images\left\all_light_left.png", 0)
    x_min = max(0, x - 50)
    x_max = min(right_array.shape[1], x + 25)
    right_im_bbox = cv2.rectangle(right_im, (x_min, y),
                                  (x_max, y + 25),
                                  (0, 0, 255), 2)
    display_image(right_im_bbox, window_name='right')


def get_disparity_map():
    left_array, right_array = _read_left_right_image_pair()


    if left_array is not None:
         left_array = left_array.astype(int)
    else:
        print("Erro: left_array é None. Verifique a leitura da imagem.")
        
    left_array = left_array.astype(int)
    right_array = right_array.astype(int)
    if left_array.shape != right_array.shape:
        raise "Left-Right image shape mismatch!"
    h, w = left_array.shape
    left_im = cv2.imread(
        "Testesdevisãoestéreo\assets\images\right\all_light_right.png", 0)
    disparity_map = np.zeros((h, w))
    # Go over each pixel position
    for y in tqdm(range(BLOCK_SIZE, h-BLOCK_SIZE)):
        for x in range(BLOCK_SIZE, w-BLOCK_SIZE):
            block_left = left_array[y:y + BLOCK_SIZE,
                                    x:x + BLOCK_SIZE]
            min_index = compare_blocks(y, x, block_left,
                                       right_array,
                                       block_size=BLOCK_SIZE)
            disparity_map[y, x] = abs(min_index[1] - x)

    print(disparity_map)
    plt.imshow(disparity_map, cmap='hot', interpolation='nearest')
    plt.savefig('depth_image.png')
    plt.show()
    img = Image.fromarray(disparity_map, 'L')
    img.show()


if __name__ == '__main__':
    get_disparity_map()
