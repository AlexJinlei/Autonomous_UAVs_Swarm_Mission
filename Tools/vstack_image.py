import glob
from PIL import Image

# Stack two images verticlly.
def vstack_images(file1, file2):
    """Merge two images into one, displayed side by side
    :param file1: path to first image file
    :param file2: path to second image file
    :return: the merged Image object
    """
    image1 = Image.open(file1)
    image2 = Image.open(file2)

    (width1, height1) = image1.size
    (width2, height2) = image2.size

    result_width = max(width1, width2)
    result_height = height1 + height2

    result = Image.new('RGB', (result_width, result_height), color=(255,255,255))
    result.paste(im=image1, box=(0, 0))
    result.paste(im=image2, box=(0, height1))
    return result

rootDir = '/Users/jz0006/Desktop/20170621_17-53-59/'

# Load original frames.
images_detected = glob.glob(rootDir+'*_detected.jpg') # Grab all jpg file names.
images_detected.sort() # Sort frame file names.
images_foreground = glob.glob(rootDir+'*_foreground.jpg') # Grab all jpg file names.
images_foreground.sort() # Sort frame file names.

print(images_detected)
print(images_foreground)

for i in xrange(len(images_detected)):
    fig1_filename = images_detected[i]
    fig2_filename = images_foreground[i]
    combined = vstack_images(fig1_filename, fig2_filename)
    combined_filename = fig1_filename[:-12] + 'combined.jpg'
    combined.save(combined_filename)
    print('{} is saved!'.format(combined_filename))










