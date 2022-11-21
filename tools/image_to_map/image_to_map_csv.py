import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

"""
Converts a pixel image to csv format for a Map object
- Image assumed to be square
- Image pixels are converted from [0, 254] to [-1,1] after grayscaling 
  (ex. to make a zero-aligned image, the image pixel values <0,127,254> will be mapped to <-1,0,1>)
- Uncomment the plt. lines below to show the image; file will be written after closing window
"""
# Set these parameters
input_filepath = '/root/CRATER_GRADER/tools/image_to_map/data/cmu_craters_smoothed.png'
resolution = 0.1

# Append for output filename
output_filepath = input_filepath.split('.')[0] + '_map.csv'

# Open image
im = np.asarray(Image.open(input_filepath).convert('L'))

# Scale to [-1,1]
im = ((im / 254) - 0.5) * 2 * 0.1

# Set and pack data; row-major order for cell data
height = im.shape[0]
width = im.shape[1]
map_data = np.concatenate([[height, width, resolution], np.flip(im, axis=0).flatten()])

plt.imshow(im, cmap='gray', vmin=-1, vmax=1)
plt.show()

# Save to file
np.savetxt(output_filepath, map_data, fmt='%f')
