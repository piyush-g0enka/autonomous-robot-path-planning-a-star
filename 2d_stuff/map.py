import numpy as np
import cv2
from matplotlib import pyplot as plt
import cv2

# Define canvas size
canvas_height = 3000
canvas_width = 6000

#Clearance Value
c=50

# Create canvas
canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

# Display canvas shape
print("Canvas shape:", canvas.shape)
# cv2_imshow(canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()


for i in range(1800+c):
    canvas[i][(1500-c):(1520+c)][:]=[255,255,255]
    canvas[i][(4500-c):(4520+c)][:]=[255,255,255]
for i in range((1200-c),3000):
    canvas[i][(3000-c):(3020+c)][:]=[255,255,255]

for i in range(2300-c,2600+c):
    canvas[i][(600-c):(900+c)][:]=[255,255,255]
    canvas[i][(2100-c):(2400+c)][:]=[255,255,255]
    canvas[i][(3600-c):(3900+c)][:]=[255,255,255]
    canvas[i][(5100-c):(5400+c)][:]=[255,255,255]

for i in range(600-c,900+c):
    canvas[i][(2100-c):(2400+c)][:]=[255,255,255]
    canvas[i][(3600-c):(3900+c)][:]=[255,255,255]
    canvas[i][(5100-c):(5400+c)][:]=[255,255,255]



cv2.imshow("Canvas", canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
