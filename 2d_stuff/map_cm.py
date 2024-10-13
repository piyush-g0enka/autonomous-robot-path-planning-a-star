import numpy as np
import cv2
from matplotlib import pyplot as plt
import cv2

# Define canvas size
canvas_height = 300
canvas_width = 600

#Clearance Value
c=20

# free space
canvas = np.zeroes((canvas_height, canvas_width, 3), dtype=np.uint8) 
# Display canvas shape
print("Canvas shape:", canvas.shape)
# cv2_imshow(canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()

#obstacles
for i in range(180+c):
    canvas[i][(150-c):(152+c)][:]=1
    canvas[i][(450-c):(452+c)][:]=1
for i in range((120-c),300):
    canvas[i][(300-c):(302+c)][:]=1

for i in range(230-c,260+c):
    canvas[i][(60-c):(90+c)][:]=1
    canvas[i][(210-c):(240+c)][:]=1
    canvas[i][(360-c):(390+c)][:]=1
    canvas[i][(510-c):(540+c)][:]=1

for i in range(60-c,90+c):
    canvas[i][(210-c):(240+c)][:]=1
    canvas[i][(360-c):(390+c)][:]=1
    canvas[i][(510-c):(540+c)][:]=1



cv2.imshow("Canvas", canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
