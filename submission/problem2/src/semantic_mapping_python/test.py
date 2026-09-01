
import cv2
sr = cv2.dnn_superres.DnnSuperResImpl_create()
sr.readModel('EDSR_x4.pb')
sr.setModel('edsr', 4)
img = cv2.imread('image.png')
result = sr.upsample(img)
print('Upscaled size:', result.shape)
cv2.imwrite('image_4x.png', result)
