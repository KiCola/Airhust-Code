import os
import cv2
txt_path = 'labels'
pic_path = 'newdataset'
os.makedirs('./labels_normalized', exist_ok=True)

txt_files = os.listdir(txt_path)
for txt_file in txt_files:
    img_file = os.path.join(pic_path, txt_file.replace(".txt", ".jpg"))
    image = cv2.imread(img_file)
    iheight, iwidth, _ = image.shape
    with open(os.path.join(txt_path, txt_file), "r") as file:
        content = file.readlines()
        for line in content:
            label, x1, y1, x2, y2 = line.strip().split()
            x_ceneter = float((float(x1)+float(x2))/2/iwidth)
            y_center = float((float(y1)+float(y2))/2/iheight)
            width = float(abs(float(x2)-float(x1))/iwidth)
            height = float(abs(float(y2)-float(y1))/iheight)

            with open(os.path.join(txt_path+"_normalized", txt_file.replace('.txt', '.txt')), "a") as fp:
                fp.write(label + ' ' + str(x_ceneter) + ' ' + str(y_center)
                         + ' ' + str(width) + ' ' + str(height) + "\n")

