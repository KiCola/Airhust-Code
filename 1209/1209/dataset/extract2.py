import os

img_path = 'labels'

img_files = os.listdir(img_path)
img_files = [img for img in img_files if img.endswith(".jpg")]

with open("./trainlist.txt", "w", encoding="utf-8") as file:
    for img in img_files:
        img_file = os.path.join(os.getcwd(), img_path, img)
        file.write(img_file+"\n")
