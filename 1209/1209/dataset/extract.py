import os

txt_path = 'labels'
pic_path = 'newdataset'

txt_files = os.listdir(txt_path)

with open("./trainlist.txt", "w", encoding="utf-8") as file:
    for txt_file in txt_files:
        img_file = os.path.join(os.getcwd(), pic_path,
                                txt_file.replace('.txt', '.jpg'))
        txt_file = os.path.join(os.getcwd(), txt_path, txt_file)
        if os.path.exists(img_file) and os.path.exists(txt_file):
            file.write(txt_file.replace(".txt", ".jpg")+"\n")
