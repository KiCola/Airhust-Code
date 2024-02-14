import cv2
import os

txt_path = './labels'
pic_path = './newdataset'

# 读取标注文件和图像文件
txt_files = os.listdir(txt_path)
for txt_file in txt_files:
    # 获取图像文件名（假设标注文件名与图像文件名对应）
    img_file = os.path.join(pic_path, txt_file.replace(".txt", ".jpg"))
    
    # 读取图像
    image = cv2.imread(img_file)
    iheight, iwidth, _ = image.shape
    # 读取标注文件
    with open(os.path.join(txt_path, txt_file), "r") as f:
        lines = f.readlines()
        for line in lines:
            # 解析标注信息（假设每行格式为：类别标签 x_center y_center width height）
            label, x1, y1, x2, y2 = line.strip().split()
            
            # 计算边界框的左上角和右下角坐标
            # x1 = int((float(x_center) - float(width) / 2)*iwidth)
            # y1 = int((float(y_center) - float(height) / 2)*iheight)
            # x2 = int((float(x_center) + float(width) / 2)*iwidth)
            # y2 = int((float(y_center) + float(height) / 2)*iheight)
            nx1 = int((float(x1) - float(x2) / 2))
            ny1 = int((float(y1) - float(y2) / 2))
            nx2 = int((float(x1) + float(x2) / 2))
            ny2 = int((float(y1) + float(y2) / 2))
            # x1 = int(float(x1))
            # y1 = int(float(y1))
            # x2 = int(float(x2))
            # y2 = int(float(y2))
            # 在图像上绘制边界框
            cv2.rectangle(image, (nx1, ny1), (nx2, ny2), (0, 255, 0), 5)
            
            # 标注类别名称
            cv2.putText(image, label, (nx1, ny1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 调整图像尺寸为指定大小
    new_width = 800
    new_height = 600
    resized_image = cv2.resize(image, (new_width, new_height))
    
    # 显示带有边界框的调整后的图像
    cv2.imshow("Resized Image with Bounding Boxes", resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
