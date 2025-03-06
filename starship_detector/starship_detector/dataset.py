import cv2
import numpy as np
import os
import glob
import random

input_folder = "/home/aryan/workspaces/umic_ws/src/starship_detector/starship_detector/data/og_images/starships.v3i.yolov5-obb/train/images"

output_folder = "/home/aryan/workspaces/umic_ws/src/starship_detector/starship_detector/data/og_images/starships.v3i.yolov5-obb/train/images"



image_files = glob.glob(os.path.join(input_folder, "*.jpg"))




def load_yolo_annotation(txt_path, img_width, img_height):
    
    bboxes = []
    with open(txt_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            data = line.strip().split()
            class_id = int(data[0])
            x_center, y_center, width, height = map(float, data[1:])

            
            x_min = int((x_center - width / 2) * img_width)
            y_min = int((y_center - height / 2) * img_height)
            x_max = int((x_center + width / 2) * img_width)
            y_max = int((y_center + height / 2) * img_height)

            bboxes.append([class_id, x_min, y_min, x_max, y_max])
    return bboxes

def convert_to_yolo_format(bboxes, img_width, img_height):
    
    yolo_bboxes = []
    for box in bboxes:
        class_id, x_min, y_min, x_max, y_max = box

        x_center = (x_min + x_max) / 2 / img_width
        y_center = (y_min + y_max) / 2 / img_height
        width = (x_max - x_min) / img_width
        height = (y_max - y_min) / img_height

        yolo_bboxes.append(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")
    return yolo_bboxes


def flip_image_and_bboxes(image, bboxes, img_width):
    
    flipped_img = cv2.flip(image, 1)
    
    new_bboxes = []
    for box in bboxes:
        class_id, x_min, y_min, x_max, y_max = box
        new_x_min = img_width - x_max
        new_x_max = img_width - x_min
        new_bboxes.append([class_id, new_x_min, y_min, new_x_max, y_max])
    

    return flipped_img, new_bboxes

def rotate_image(image, bboxes, angle):
   
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)
    
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated_img = cv2.warpAffine(image, M, (w, h))
    
    new_bboxes = []
    for box in bboxes:
        class_id, x_min, y_min, x_max, y_max = box
        points = np.array([
            [x_min, y_min], [x_max, y_min], [x_max, y_max], [x_min, y_max]
        ])
        ones = np.ones((4, 1))
        points = np.hstack([points, ones])
        rotated_points = np.dot(M, points.T).T

        x_coords = rotated_points[:, 0]
        y_coords = rotated_points[:, 1]
        new_bboxes.append([
            class_id,
            int(min(x_coords)), int(min(y_coords)),
            int(max(x_coords)), int(max(y_coords))
        ])

   
    return rotated_img, new_bboxes

def apply_random_histogram_equalization(image):
    """Applies histogram equalization to enhance contrast."""
    if random.random() < 0.5:  # 50% chance to apply
        if len(image.shape) == 2:  # Grayscale image
            return cv2.equalizeHist(image)
        else:  # Color image (apply to each channel separately)
            img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
            img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])  # Apply to Y channel
            return cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        

    return image


for img_path in image_files:
    img = cv2.imread(img_path)
    height, width = img.shape[:2]


    
    txt_path = img_path.replace(".jpg", ".txt") 


    try:
        with open(txt_path, "r") as f:
            bboxes = load_yolo_annotation(txt_path, width, height)
    except FileNotFoundError:
        print(f"Warning: Annotation file not found for {img_path}, skipping...")
        continue  # Skip images without annotations
    except Exception as e:
        print(f"Error reading {txt_path}: {e}")
        continue  # Skip this image in case of any other error
  
    if random.random() < 0.5:  
        img, bboxes, = flip_image_and_bboxes(img, bboxes, width)

    img_path = img_path.replace(".jpg", "f.jpg")
        
    if random.random() < 0.7:  
        angle = random.randint(-15, 15)  
        img, bboxes,= rotate_image(img,bboxes,angle)

    img_path = img_path.replace(".jpg", "f.jpg")

        
    if random.random()<0.2:
        img= cv2.GaussianBlur(img, (3,3), 0)

    img_path = img_path.replace(".jpg", "f.jpg")

    
    if random.random()<0.5:
        img=apply_random_histogram_equalization(img)
    
    img_path = img_path.replace(".jpg", "f.jpg")
   
    yolo_bboxes = convert_to_yolo_format(bboxes, width, height)

    
    output_img_path = os.path.join(output_folder, os.path.basename(img_path))
    cv2.imwrite(output_img_path, img)

    
    output_txt_path = output_img_path.replace(".jpg", ".txt")
    with open(output_txt_path, "w") as f:
        f.write("\n".join(yolo_bboxes))

