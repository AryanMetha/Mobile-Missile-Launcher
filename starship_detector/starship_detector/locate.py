import pyrealsense2 as rs
import numpy as np
import cv2
import torch

model = torch.hub.load("ultralytics/yolov5", "custom", path="yolov5nu.pt", source="local")

#set up camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

pipeline.start(config)


#main body
while 1:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    result = model(color_image)
    #note:result will be a tensor which may contain nothing 
    #since only one object
    for vector in result.xywh[0]:
        xc, yc, w, h, conf, cls = vector
        z=depth_image[xc][yc]*depth_scale
