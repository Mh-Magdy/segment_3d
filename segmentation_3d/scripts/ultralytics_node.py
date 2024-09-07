#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
from segmentation_msgs.msg import ObjectsSegment, ObjectSegment
from ultralytics import YOLO
import cv2
import time

# Initialize the node
rospy.init_node("ultralytics_segmentation")
time.sleep(1)

# Load the YOLO segmentation model
segmentation_model = YOLO("yolov8n-seg.pt")
rospy.loginfo("YOLOv8 model loaded successfully")

# Publishers
objects_segment_pub = rospy.Publisher("/ultralytics/segmentation/objects_segment", ObjectsSegment, queue_size=10)
seg_image_pub = rospy.Publisher("/ultralytics/segmentation/image", Image, queue_size=10)

def callback(rgb_data):
    # Convert the RGB image to a NumPy array
    try:
        image = ros_numpy.numpify(rgb_data)
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return

    height, width, _ = image.shape
    rospy.loginfo(f"Received image with shape: {image.shape}")

    # Apply segmentation model to the RGB image
    seg_result = segmentation_model(image)

    # Prepare the ObjectsSegment message
    objects_msg = ObjectsSegment()
    objects_msg.header = rgb_data.header  # Copy the header from the RGB image
    objects_msg.header.stamp = rospy.Time.now()

    for index, cls in enumerate(seg_result[0].boxes.cls):
        class_index = int(cls.cpu().numpy())
        name = seg_result[0].names[class_index]

        # Ensure result.masks is not None
        if seg_result[0].masks is not None:
            mask = seg_result[0].masks.data.cpu().numpy()[index, :, :]
            mask_resized = cv2.resize(mask, (width, height))
            binary_mask = (mask_resized > 0.5).astype(np.uint8)

            # Get pixel indices for the mask
            y_indices, x_indices = np.where(binary_mask > 0)

            if len(x_indices) == 0 or len(y_indices) == 0:
                rospy.logwarn(f"No valid indices found for object: {name}")
                continue

            # Create ObjectSegment message
            obj_msg = ObjectSegment()
            obj_msg.header = objects_msg.header
            obj_msg.class_name = name
            obj_msg.probability = seg_result[0].boxes.conf[index].item()  # Accessing the probability score
            obj_msg.x_indices = x_indices.tolist()
            obj_msg.y_indices = y_indices.tolist()

            # Append the object segment to the array
            objects_msg.objects.append(obj_msg)
        else:
            rospy.logwarn("Segmentation result has no masks")

    # Publish the ObjectsSegment message
    if objects_msg.objects:
        rospy.loginfo(f"Publishing {len(objects_msg.objects)} segmented objects")
        objects_segment_pub.publish(objects_msg)

    # Segmentation Visualization
    if seg_image_pub.get_num_connections() > 0:
        try:
            # Generate and publish the annotated segmentation image
            seg_annotated = seg_result[0].plot(show=False)
            seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding="rgb8"))
            rospy.loginfo("Segmentation image published")
        except Exception as e:
            rospy.logerr(f"Error while publishing segmentation image: {e}")

# Subscriber
rgb_sub = rospy.Subscriber("/realsense/color/image_raw", Image, callback)

# Keep the node running
rospy.spin()

