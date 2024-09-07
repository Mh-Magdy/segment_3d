Header header           # Standard ROS message header
string class_name       # Class name of the detected object (e.g., "person")
float32 probability     # Probability of the detected class
int32[] x_indices       # X indices of the mask in the original image
int32[] y_indices       # Y indices of the mask in the original image

