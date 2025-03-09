import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# Hyperparameters
IMG_WIDTH = 1920
IMG_HEIGHT = 1080
NUM_ANCHORS = 36  # Number of row anchors
LANE_THRESHOLD = 0.2  # Lower threshold to detect more lanes

def generate_anchors(img_height, img_width, num_anchors):
    """Generates row-based anchors along the width of the image."""
    step = img_height // num_anchors
    anchors = [(i * step, j) for i in range(num_anchors) for j in range(0, img_width, img_width // 20)]
    return np.array(anchors)

# Generate anchors
anchors = generate_anchors(IMG_HEIGHT, IMG_WIDTH, NUM_ANCHORS)

def detect_lanes(image, model):
    """Perform lane detection using a neural network model."""
    image_resized = cv2.resize(image, (IMG_WIDTH, IMG_HEIGHT))
    image_tensor = torch.tensor(image_resized, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0) / 255.0
    
    with torch.no_grad():
        output = model(image_tensor)  # CNN 모델로 변경했으므로 reshape 불필요
    
    output = output.squeeze(0).numpy().flatten()  # Flatten to match anchors shape
    
    detected_lanes = anchors[output > LANE_THRESHOLD]  # Ensure correct indexing
    return detected_lanes

# CNN 기반 LaneNet
class LaneNet(nn.Module):
    def __init__(self):
        super(LaneNet, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5, stride=2, padding=2)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2, padding=2)
        self.conv3 = nn.Conv2d(32, 64, kernel_size=5, stride=2, padding=2)
        self.pool = nn.AdaptiveAvgPool2d((10, 10))  # Output fixed size
        self.fc1 = nn.Linear(64 * 10 * 10, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, NUM_ANCHORS * 20)  # Predict lane anchors

        self.relu = nn.ReLU()
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.pool(x)
        x = x.view(x.shape[0], -1)  # Flatten
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.sigmoid(self.fc3(x))
        return x.view(x.shape[0], NUM_ANCHORS, 20)  # Reshape to match anchor structure

# Initialize the model
model = LaneNet()
optimizer = optim.Adam(model.parameters(), lr=0.001)
criterion = nn.BCELoss()

# Load and preprocess the image
image = cv2.imread(r"C:\Users\JMP\Desktop\2025-02-03_09-15-40_frame_1302.png")
detected_lanes = detect_lanes(image, model)

# Draw detected lanes with larger circles
for lane in detected_lanes:
    cv2.circle(image, (lane[1], lane[0]), 8, (0, 255, 0), thickness=2)  # Larger circles, thicker

# Display the image with a fixed window size
cv2.namedWindow("Detected Lanes", cv2.WINDOW_NORMAL)
cv2.moveWindow("Detected Lanes", 400, 200)
cv2.resizeWindow("Detected Lanes", 1280, 720)
cv2.imshow("Detected Lanes", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
