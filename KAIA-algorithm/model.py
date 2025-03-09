import torch
import torch.nn as nn
import torch.nn.functional as F

class GEON_MODEL(nn.Module):
    def __init__(self, k=20, num_classes=8):
        super(GEON_MODEL, self).__init__()
        self.k = k

        self.conv1 = nn.Conv2d(2 * 4, 64, kernel_size=1, bias=False)  
        self.conv2 = nn.Conv2d(128, 64, kernel_size=1, bias=False)  
        self.conv3 = nn.Conv2d(128, 128, kernel_size=1, bias=False)   
        self.conv4 = nn.Conv2d(256, 256, kernel_size=1, bias=False)   
        self.conv5 = nn.Conv1d(512, 1024, kernel_size=1, bias=False) 

        self.bn1 = nn.BatchNorm2d(64)
        self.bn2 = nn.BatchNorm2d(64)
        self.bn3 = nn.BatchNorm2d(128)
        self.bn4 = nn.BatchNorm2d(256)
        self.bn5 = nn.BatchNorm1d(1024)

        self.linear1 = nn.Linear(1024, 512, bias=False)
        self.bn6 = nn.BatchNorm1d(512)
        self.dp1 = nn.Dropout(p=0.5)
        self.linear2 = nn.Linear(512, 256)
        self.bn7 = nn.BatchNorm1d(256)
        self.dp2 = nn.Dropout(p=0.5)
        self.linear3 = nn.Linear(256, num_classes)


    def get_graph_feature(self, x, k=None):
        if k is None:
            k = self.k

        batch_size, num_dims, num_points = x.size()
        if k > num_points:
            k = num_points  

        x = x.view(batch_size, num_dims, num_points)
        idx = knn(x, k=k)  
        device = x.device  

        idx_base = torch.arange(0, batch_size, device=device).view(-1, 1, 1) * num_points
        idx = idx + idx_base
        idx = idx.view(-1)

        feature = x.transpose(2, 1).contiguous()  # (batch_size, num_points, num_dims)
        feature = feature.view(batch_size * num_points, -1)[idx, :]
        feature = feature.view(batch_size, num_points, k, num_dims)
        x = x.transpose(2, 1).contiguous().view(batch_size, num_points, 1, num_dims)
        x = x.repeat(1, 1, k, 1)

        feature = torch.cat((feature - x, x), dim=3).permute(0, 3, 1, 2).contiguous()
        return feature


    def forward(self, x):
        batch_size = x.size(0)
        
        x = self.get_graph_feature(x)  
        x = F.relu(self.bn1(self.conv1(x)))  
        x1 = x.max(dim=-1, keepdim=False)[0]
        
        x = self.get_graph_feature(x1)  
        x = F.relu(self.bn2(self.conv2(x)))  
        x2 = x.max(dim=-1, keepdim=False)[0]
        
        x = self.get_graph_feature(x2)  
        x = F.relu(self.bn3(self.conv3(x)))  
        x3 = x.max(dim=-1, keepdim=False)[0]

        x = self.get_graph_feature(x3)  
        x = F.relu(self.bn4(self.conv4(x)))  
        x4 = x.max(dim=-1, keepdim=False)[0]

        x = torch.cat((x1, x2, x3, x4), dim=1)  

        x = F.relu(self.bn5(self.conv5(x)))  
        x = F.adaptive_max_pool1d(x, 1).view(batch_size, -1)
        x = F.relu(self.bn6(self.linear1(x)))
        x = self.dp1(x)
        x = F.relu(self.bn7(self.linear2(x)))
        x = self.dp2(x)
        x = self.linear3(x)

        return x


def knn(x, k):
    inner = -2 * torch.matmul(x.transpose(2, 1), x)
    xx = torch.sum(x**2, dim=1, keepdim=True)
    pairwise_distance = -xx - inner - xx.transpose(2, 1)

    idx = pairwise_distance.topk(k=k, dim=-1)[1]
    return idx

"""
## model test ###
print("GEON Model Test")

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"{device = }")

k = 10 # 20
model = GEON_MODEL(k=10, num_classes=8)
model = model.to(device)

batch_size = 64 #32
num_points = 126 #300
num_dims = 4
x = torch.rand(batch_size, num_dims, num_points).to(device)
print(f"{x.shape = }")

output = model(x)
print(f"Output shape: {output.shape}")
"""