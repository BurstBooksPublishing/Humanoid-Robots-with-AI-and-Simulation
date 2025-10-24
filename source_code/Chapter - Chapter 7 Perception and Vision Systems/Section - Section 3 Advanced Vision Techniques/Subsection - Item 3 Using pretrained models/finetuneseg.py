import torch
import torchvision
# load pretrained semantic segmentation model
model = torchvision.models.segmentation.fcn_resnet50(pretrained=True)  # pretrained backbone
# replace classifier head for N classes
num_classes = 5  # e.g., human, obstacle, floor, object, background
model.classifier[4] = torch.nn.Conv2d(512, num_classes, kernel_size=1)  # replace last conv
# freeze backbone parameters (feature extraction mode)
for name, param in model.backbone.named_parameters():
    param.requires_grad = False
# move to device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)
# optimizer only for head parameters
optimizer = torch.optim.Adam(filter(lambda p: p.requires_grad, model.parameters()), lr=1e-4)
criterion = torch.nn.CrossEntropyLoss(ignore_index=255)  # mask out invalid pixels
# training loop (sketch)
for epoch in range(10):
    model.train()
    for imgs, masks in dataloader:
        imgs, masks = imgs.to(device), masks.to(device)
        preds = model(imgs)['out']
        loss = criterion(preds, masks)
        optimizer.zero_grad()
        loss.backward()  # gradient flows only to head
        optimizer.step()
# after N epochs, unfreeze some layers and reduce lr for fine-tuning