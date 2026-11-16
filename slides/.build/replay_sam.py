from pathlib import Path
import numpy as np
from ultralytics import SAM
from PIL import Image
root=Path('/home/duong-12a/fr3_hungros2')
image=root/'nhat_ky/du_lieu/2026-11-02_021605_hop_gia_trang/camera_mau_dau.jpg'
model=SAM('/home/duong-12a/graspnet_runtime/mobile_sam.pt')
r=model(str(image),bboxes=[[344,85,439,213]],device='cpu',verbose=False)[0]
mask=r.masks.data[0].cpu().numpy()>0.5
np.save(root/'slides/assets/mobilesam_replay_mask.npy',mask)
Image.fromarray((mask*255).astype('uint8')).save(root/'slides/assets/mobilesam_replay_mask.png')
print('mask shape:',mask.shape,'pixels:',int(mask.sum()))
