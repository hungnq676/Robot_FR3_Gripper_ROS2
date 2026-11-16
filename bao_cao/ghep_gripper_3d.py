#!/usr/bin/env python3
"""Compose the VTK gripper renders (ve_gripper_3d.py) into one labelled figure: (a) open on the FR3 wrist,
(b) closed, (c) front view of frame + jaws with the 88 mm opening."""
import os
from PIL import Image, ImageChops, ImageDraw, ImageFont

H = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hinh")
F = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
font, bold = ImageFont.truetype(F, 34), ImageFont.truetype(F.replace("Sans.", "Sans-Bold."), 38)


def crop(name, pad=30):
    im = Image.open(os.path.join(H, name)).convert("RGB")
    bb = ImageChops.difference(im, Image.new("RGB", im.size, "white")).getbbox()
    return im.crop((max(bb[0] - pad, 0), max(bb[1] - pad, 0), min(bb[2] + pad, im.width), min(bb[3] + pad, im.height)))


a, b, c = crop("gripper3d_mo.png"), crop("gripper3d_khep.png"), crop("gripper3d_truoc.png")
h = 900
a, b = [im.resize((int(im.width * h / im.height), h)) for im in (a, b)]
c = c.resize((int(c.width * 620 / c.height), 620))
LBL = 560
W = a.width + b.width + LBL + 60
sheet = Image.new("RGB", (W, h + 80 + 50 + c.height + 90), "white")
sheet.paste(a, (0, 50)); sheet.paste(b, (a.width + 30, 50))
d = ImageDraw.Draw(sheet)
d.text((10, 0), "(a) Má mở hết (88 mm)", font=bold, fill="black")
d.text((a.width + 40, 0), "(b) Má khép hết (~30 mm)", font=bold, fill="black")
# callouts on (b), right side: fractions of the panel height
x0 = a.width + 30 + b.width
for frac, txt in ((0.10, "Khâu cổ tay J6 của FR3"), (0.26, "Mặt bích lắp dụng cụ"),
                  (0.43, "Servo + mạch ESP32\n(hình trụ minh họa,\nthực tế khác)"), (0.66, "Khung gripper in 3D"), (0.86, "Hai má kẹp (in 3D)")):
    y = 50 + int(frac * h)
    d.line((x0 - 60, y, x0 + 10, y), fill=(60, 60, 60), width=3)
    d.text((x0 + 20, y - 20), txt, font=font, fill="black")
y0 = h + 80
sheet.paste(c, ((W - c.width) // 2, y0 + 50))
d.text(((W - c.width) // 2, y0), "(c) Nhìn chính diện khung và hai má", font=bold, fill="black")
yb = y0 + 50 + c.height + 20
d.line(((W - c.width) // 2 + 140, yb, (W + c.width) // 2 - 140, yb), fill="black", width=3)
d.text((W // 2 - 190, yb + 10), "khoảng mở tối đa 88 mm", font=font, fill="black")
sheet.save(os.path.join(H, "gripper_3d.png"))
print(sheet.size)
