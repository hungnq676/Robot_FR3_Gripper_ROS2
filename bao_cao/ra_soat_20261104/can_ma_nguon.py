from docx import Document
from docx.shared import Pt
import ast
f='bao_cao/Bao_cao_NCKH_FR3_ra_soat.docx';d=Document(f)
blocks=[];cur=[]
for p in d.paragraphs:
 if any(r.font.name=='Courier New' and r.font.size and r.font.size.pt==9 for r in p.runs):cur.append(p)
 elif cur:blocks.append(cur);cur=[]
if cur:blocks.append(cur)
patch={
'        R = np.column_stack([y, z, x]) @ Rotation.from_euler("z", turn).as_matrix()':'        R = (np.column_stack([y, z, x])\n             @ Rotation.from_euler("z", turn).as_matrix())',
'            low = (R @ FINGER_CORNERS.T)[2].min()   # lowest finger corner below the TCP':'            # Lowest finger corner below the TCP\n            low = (R @ FINGER_CORNERS.T)[2].min()',
'            limit = (T[2, 3] - TCP_FROM_TIPS * x[2] + low - (TABLE_TOP_Z + CORNER_ABOVE_TABLE)) / -x[2]':'            limit = (\n                T[2, 3] - TCP_FROM_TIPS * x[2] + low\n                - (TABLE_TOP_Z + CORNER_ABOVE_TABLE)\n            ) / -x[2]',
'    q = (np.load(path).astype(float) - P[:3, 3]) @ P[:3, :3]     # x closing, y finger width, z approach':'    # x: closing; y: finger width; z: approach\n    q = (np.load(path).astype(float) - P[:3, 3]) @ P[:3, :3]',
'    slab = (np.abs(q[:, 1]) < JAW_HALF_Y) & (q[:, 2] > TCP_FROM_TIPS - 0.040) & (q[:, 2] < TCP_FROM_TIPS)':'    slab = (\n        (np.abs(q[:, 1]) < JAW_HALF_Y)\n        & (q[:, 2] > TCP_FROM_TIPS - 0.040)\n        & (q[:, 2] < TCP_FROM_TIPS)\n    )',
'    return 2000 * np.abs(x).max(), 1000 * (x.max() - x.min()), int(inside.sum()), int(hit.sum())':'    return (\n        2000 * np.abs(x).max(), 1000 * (x.max() - x.min()),\n        int(inside.sum()), int(hit.sum())\n    )',
'    return float(np.degrees(np.max(np.abs(np.array(vals) - vals[0])))) if vals else 0.0':'    return (\n        float(np.degrees(np.max(np.abs(np.array(vals) - vals[0]))))\n        if vals else 0.0\n    )',
'    if p is not None and abs(p - target) > 3:          # stalled before the target: pressing on the object':'    # Stalled before target: pressing on the object\n    if p is not None and abs(p - target) > 3:',
'    p = self.move(CLOSE_POS, ms)                       # not touching yet: close until it stalls':'    # Not touching yet: close until it stalls\n    p = self.move(CLOSE_POS, ms)',
'        self._packet(1, struct.pack("<HH", int(max(0, min(1000, p + sign * units))), 200))':'        self._packet(1, struct.pack(\n            "<HH", int(max(0, min(1000, p + sign * units))), 200\n        ))',
'    return dict(center=np.array([cx, cy, (top + table_z) / 2]), size=np.array([w, h, top - table_z]),':'    return dict(\n        center=np.array([cx, cy, (top + table_z) / 2]),\n        size=np.array([w, h, top - table_z]),'
}
for block in blocks:
 before='\n'.join(p.text for p in block)
 if not before.startswith('def '): continue
 for i,p in enumerate(block):
  if p.text in patch:
   new=patch[p.text];p.runs[0].text=new
   for r in p.runs[1:]:r.text=''
  p.paragraph_format.keep_with_next=i<len(block)-1
 after='\n'.join(p.text for p in block)
 assert ast.dump(ast.parse(before))==ast.dump(ast.parse(after))
d.save(f)
print('Code blocks formatted, AST unchanged:',len(blocks))
