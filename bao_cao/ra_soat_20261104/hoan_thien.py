from docx import Document
from docx.shared import Cm,Pt
from docx.oxml.ns import qn
import re,unicodedata
f='bao_cao/Bao_cao_NCKH_FR3_ra_soat.docx';d=Document(f)
def norm(s):return re.sub(r'\s+','',unicodedata.normalize('NFC',s)).lower().replace('–','-').replace('−','-')
pages=open('/tmp/report-v2-pages.txt').read().split('\f')
# TOC has a cached result. Refresh its displayed page numbers from rendered headings.
last=1;unmatched=[]
for p in d.paragraphs:
 if not p.style or not p.style.name.lower().startswith('toc ') or '\t' not in p.text:continue
 title=p.text.rsplit('\t',1)[0];target=norm(title)
 candidates=[]
 for i,pg in enumerate(pages):
  if 4<=i+1<=8:continue
  if i+1<last:continue
  if target in norm(pg):candidates.append(i+1)
 if not candidates:unmatched.append(title);continue
 n=candidates[0];last=n
 for r in reversed(p.runs):
  if r.text.strip().isdigit():r.text=str(n);break
print('TOC unmatched',unmatched)
# Allocate enough width for evaluation text.
t=d.tables[34];t.autofit=False
widths=[.9,2.5,9.5,3.1]
for col,w in zip(t.columns,widths):col.width=Cm(w)
for row in t.rows:
 for cell,w in zip(row.cells,widths):
  cell.width=Cm(w)
  for p in cell.paragraphs:
   for r in p.runs:r.font.size=Pt(10.5)
# Protect the cached TOC on open; the field remains available for manual refresh.
for e in d.settings.element.findall(qn('w:updateFields')):e.set(qn('w:val'),'false')
d.save(f)
