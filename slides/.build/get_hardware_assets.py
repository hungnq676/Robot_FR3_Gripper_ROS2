import urllib.request,json,re
from pathlib import Path
base=Path('/home/duong-12a/fr3_hungros2/slides/assets')
def get(u):
 return urllib.request.urlopen(urllib.request.Request(u,headers={'User-Agent':'Mozilla/5.0'}),timeout=25).read()
u='https://docs.hiwonder.com/projects/BusLinker/en/latest/_static/media/chapter_1/section_1/image_1.jpg';(base/'buslinker_v3.jpg').write_bytes(get(u));print('bus saved')
for url in ['https://www.hiwonder.com/products/htd-35h.js','https://www.hiwonder.com/products/htd-35h.json']:
 try:
  data=json.loads(get(url));p=data.get('product',data);imgs=p.get('images',[]);print('servo candidates',imgs[:2]);u=imgs[0];u=u.get('src') if isinstance(u,dict) else u
  if u.startswith('//'):u='https:'+u
  (base/'hiwonder_htd35h.jpg').write_bytes(get(u));print('servo saved',u);break
 except Exception as e:print(type(e).__name__,str(e)[:100])
try:
 u='https://www.espboards.dev/esp32/esp32cam/';s=get(u).decode();imgs=re.findall(r'<img[^>]+src=[\"\']([^\"\']+)',s);print('esp candidates',imgs[:10]);Path('/tmp/fr3_esp.html').write_text(s)
except Exception as e:print('esp',e)
