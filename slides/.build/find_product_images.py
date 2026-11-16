import urllib.request,re
pages={'servo':'https://www.hiwonder.com/products/htd-35h','bus':'https://docs.hiwonder.com/projects/BusLinker/en/latest/','esp':'https://www.espboards.dev/esp32/esp32cam/'}
for k,u in pages.items():
 s=urllib.request.urlopen(u,timeout=25).read().decode();open('/tmp/fr3_'+k+'.html','w').write(s)
 print(k)
 for pat in [r'<meta[^>]+(?:property|name)=[\"\']og:image[\"\'][^>]+content=[\"\']([^\"\']+)',r'<img[^>]+src=[\"\']([^\"\']+)']:
  for x in re.findall(pat,s)[:12]:print(x[:300])
