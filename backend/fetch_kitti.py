"""Extract OXTS-only files from an official KITTI archive via HTTP range requests."""
import io, struct, pathlib, requests, zipfile
URL='https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0005/2011_09_26_drive_0005_sync.zip'
OUT=pathlib.Path(__file__).parent/'data'/'oxts'; OUT.mkdir(parents=True, exist_ok=True)
s=requests.Session()
def grab(a,b):
 r=s.get(URL,headers={'Range':f'bytes={a}-{b}'},timeout=60);r.raise_for_status()
 if r.status_code!=206:raise RuntimeError('Range unavailable')
 return r.content
head=s.head(URL,timeout=20);head.raise_for_status();length=int(head.headers['Content-Length'])
tail=grab(length-65536,length-1);p=tail.rfind(b'PK\x05\x06')
if p<0:raise RuntimeError('ZIP EOCD absent')
_,_,_,_,count,size,offset,_=struct.unpack_from('<IHHHHIIH',tail,p)
central=grab(offset,offset+size-1);i=0;entries=[]
for _ in range(count):
 v=struct.unpack_from('<IHHHHHHIIIHHHHHII',central,i);name=central[i+46:i+46+v[10]].decode();i+=46+v[10]+v[11]+v[12]
 if '/oxts/' in name and not name.endswith('/'):
  entries.append((name,v[8],v[-1]))
for name,compressed_size,loc in entries:
 # local header gives length of filename and extra data
 header=grab(loc,loc+29)
 if header[:4]!=b'PK\x03\x04':raise RuntimeError('Bad local header')
 name_size,extra_size=struct.unpack_from('<HH',header,26)
 start=loc+30+name_size+extra_size
 data=grab(start,start+compressed_size-1)
 # KITTI is stored, not compressed; if this changes, reject instead of silently corrupting
 dest=OUT/'timestamps.txt' if name.endswith('/oxts/timestamps.txt') else OUT/'data'/pathlib.Path(name).name
 dest.parent.mkdir(exist_ok=True)
 dest.write_bytes(data)
print(f'Fetched {len(entries)} OXTS files from {URL} ({sum(s for _,s,_ in entries):,} bytes).')
