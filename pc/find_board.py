import socket, struct, sys, time

def query(names, wait=4.0):
    pkts=[]
    for nm in names:
        q=b''.join(bytes([len(l)])+l.encode() for l in nm.split('.'))+b'\x00'
        pkts.append(struct.pack('!HHHHHH',0,0,1,0,0,0)+q+struct.pack('!HH',1,1))
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    s.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,255)
    s.bind(('',0)); s.settimeout(0.5)
    for p in pkts:
        for _ in range(2): s.sendto(p,('224.0.0.251',5353))
    found={}
    t0=time.time()
    while time.time()-t0<wait:
        try: data,addr=s.recvfrom(9000)
        except socket.timeout: continue
        try: found.update(parse(data))
        except Exception: pass
    return found

def rdname(d,i):
    parts=[]
    while True:
        if i>=len(d): return '.'.join(parts),i
        L=d[i]
        if L==0: return '.'.join(parts),i+1
        if L&0xC0==0xC0:
            ptr=struct.unpack('!H',d[i:i+2])[0]&0x3FFF
            sub,_=rdname(d,ptr); parts.append(sub); return '.'.join(parts),i+2
        parts.append(d[i+1:i+1+L].decode('utf-8','replace')); i+=1+L

def parse(d):
    out={}
    qd,an,ns,ar=struct.unpack('!HHHH',d[4:12])
    i=12
    for _ in range(qd):
        _,i=rdname(d,i); i+=4
    for _ in range(an+ns+ar):
        nm,i=rdname(d,i)
        if i+10>len(d): break
        t,c,ttl,rl=struct.unpack('!HHIH',d[i:i+10]); i+=10
        rd=d[i:i+rl]; i+=rl
        if t==1 and rl==4:
            out[nm]=socket.inet_ntoa(rd)
    return out

names=sys.argv[1:] or ['peoples.local','raspberrypi.local','ubuntu.local','mowbot.local']
r=query(names)
if r:
    for k,v in sorted(r.items()): print(f"  {k:28s} {v}")
else:
    print("  aucune reponse mDNS")
