import numpy as np
from mnist_pipeline import MiniCNNWeights, load_mem_int16, CpuMockSage16
W=MiniCNNWeights("."); sage=CpuMockSage16()
imgs=load_mem_int16("images.mem").reshape(-1,26,26); labs=load_mem_int16("labels.mem")
def onchip(img):
    pool=np.zeros((8,6,6),np.int32); img16=img.astype(np.int16)
    for co in range(8):
        for ty in range(6):
            for tx in range(6):
                t=sage.conv3x3(img16[4*ty:4*ty+6,4*tx:4*tx+6],W.conv1[co,0]); s=int(t.sum())
                pool[co,ty,tx]=(s>>4) if s>=0 else -((-s)>>4)
    p16=np.clip(pool,-32768,32767).astype(np.int16); c2=np.zeros((16,4,4),np.int32)
    for co in range(16):
        for ci in range(8): c2[co]+=sage.conv3x3(p16[ci],W.conv2[co,ci])
    sh=c2>>3; flat=np.clip(sh.reshape(-1),-32768,32767).astype(np.int32)
    def fc(x,w):
        L,N=w.shape; A=np.tile(np.clip(x,-32768,32767).astype(np.int16),(4,1)); y=np.zeros(N,np.int32)
        for oc in range(0,N,4):
            acc=np.zeros((4,4),np.int32)
            for lb in range(0,L,4):
                acc+=sage.matmul(A[:,lb:lb+4].flatten(),w[lb:lb+4,oc:oc+4].flatten()).reshape(4,4)
            y[oc:oc+4]=acc[0]
        return y
    fc1=np.maximum(fc(flat,W.w1.astype(np.int32)),0)
    fc2=fc(np.clip(fc1,-32768,32767),W.w2.astype(np.int32))
    return int(np.argmax(fc2[:10])),fc2
# dump first 12 images as hex .mem (one image per file) + a manifest of golden digits
digs=[]
for i in range(12):
    with open(f"img{i}.mem","w") as f:
        for v in imgs[i].reshape(-1): f.write(f"{int(v)&0xFFFF:04x}\n")
    d,fc2=onchip(imgs[i]); digs.append(d)
with open("golden.txt","w") as f:
    for i,d in enumerate(digs): f.write(f"{i} {d} {int(labs[i])}\n")
print("golden digits (idx pred truth):")
print(open("golden.txt").read())
