import numpy as np
from mnist_pipeline import MiniCNNWeights, load_mem_int16, CpuMockSage16, CONV2_SHIFT

W = MiniCNNWeights(".")
sage = CpuMockSage16()
imgs = load_mem_int16("images.mem").reshape(-1,26,26)
labs = load_mem_int16("labels.mem")

def cpu_predict(img):
    # faithful reimpl of mnist_pipeline.predict, capturing intermediates
    x = img[None,:,:].astype(np.int32)
    c1 = np.zeros((8,24,24),np.int32)
    for co in range(8):
        for ky in range(3):
            for kx in range(3):
                c1[co]+=x[0,ky:ky+24,kx:kx+24]*int(W.conv1[co,0,ky,kx])
    s=c1.reshape(8,6,4,6,4).sum(axis=(2,4))
    pool=np.trunc(s/16).astype(np.int32)                 # (8,6,6) trunc toward 0
    c2=np.zeros((16,4,4),np.int32)
    for co in range(16):
        for ci in range(8):
            for ky in range(3):
                for kx in range(3):
                    c2[co]+=pool[ci,ky:ky+4,kx:kx+4]*int(W.conv2[co,ci,ky,kx])
    sh=c2>>CONV2_SHIFT                                    # floor shift
    flat=sh.reshape(-1).astype(np.int32)                 # CHW
    fc1=flat@W.w1.astype(np.int32); fc1=np.maximum(fc1,0)
    fc2=fc1@W.w2.astype(np.int32)
    return int(np.argmax(fc2[:10])), dict(pool=pool,c2=c2,sh=sh,fc1=fc1,fc2=fc2)

def onchip_predict(img):
    # everything via the 6x6->4x4 conv primitive (sage.conv3x3) + hw-style ops
    # --- Conv1 fused with Pool4: each 4x4 conv tile == one 4x4 pool window ---
    pool=np.zeros((8,6,6),np.int32)
    img16=np.clip(img,-32768,32767).astype(np.int16)
    for co in range(8):
        for ty in range(6):
            for tx in range(6):
                patch=img16[4*ty:4*ty+6, 4*tx:4*tx+6]          # 6x6
                tile=sage.conv3x3(patch, W.conv1[co,0])        # 4x4 (==pool window)
                s=int(tile.sum())
                # trunc toward zero by /16
                pool[co,ty,tx]= (s>>4) if s>=0 else -((-s)>>4)
    # --- Conv2: accumulate over 8 input ch ---
    pool16=np.clip(pool,-32768,32767).astype(np.int16)
    c2=np.zeros((16,4,4),np.int32)
    for co in range(16):
        for ci in range(8):
            c2[co]+=sage.conv3x3(pool16[ci], W.conv2[co,ci])
    sh=c2>>CONV2_SHIFT
    flat=sh.reshape(-1).astype(np.int32)
    # --- FC1 via 4x4 matmul tiles, replicate-row trick ---
    def fc(x, w):
        L,N=w.shape
        L4=((L+3)//4)*4; N4=((N+3)//4)*4
        xp=np.zeros(L4,np.int16); xp[:L]=np.clip(x,-32768,32767).astype(np.int16)
        wp=np.zeros((L4,N4),np.int16); wp[:L,:N]=w
        A=np.tile(xp,(4,1)); y=np.zeros(N,np.int32)
        for oc in range(0,N,4):
            acc=np.zeros((4,4),np.int32)
            for lb in range(0,L4,4):
                c=sage.matmul(A[:,lb:lb+4].flatten(), wp[lb:lb+4,oc:oc+4].flatten())
                acc+=c.reshape(4,4)
            y[oc:min(oc+4,N)]=acc[0,:min(oc+4,N)-oc]
        return y
    fc1=fc(flat,W.w1.astype(np.int32)); fc1=np.maximum(fc1,0)
    fc2=fc(fc1,W.w2.astype(np.int32))
    return int(np.argmax(fc2[:10])), dict(pool=pool,c2=c2,sh=sh,fc1=fc1,fc2=fc2)

N=200
ok_cpu=ok_match=0
mismatch=[]
for i in range(N):
    pc,ic=cpu_predict(imgs[i])
    po,io=onchip_predict(imgs[i])
    if pc==int(labs[i]): ok_cpu+=1
    if pc==po: ok_match+=1
    else: mismatch.append(i)
    for k in ic:
        if not np.array_equal(ic[k],io[k]):
            print(f"img{i} stage {k} DIFFERS max|d|={np.abs(ic[k]-io[k]).max()}")
print(f"cpu vs labels: {ok_cpu}/{N}")
print(f"onchip vs cpu (digit match): {ok_match}/{N}  mismatches={mismatch[:10]}")
