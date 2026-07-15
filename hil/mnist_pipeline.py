"""
mnist_pipeline.py — MiniCNN-v2 inference via SAGE-16

EMPIRICALLY VALIDATED TOPOLOGY (matches paper's preds.txt at 9980/10000):

    26x26 int16 input
        → Conv1 (3x3, 1→8, NO bias, NO ReLU)     → 8 x 24 x 24
        → Pool4 (4x4 avg-pool, divide-by-16)      → 8 x 6 x 6
        → Conv2 (3x3, 8→16, NO bias, NO ReLU)    → 16 x 4 x 4
        → >> 3      (arithmetic right shift)
        → flatten (CHW order)                     → 256
        → FC1 (256 → 32, no bias)
        → ReLU
        → FC2 (32 → 12, last 2 are padding)
        → argmax over first 10                    → digit 0..9

The paper's text describes "Conv1→ReLU, Conv2→ReLU" but the exported
weights only match the test predictions when ReLU is OMITTED after both
convolutions and applied only between FC1 and FC2. This is consistent
with a model where the ReLU's of the conv stack got folded/absorbed into
the export quantization; the result is a single ReLU between the linear
layers, which is enough nonlinearity for MNIST.

The `info.txt` value `conv2_shift: 4` is also off-by-one for what the
exported predictions actually use: empirically the right shift is 3.
We use 3.

Weight layouts (verified):
    conv1_k.mem :   72 = 8*1*3*3,    stored as (8, 1, 3, 3)   OIHW
    conv2_k.mem : 1152 = 16*8*3*3,   stored as (16, 8, 3, 3)  OIHW
    w1.mem      : 8192 = 32*256,     stored as (32, 256), transpose to (256, 32) for matmul
    w2.mem      :  384 = 12*32,      stored as (12, 32),  transpose to (32, 12).
                                     The 12 outputs are the 10 classes + 2 padding.

How we use SAGE-16:
    - Conv2 (8 in, 16 out, 3x3 on 6x6 input) is the natural fit for the
      conv3x3 kernel: each SAGE call handles one (input_channel, output_channel)
      pair, producing a 4x4 partial sum. We do 16 * 8 = 128 conv calls.
    - FC1 and FC2 use the matmul kernel, tiled into 4x4 GEMM blocks.
      To compute y = x @ W for a 1xL row vector we replicate x into 4
      rows and pull off the first row of the result. This wastes 3/4 of
      each MAC group but keeps the code simple.

Conv1 stays on the CPU (24x24 output doesn't tile cleanly into 4x4 blocks
and would need a tiled-with-overlap scheme; it's only 4608 MACs and runs
in ~1ms in numpy).
"""

import numpy as np
from pathlib import Path


CONV2_SHIFT = 3   # empirically-determined shift between conv2 and FC1


# -----------------------------------------------------------------------------
# weight loading
# -----------------------------------------------------------------------------

def load_mem_int16(path):
    """Load a .mem file of 4-digit hex words as signed int16."""
    with open(path) as f:
        vals = [int(line.strip(), 16) for line in f if line.strip()]
    arr = np.array(vals, dtype=np.uint16).astype(np.int32)
    arr = np.where(arr >= 0x8000, arr - 0x10000, arr).astype(np.int16)
    return arr


class MiniCNNWeights:
    """Holds the trained weights in convenient numpy shapes."""

    def __init__(self, weights_dir):
        d = Path(weights_dir)
        self.conv1 = load_mem_int16(d / "conv1_k.mem").reshape(8, 1, 3, 3)
        self.conv2 = load_mem_int16(d / "conv2_k.mem").reshape(16, 8, 3, 3)
        # FC weights are stored output-major; transpose so x @ w semantics work.
        self.w1 = load_mem_int16(d / "w1.mem").reshape(32, 256).T   # (256, 32)
        self.w2 = load_mem_int16(d / "w2.mem").reshape(12, 32).T    # (32, 12)

    def __repr__(self):
        return (f"MiniCNNWeights(conv1={self.conv1.shape}, conv2={self.conv2.shape}, "
                f"w1={self.w1.shape}, w2={self.w2.shape})")


# -----------------------------------------------------------------------------
# pure-numpy reference operations
# -----------------------------------------------------------------------------

def conv2d_int(x, k):
    """Valid 2D conv (no padding, stride 1), integer arithmetic.

    x: (C_in, H, W) int
    k: (C_out, C_in, 3, 3) int
    returns: (C_out, H-2, W-2) int32
    """
    C_in, H, W = x.shape
    C_out, _, kH, kW = k.shape
    assert kH == 3 and kW == 3 and _ == C_in
    x = x.astype(np.int32)
    k = k.astype(np.int32)
    out_h, out_w = H - 2, W - 2
    out = np.zeros((C_out, out_h, out_w), dtype=np.int32)
    for co in range(C_out):
        for ci in range(C_in):
            for ky in range(3):
                for kx in range(3):
                    out[co] += x[ci, ky:ky+out_h, kx:kx+out_w] * k[co, ci, ky, kx]
    return out


def pool4_avg(x):
    """4x4 average pool with stride 4, integer truncation toward zero.

    x: (C, H, W) int32, H and W divisible by 4
    returns: (C, H/4, W/4) int32
    """
    C, H, W = x.shape
    assert H % 4 == 0 and W % 4 == 0
    s = x.reshape(C, H // 4, 4, W // 4, 4).sum(axis=(2, 4))
    return np.trunc(s / 16).astype(np.int32)


def relu(x):
    return np.maximum(x, 0)


# -----------------------------------------------------------------------------
# SAGE-16-accelerated layers
# -----------------------------------------------------------------------------

def conv2_via_sage(x, k, sage16):
    """Conv2: 8 in, 16 out, 3x3, on a 6x6 input → 4x4 output per channel.

    SAGE's conv3x3 kernel takes one 6x6 image and one 3x3 kernel, returns
    4x4. We accumulate over the 8 input channels in software.

    x: (8, 6, 6) int (post-pool feature maps; small values, fit int16)
    k: (16, 8, 3, 3) int16
    returns: (16, 4, 4) int32
    """
    assert x.shape == (8, 6, 6), f"expected (8,6,6), got {x.shape}"
    assert k.shape == (16, 8, 3, 3), f"expected (16,8,3,3), got {k.shape}"

    x_int16 = np.clip(x, -32768, 32767).astype(np.int16)

    out = np.zeros((16, 4, 4), dtype=np.int32)
    for co in range(16):
        for ci in range(8):
            out[co] += sage16.conv3x3(x_int16[ci], k[co, ci])
    return out


def matmul_via_sage(x, w, sage16):
    """Compute y = x @ w where x is a 1D vector of length L, w is (L, N).

    SAGE's matmul kernel computes a 4x4 GEMM: A (4x4) @ B (4x4) → C (4x4),
    presented as 16-element flat int16 arrays in/out.

    We tile by:
      - padding L and N to multiples of 4
      - replicating x into 4 rows to form a (4, L) row-matrix
      - for each output column block (4 outputs):
          accumulate (4,4) @ (4,4) over L/4 inner blocks
          take the first row as the answer (rows 1-3 are duplicates)

    Slightly wasteful (we compute 4x more MACs than strictly needed) but
    correct and simple. For MiniCNN-v2 the totals are:
      FC1 (256→32): 64 inner blocks × 8 output blocks = 512 SAGE calls
      FC2 (32→12):   8 inner blocks × 3 output blocks =  24 SAGE calls
    """
    L = x.shape[0]
    L_w, N = w.shape
    assert L == L_w

    L4 = ((L + 3) // 4) * 4
    N4 = ((N + 3) // 4) * 4
    x_pad = np.zeros(L4, dtype=np.int16)
    x_pad[:L] = np.clip(x, -32768, 32767).astype(np.int16)
    w_pad = np.zeros((L4, N4), dtype=np.int16)
    w_pad[:L, :N] = w

    A_full = np.tile(x_pad, (4, 1))

    y = np.zeros(N, dtype=np.int32)
    for oc in range(0, N, 4):
        oc_end = min(oc + 4, N)
        acc = np.zeros((4, 4), dtype=np.int32)
        for lb in range(0, L4, 4):
            A_block = A_full[:, lb:lb+4]                 # (4, 4)
            B_block = w_pad[lb:lb+4, oc:oc+4]            # (4, 4)
            c = sage16.matmul(A_block.flatten(), B_block.flatten())
            acc += c.reshape(4, 4)
        y[oc:oc_end] = acc[0, :oc_end - oc]
    return y


# -----------------------------------------------------------------------------
# end-to-end inference
# -----------------------------------------------------------------------------

def predict(image_26x26, weights, sage16, accelerate=("conv2", "fc1", "fc2")):
    """Run full MiniCNN-v2 inference on one 26x26 image.

    image_26x26: 2D int array, values in roughly [-32, 31]
    weights: MiniCNNWeights
    sage16: object with .conv3x3() and .matmul() methods
    accelerate: which layers to run via SAGE-16 (others fall back to numpy)
    returns: predicted digit (0..9)
    """
    assert image_26x26.shape == (26, 26), f"expected (26,26), got {image_26x26.shape}"

    # Conv1 stays on CPU
    x = image_26x26[None, :, :].astype(np.int32)
    x = conv2d_int(x, weights.conv1)               # (8, 24, 24)
    # NO ReLU (verified empirically)

    x = pool4_avg(x)                               # (8, 6, 6)

    if "conv2" in accelerate:
        x = conv2_via_sage(x, weights.conv2, sage16)
    else:
        x = conv2d_int(x, weights.conv2)           # (16, 4, 4)
    # NO ReLU (verified empirically)

    x = x >> CONV2_SHIFT                           # right-shift by 3

    x_flat = x.reshape(-1).astype(np.int32)        # (256,) CHW order

    if "fc1" in accelerate:
        x_fc1 = matmul_via_sage(x_flat, weights.w1, sage16)
    else:
        x_fc1 = x_flat @ weights.w1.astype(np.int32)
    x_fc1 = relu(x_fc1)                            # ReLU between FCs only

    if "fc2" in accelerate:
        x_fc2 = matmul_via_sage(x_fc1, weights.w2, sage16)
    else:
        x_fc2 = x_fc1 @ weights.w2.astype(np.int32)

    return int(np.argmax(x_fc2[:10]))              # ignore padding outputs


def predict_batch(images, weights, sage16, accelerate=("conv2", "fc1", "fc2"),
                  progress=False):
    """Predict over a (N, 26, 26) array. Returns (N,) int array."""
    preds = np.zeros(len(images), dtype=np.int32)
    for i, img in enumerate(images):
        preds[i] = predict(img, weights, sage16, accelerate=accelerate)
        if progress and (i+1) % 100 == 0:
            print(f"  {i+1}/{len(images)}")
    return preds


# -----------------------------------------------------------------------------
# CPU-only mock for laptop testing (no FPGA required)
# -----------------------------------------------------------------------------

class CpuMockSage16:
    """Numpy stand-in for the AXI-Lite SAGE-16 IP. Lets you develop and
    test the pipeline without the board."""

    def conv3x3(self, img_6x6, k_3x3):
        img = np.asarray(img_6x6, dtype=np.int32)
        k = np.asarray(k_3x3, dtype=np.int32)
        out = np.zeros((4, 4), dtype=np.int32)
        for oy in range(4):
            for ox in range(4):
                out[oy, ox] = int((img[oy:oy+3, ox:ox+3] * k).sum())
        return out

    def matmul(self, a_16, b_16):
        A = np.asarray(a_16, dtype=np.int32).reshape(4, 4)
        B = np.asarray(b_16, dtype=np.int32).reshape(4, 4)
        return (A @ B).flatten()

    def quat(self, q1_4, q2_16):
        # Not used by MNIST. The IMU path has its own Hamilton-product helper.
        return np.zeros(16, dtype=np.int32)
