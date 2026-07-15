"""
preprocess.py — Convert a webcam frame into MiniCNN-v2's expected input.

MiniCNN-v2 expects: 26x26 int16, values in approximately [-32, 31], with
background = -32 and digit strokes positive. The training data was
quantized to ~6-bit signed.

Pipeline:
  1. Grayscale conversion (if RGB)
  2. Adaptive threshold to separate digit from background (handles
     varied lighting)
  3. Invert if necessary (digit should be lighter than background)
  4. Find largest connected component (the digit)
  5. Crop to bounding box with padding
  6. Center on a square canvas (preserve aspect ratio)
  7. Resize to 26x26
  8. Quantize from [0, 255] to [-32, 31]

This module has minimal dependencies (numpy + Pillow for image I/O,
OpenCV for the find-the-digit logic). Both are available on regular
Linux. The PYNQ board doesn't run this — it runs on the laptop side
of the network setup.
"""

import numpy as np

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


def _grayscale(img):
    """Accepts numpy array; converts to single-channel uint8 grayscale."""
    img = np.asarray(img)
    if img.ndim == 3:
        if HAVE_CV2:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Fallback: ITU-R BT.601 luminance
        return (0.299 * img[..., 2] + 0.587 * img[..., 1]
                + 0.114 * img[..., 0]).astype(np.uint8)
    return img.astype(np.uint8)


def _adaptive_threshold(gray):
    """Returns a binary image (0 = background, 255 = digit foreground).

    Uses adaptive thresholding so varying lighting doesn't kill us.
    Automatically detects which side (light or dark) is the digit by
    looking at the corners (assumed to be background).
    """
    if HAVE_CV2:
        # Adaptive mean threshold
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, blockSize=25, C=10
        )
    else:
        # Simple Otsu-ish fallback: threshold at median of midrange values
        thresh = int(np.median(gray))
        binary = (gray > thresh).astype(np.uint8) * 255

    # Determine polarity: corners should be background. If corners are
    # mostly white (255), the digit is dark -> invert. MNIST digits are
    # white on black background.
    corners = np.concatenate([
        binary[:5, :5].flatten(), binary[:5, -5:].flatten(),
        binary[-5:, :5].flatten(), binary[-5:, -5:].flatten()
    ])
    if corners.mean() > 127:
        binary = 255 - binary
    return binary


def _largest_component_bbox(binary):
    """Find the bounding box of the largest connected component of non-zero
    pixels. Returns (x, y, w, h) or None if there's nothing."""
    if HAVE_CV2:
        nlabels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
        if nlabels <= 1:
            return None
        # stats: [label, x, y, w, h, area]
        # Skip label 0 (background); find the largest by area
        idx = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        x, y, w, h, _ = stats[idx]
        return int(x), int(y), int(w), int(h)
    # Fallback: just bbox of all foreground pixels
    ys, xs = np.where(binary > 0)
    if len(xs) == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1)


def _center_on_canvas(crop, canvas_size):
    """Place `crop` (uint8) on a square `canvas_size`x`canvas_size` black
    canvas, preserving aspect ratio and centering."""
    h, w = crop.shape
    s = max(h, w)
    # We want the digit to take up most but not all of the 26x26 canvas
    # (MNIST digits are inside a 20x20 region of a 28x28 canvas; for our
    # 26x26 we aim for ~20-pixel digit).
    target_inner = int(canvas_size * 0.75)
    if HAVE_CV2:
        # Resize keeping aspect ratio so the longer side = target_inner
        scale = target_inner / s
        new_w, new_h = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
        resized = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_AREA)
    else:
        from PIL import Image
        scale = target_inner / s
        new_w, new_h = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
        resized = np.array(Image.fromarray(crop).resize((new_w, new_h), Image.LANCZOS))

    canvas = np.zeros((canvas_size, canvas_size), dtype=np.uint8)
    y0 = (canvas_size - new_h) // 2
    x0 = (canvas_size - new_w) // 2
    canvas[y0:y0+new_h, x0:x0+new_w] = resized

    # Center-of-mass shift: nudge so the digit's CoM is at the canvas center
    # (MNIST does this; helps significantly)
    if HAVE_CV2:
        m = cv2.moments(canvas)
        if m["m00"] > 0:
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
            shift_x = int(round(canvas_size / 2 - cx))
            shift_y = int(round(canvas_size / 2 - cy))
            M = np.float32([[1, 0, shift_x], [0, 1, shift_y]])
            canvas = cv2.warpAffine(canvas, M, (canvas_size, canvas_size),
                                    borderValue=0)
    return canvas


def preprocess_frame(frame, target_size=26, return_intermediate=False):
    """Full pipeline: webcam frame → 26x26 int16 in [-32, 31].

    frame: numpy uint8 array, HxW or HxWx3 (BGR or RGB; we don't care -
           grayscale is symmetric)
    target_size: 26 for MiniCNN-v2
    return_intermediate: if True, also return the intermediate uint8
                         centered canvas for debugging/display

    Returns: int16 (target_size, target_size) array
    """
    gray = _grayscale(frame)
    binary = _adaptive_threshold(gray)

    bbox = _largest_component_bbox(binary)
    if bbox is None:
        # Empty frame; return all-background
        out = np.full((target_size, target_size), -32, dtype=np.int16)
        return (out, np.zeros((target_size, target_size), np.uint8)) if return_intermediate else out

    x, y, w, h = bbox
    # Pad bbox by a small margin
    pad = max(2, min(w, h) // 4)
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(binary.shape[1], x + w + pad)
    y1 = min(binary.shape[0], y + h + pad)
    crop = binary[y0:y1, x0:x1]

    canvas = _center_on_canvas(crop, target_size)

    # Quantize [0, 255] -> [-32, 31]
    # We map: 0 -> -32 (background), 255 -> +31 (digit peak)
    # Formula: ((x / 255) * 63) - 32, with rounding & clipping
    q = np.round((canvas.astype(np.float32) / 255.0) * 63.0 - 32.0)
    q = np.clip(q, -32, 31).astype(np.int16)

    if return_intermediate:
        return q, canvas
    return q


def preprocess_pil(path, **kwargs):
    """Convenience: load a file (PNG, JPG, etc.) via PIL and preprocess.
    Avoids needing OpenCV's imread."""
    from PIL import Image
    img = np.array(Image.open(path).convert("RGB"))
    return preprocess_frame(img, **kwargs)
