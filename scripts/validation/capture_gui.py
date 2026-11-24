#!/usr/bin/env python3
"""Capture only this simulator's three GUI windows, including WSLg buffers."""
import ctypes as C
import json
from pathlib import Path
import re
import subprocess

from PIL import Image


class XImage(C.Structure):
    _fields_ = [('width', C.c_int), ('height', C.c_int), ('xoffset', C.c_int),
        ('format', C.c_int), ('data', C.c_void_p), ('byte_order', C.c_int),
        ('bitmap_unit', C.c_int), ('bitmap_bit_order', C.c_int), ('bitmap_pad', C.c_int),
        ('depth', C.c_int), ('bytes_per_line', C.c_int), ('bits_per_pixel', C.c_int),
        ('red_mask', C.c_ulong), ('green_mask', C.c_ulong), ('blue_mask', C.c_ulong)]


def main():
    ws = Path(__file__).resolve().parents[2]
    session = json.loads((ws / 'reports/gazebo_latest.json').read_text())
    folder = Path(session['report_dir'])
    x = C.CDLL('libX11.so.6')
    composite = C.CDLL('libXcomposite.so.1')
    x.XOpenDisplay.argtypes, x.XOpenDisplay.restype = [C.c_char_p], C.c_void_p
    x.XCloseDisplay.argtypes = [C.c_void_p]
    composite.XCompositeNameWindowPixmap.argtypes = [C.c_void_p, C.c_ulong]
    composite.XCompositeNameWindowPixmap.restype = C.c_ulong
    x.XGetImage.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint, C.c_ulong, C.c_int]
    x.XGetImage.restype = C.POINTER(XImage)
    x.XDestroyImage.argtypes = [C.POINTER(XImage)]
    x.XFreePixmap.argtypes = [C.c_void_p, C.c_ulong]
    display = x.XOpenDisplay(None)
    if not display:
        raise RuntimeError('host display unavailable')
    try:
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, timeout=5.)
        (folder / 'windows.txt').write_text(tree)
        for name, pattern in [('gazebo', '"Gazebo": ("gazebo" "gazebo")'),
                              ('rviz', '("rviz2" "rviz2")'),
                              ('control', '"Rangermini + xArm6 | C++ Gazebo control":')]:
            row = next(row for row in tree.splitlines() if pattern in row)
            window = int(re.search(r'0x[0-9a-fA-F]+', row).group(), 16)
            width, height = map(int, re.search(r'\s(\d+)x(\d+)\+', row).groups())
            pixmap = composite.XCompositeNameWindowPixmap(display, window)
            ptr = x.XGetImage(display, pixmap, 0, 0, width, height, C.c_ulong(-1).value, 2)
            if not ptr:
                raise RuntimeError('window capture failed: ' + name)
            try:
                frame = ptr.contents
                # Named pixmaps may report zero color masks in WSLg; its
                # little-endian, 24-bit XRGB buffers still use BGRX bytes.
                if frame.bits_per_pixel != 32 or frame.byte_order != 0:
                    raise RuntimeError('unsupported display pixel format')
                raw = C.string_at(frame.data, frame.bytes_per_line * frame.height)
                image = Image.frombytes('RGB', (frame.width, frame.height), raw, 'raw', 'BGRX', frame.bytes_per_line, 1)
                if max(high for low, high in image.getextrema()) == 0:
                    raise RuntimeError('all-black screenshot: ' + name)
                image.save(folder / (name + '.png'))
                print(folder / (name + '.png'))
            finally:
                x.XDestroyImage(ptr)
                x.XFreePixmap(display, pixmap)
    finally:
        x.XCloseDisplay(display)


if __name__ == '__main__':
    main()
