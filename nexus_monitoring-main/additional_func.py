import math
from config import *
from urllib.request import urlopen, Request
import io
from PIL import Image


def ll_to_tile(lat, lng, z):
    n = 2**z
    tx = int((lng+180)/360*n)
    lr = math.radians(lat)
    ty = int((1 - math.log(math.tan(lr)+1/math.cos(lr))/math.pi)/2*n)
    return tx, ty

def lng_to_px(lng, tl_px, z):
    return (lng+180)/360*(2**z)*TILE_SIZE - tl_px

def lat_to_py(lat, tl_py, z):
    lr = math.radians(lat)
    return (1-math.log(math.tan(lr)+1/math.cos(lr))/math.pi)/2*(2**z)*TILE_SIZE - tl_py

_cache = {}

def fetch_tile(z, x, y):
    key = (z, x, y)
    if key in _cache:
        return _cache[key]
    try:
        req = Request(TILE_URL.format(z=z, x=x%2**z, y=y),
                      headers={"User-Agent":"ArduinoGPSTracker/1.0"})
        with urlopen(req, timeout=8) as r:
            img = Image.open(io.BytesIO(r.read())).convert("RGBA")
    except Exception:
        img = Image.new("RGBA",(TILE_SIZE,TILE_SIZE),(20,25,35,255))
    if len(_cache) > 150:
        _cache.pop(next(iter(_cache)))
    _cache[key] = img
    return img