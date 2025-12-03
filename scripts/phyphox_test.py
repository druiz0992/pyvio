import requests
import time
from pyproj import CRS, Transformer
import math

IP = "192.168.1.34"          # phyphox device IP
ORIGIN_UTM = None            # set after first valid GPS point

BUFFERS = [
    "gps_time",
    "gpsLat",
    "gpsLon",
    "gpsZ",
    "gpsV",
    "gpsDir",
    "gpsAccuracy",
    "gpsZAccuracy",
    "gpsSatellites",
    "gpsStatus"
]

# -------------------------------
# 1. Convert lat/lon to UTM
# -------------------------------
def latlon_to_utm(lat, lon):
    """Returns (x, y, zone_number). Assumes WGS84."""
    zone = int((lon + 180) / 6) + 1
    crs_utm = CRS.from_epsg(32600 + zone)  # northern hemisphere
    transformer = Transformer.from_crs("EPSG:4326", crs_utm, always_xy=True)
    x, y = transformer.transform(lon, lat)
    return x, y, zone

# -------------------------------
# 2. Convert UTM to local frame
# -------------------------------
def to_local_frame(utm_x, utm_y, utm_z, heading_deg):
    global ORIGIN_UTM

    # If heading is None, default to 0°
    if heading_deg is None:
        heading_deg = 0.0

    heading = math.radians(heading_deg)

    if ORIGIN_UTM is None:
        ORIGIN_UTM = (utm_x, utm_y, utm_z)
        print(">>> ORIGIN SET:", ORIGIN_UTM)

    dx = utm_x - ORIGIN_UTM[0]
    dy = utm_y - ORIGIN_UTM[1]
    dz = utm_z - ORIGIN_UTM[2]

    x_local = dx*math.cos(heading) - dy*math.sin(heading)
    y_local = dx*math.sin(heading) + dy*math.cos(heading)

    return x_local, y_local, dz

# -------------------------------
# Retrieve Phyphox remote buffers
# -------------------------------
def get_full_data():
    query = "&".join(f"{b}=full" for b in BUFFERS)
    url = f"http://{IP}/get?{query}"
    try:
        r = requests.get(url, timeout=2)
        r.raise_for_status()
        return r.json().get("buffer", {})
    except Exception as e:
        print("HTTP error:", e)
        return {}

def print_data(buf):
    def get(b):
        return buf.get(b, {}).get("buffer", [])

    times = get("gps_time")
    lats  = get("gpsLat")
    lons  = get("gpsLon")
    zs    = get("gpsZ")
    vs    = get("gpsV")
    dirs  = get("gpsDir")
    sats  = get("gpsSatellites")
    acc_h = get("gpsAccuracy")
    acc_z = get("gpsZAccuracy")
    status = get("gpsStatus")

    n = max(len(times), len(lats), len(lons), len(zs), len(vs), len(dirs),
            len(sats), len(acc_h), len(acc_z), len(status))

    for i in range(n):
        if i >= len(lats): 
            continue
        lat = lats[i]
        lon = lons[i]
        alt = zs[i] if i < len(zs) else None
        heading = dirs[i] if i < len(dirs) else 0.0

        if lat is None or lon is None:
            continue

        # ---- UTM conversion ----
        utm_x, utm_y, zone = latlon_to_utm(lat, lon)
        utm_z = alt if alt is not None else 0.0

        # ---- Local vehicle frame ----
        x_local, y_local, z_local = to_local_frame(utm_x, utm_y, utm_z, heading)

        print(
            f"[{i}] lat={lat:.7f}, lon={lon:.7f}, alt={alt} "
            f"| UTM=({utm_x:.2f}, {utm_y:.2f}, zone {zone}) "
            f"| LOCAL=({x_local:.2f}, {y_local:.2f}, {z_local:.2f}) "
            f"| heading={heading}°"
        )


def main():
    print("Polling full buffer from phyphox ...")
    while True:
        buf = get_full_data()
        if buf:
            print_data(buf)
        else:
            print("No buffer data returned.")
        print("-" * 80)
        time.sleep(1)


if __name__ == "__main__":
    main()

