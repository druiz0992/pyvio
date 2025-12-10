import numpy as np
from pyproj import Transformer

import numpy as np
from pyproj import Transformer


class LLA2ENU:
    def __init__(self, lat0, lon0, h0):
        """
        lat0, lon0, h0 come from GPS.
        BUT h0 is ignored for defining the ENU frame (set to 0 internally).
        """
        # Store user values for compatibility
        self.lat0_deg = lat0
        self.lon0_deg = lon0
        self.h0_user = h0

        # Internal reference (h0 ignored)
        self.lat0 = np.radians(lat0)
        self.lon0 = np.radians(lon0)

        # Transformer LLA <-> ECEF
        self.lla_to_ecef = Transformer.from_crs(
            "EPSG:4979",  # WGS84 lat/lon/ellipsoidal height
            "EPSG:4978",  # ECEF XYZ
            always_xy=True
        )

        # ENU origin in ECEF (altitude forced = 0)
        # This is the key modification
        self.x0, self.y0, self.z0 = self.lla_to_ecef.transform(lon0, lat0, 0.0)

        # Precompute ENU rotation matrix
        clat = np.cos(self.lat0)
        slat = np.sin(self.lat0)
        clon = np.cos(self.lon0)
        slon = np.sin(self.lon0)

        self.R = np.array([
            [-slon,          clon,         0],
            [-clon*slat, -slon*slat,  clat],
            [ clon*clat,  slon*clat,  slat]
        ])

    @classmethod
    def from_first_sample(cls, lat, lon, h):
        """API-compatible initializer."""
        return cls(lat, lon, h)

    def convert(self, lat, lon, h):
        """
        Convert GPS (lat, lon, h) → ENU.
        h IS used normally here to generate the correct U component.
        """
        # ECEF of incoming measurement
        x, y, z = self.lla_to_ecef.transform(lon, lat, h)

        dx = np.array([x - self.x0, y - self.y0, z - self.z0])
        return self.R @ dx
