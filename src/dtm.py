import numpy as np
import rasterio

class DTM:
    def __init__(self, path):
        self.src = rasterio.open(path)
        self.array = self.src.read(1)
        self.transform = self.src.transform
        self.nodata = self.src.nodata

    def elevation_at(self, x, y):
        """Return bilinearly-interpolated elevation at geographic coords (x,y).
        x,y should be in the same CRS as the raster (usually lon/lat or projected).
        """
        # Convert world coords to raster row/col
        col, row = ~self.transform * (x, y)
        # indices
        i = int(np.floor(row))
        j = int(np.floor(col))
        # bounds check
        if i < 0 or j < 0 or i + 1 >= self.array.shape[0] or j + 1 >= self.array.shape[1]:
            return float('nan')
        # fractional
        dy = row - i
        dx = col - j
        z00 = self.array[i, j]
        z10 = self.array[i, j+1]
        z01 = self.array[i+1, j]
        z11 = self.array[i+1, j+1]
        # handle nodata
        vals = np.array([z00, z10, z01, z11], dtype=float)
        if np.all(vals == self.nodata):
            return float('nan')
        # bilinear interpolation
        z0 = z00*(1-dx) + z10*dx
        z1 = z01*(1-dx) + z11*dx
        z = z0*(1-dy) + z1*dy
        return float(z)
