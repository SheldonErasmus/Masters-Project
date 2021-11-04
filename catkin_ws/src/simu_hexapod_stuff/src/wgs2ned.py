from math import pi, sqrt, sin, cos

#Convert WGS84 to ECEF
def WGS2XYZ(lat,lon,alt):
    # Convert lat, long, height in WGS84 to ECEF X,Y,Z
    # lat and long given in decimal degrees.
    # altitude should be given in meters

    lat = lat/180.0*pi #converting to radians
    lon = lon/180.0*pi #converting to radians
    a = 6378137.0 #earth semi-major axis in meters
    f = 1.0/298.257223563 #reciprocal flattening
    e2 = 2.0*f -f**2.0 #eccentricity squared
     
    chi = sqrt(1-e2*(sin(lat))**2)
    X = (a/chi + alt)*cos(lat)*cos(lon)
    Y = (a/chi + alt)*cos(lat)*sin(lon)
    Z = (a*(1-e2)/chi + alt)*sin(lat)

    return X, Y, Z

def WGS2NED(refLat,refLon,refAlt,lat,lon,alt):
    #First convert WGS84 to ECEF
    (Xr, Yr, Zr) = WGS2XYZ(refLat,refLon,refAlt) #Reference location in ECEF
    (X, Y, Z) = WGS2XYZ(lat,lon,alt) #Target location in ECEF

    refLat = refLat/180.0*pi #converting to radians
    refLon = refLon/180.0*pi #converting to radians

    #Convert ECEF to ENU
    e = -sin(refLon)*(X-Xr) + cos(refLon)*(Y-Yr)
    n = -sin(refLat)*cos(refLon)*(X-Xr) - sin(refLat)*sin(refLon)*(Y-Yr) + cos(refLat)*(Z-Zr)
    u = cos(refLat)*cos(refLon)*(X-Xr) + cos(refLat)*sin(refLon)*(Y-Yr) + sin(refLat)*(Z-Zr)

    #Return in NED
    d = -u
    return n,e,d

"""# **EXAMPLE**
#The local coordinate origin (Zermatt, Switzerland)
lat0 = 46.017 # deg
lon0 = 7.750  # deg
h0 = 1673     # meters

#The point of interest
lat = 45.976  # deg
lon = 7.658   # deg
h = 4531      # meters

(N,E,D) = WGS2NED(lat0, lon0, h0, lat, lon, h)
print(N,E,D)"""