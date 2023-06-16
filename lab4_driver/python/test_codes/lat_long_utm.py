import utm



lat = 42.202456
longit = -71.053999
(easting, northing, zone_number, zone_letter) = utm.from_latlon(lat, longit)

print("(easting, northing, zone_number, zone_letter)")
print((easting, northing, zone_number, zone_letter))