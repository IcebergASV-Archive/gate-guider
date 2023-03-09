
def meters_per_degree():
    #meters per degree latitude
    meters_per_lat = 111132.92 - 559.82*cos(2*lat) + 1.175*cos(4*lat) - 0.0023*cos(6*lat)
    #meters per degree longitude
    meters_per_lon = 111412.84*cos(lat) - 93.5*cos(3*lat) + 0.118*cos(5*lat)

    return


