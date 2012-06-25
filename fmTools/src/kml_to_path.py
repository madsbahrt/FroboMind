#!/usr/bin/env python

from lxml import etree # xml processing
import argparse

from math import *

# LATITUDE/LONGITUDE TO UTM ##################################################
def ll2utm (lat, lon):

   flat = 1/298.257223563 # WGS84 flat
   a = 6378137 # WGS84 equatorial radius
   k0 = 0.9996
   latr = lat * pi/180.
   lonr = lon * pi/180.
   e = 0
   n = 0
   znum = -1
   zlet = '0'

   # test if the UTM projection is defined for this latitude and longitude
   if lat <= 84.0 and lat >= -80.0:
       # determine the UTM zone number
       znum = int((lon + 180)/6) + 1

       if lat >= 56. and lat < 64. and lon >= 3. and lon < 12.:
           znum = 32

       # take care of zone numbers for Svalbard
       if lat >= 72. and lat < 84.:
           if lon >=  0. and lon <  9.: znum = 31
           elif lon >=  9. and lon < 21.: znum = 33
           elif lon >= 21. and lon < 33.: znum = 35
           elif lon >= 33. and lon < 42.: znum = 37

       # determine the UTM zone letter
       if  84.0 >= lat and lat >= 72.0: zlet = 'X'
       elif 72.0 > lat and lat >= 64.0: zlet = 'W'
       elif 64.0 > lat and lat >= 56.0: zlet = 'V'
       elif 56.0 > lat and lat >= 48.0: zlet = 'U'
       elif 48.0 > lat and lat >= 40.0: zlet = 'T'
       elif 40.0 > lat and lat >= 32.0: zlet = 'S'
       elif 32.0 > lat and lat >= 24.0: let = 'R'
       elif 24.0 > lat and lat >= 16.0: zlet = 'Q'
       elif 16.0 > lat and lat >= 8.0: zlet = 'P'
       elif 8.0 > lat and lat >= 0.0: zlet = 'N'
       elif 0.0 > lat and lat >= -8.0: zlet = 'M'
       elif -8.0 > lat and lat >= -16.0: zlet = 'L'
       elif -16.0 > lat and lat >= -24.0: zlet = 'K'
       elif -24.0 > lat and lat >= -32.0: zlet = 'J'
       elif -32.0 > lat and lat >= -40.0: zlet = 'H'
       elif -40.0 > lat and lat >= -48.0: zlet = 'G'
       elif -48.0 > lat and lat >= -56.0: zlet = 'F'
       elif -56.0 > lat and lat >= -64.0: zlet = 'E'
       elif -64.0 > lat and lat >= -72.0: zlet = 'D'
       elif -72.0 > lat and lat >= -80.0: zlet = 'C'

       # calculate UTM northing and easting
       es = 2*flat-flat*flat
       eps = (es)/(1-es)

       # find the center longitude for the UTM zone
       lonr_center = ((znum -1)*6-180+3) * pi/180.

       N = a/sqrt(1-es*sin(latr)*sin(latr))
       T = tan(latr)*tan(latr)
       C = eps*cos(latr)*cos(latr)
       A = cos(latr)*(lonr-lonr_center)
       M = a*((1 - es/4 - 3*es*es/64 - 5*es*es*es/ 256)*latr - (3* es/8 + 3*es*es/32 + 45*es*es*es/1024)*sin(2*latr) + (15*es*es/256 + 45*es*es*es/1024)*sin(4*latr)-(35*es*es*es/3072)*sin(6*latr))
       e = (k0*N*(A+(1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C-58*eps)*A*A*A*A*A/120) + 500000.0)
       n = (k0*(M+N*tan(latr)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24 + (61-58*T+T*T+600*C-330*eps)*A*A*A*A*A*A/720)));

       if lat < 0:
           n += 10000000.0 # 10000000 meter offset for southern hemisphere

   return (znum, zlet, n, e)





if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='kml_to_path',description=""" extracts utm coordinates from kml file and converts to simple csv of xy points """)
    parser.add_argument('-i', '--input',help="input kml file")
    parser.add_argument('-o', '--output',help="output csv file")
    
    args= parser.parse_args()
    
    NS = 'http://earth.google.com/kml/2.2'
    
    ns = {'kml':NS}

    try:
        doc = etree.parse(args.input)
    except:
        print "could not open file %s" % args.input
        exit(1)
        
    d =  doc.xpath('//kml:Document/kml:Placemark',namespaces=ns)
    
    for placemark in d:
        print "Processing Placemark: ",placemark.findtext("kml:name",namespaces=ns)
        for coordinate in placemark.findall("kml:LineString/kml:coordinates",namespaces=ns):
           with open(args.output,"w") as output:
               output.write("x, y\n")
               for line in coordinate.text.split("\n"):
                   if line and len(line.split(",")) > 1:
                       print line
                       lat,lon,alt = line.split(",")
                       lat,lon,alt = [float(i) for i in [lat,lon,alt]]
                       num,let,north,east = ll2utm(lat, lon)
                       output.write("{0}, {1}\n".format(east,north))
                   