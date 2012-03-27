#!/usr/bin/env python

import argparse

bitrate_to_pcan_hex = {
    1000000:"0x0014",
     500000:"0x001C",
     250000:"0x011C",
     125000:"0x031C",
     100000:"0x432F",
      50000:"0x472F",
      20000:"0x532F",
      10000:"0x672F",
       5000:"0x7F7F" 
}

def print_available_baudrates():
    for k in reversed(sorted(bitrate_to_pcan_hex)):
        print k

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Configures the CAN bitrate of an PCAN USB dongle via /dev/pcanXX entry')
    parser.add_argument('device', metavar='-d', type=str,
                   help='path to PCAN device entry to operate on')
    parser.add_argument('bitrate', metavar='-b', type=str,
                   help='bitrate in bits/s K can be used to indicate Kilo e.g. 250K is also acceptable')

    args = parser.parse_args()

    bitrate_str = args.bitrate.upper()

    if "K" in bitrate_str:
        bitrate = int(bitrate_str.strip("K")) * 1000
    else:
        bitrate = int(bitrate_str)

    if not bitrate in bitrate_to_pcan_hex:
        print "Could not find entry for bitrate %d" % bitrate
        print "The following bitrates are available"
        print_available_baudrates()
        
    else:
        print "Configuring driver with the following string"
        print "i %s e\n" % bitrate_to_pcan_hex[bitrate]
        with open(args.device,"w") as dev:
            dev.write("i %s e\n" % bitrate_to_pcan_hex[bitrate])
        print "Done"

