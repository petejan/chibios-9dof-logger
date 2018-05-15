import sys
import struct

typeCount = [0, 0, 0, 0, 0, 0, 0, 0]

for fn in sys.argv[1:]:
    #print fn
    with open(fn, "rb") as f:
        errorCount = 0
        while True:
            byte = f.read(3)
            if not byte:
                break
            (type, N) = struct.unpack("<BH", byte)
            #print 'type', type, ' N ', N
            pos = f.tell()
            if (type < 8) & (N < 400):
                pkt = f.read(N)
                check = f.read(1)
                if not check:
                    break
                checkB = ord(check)
                s = 0
                for b in pkt:
                    s = s + ord(b)
                s = s & 0xff
                if (checkB == s):
                    typeCount[type] = typeCount[type] + 1
                    try:
                        if type == 4:
                            # print("pkt " , ord(pkt[0]), ord(pkt[1]), ord(pkt[2]), ord(pkt[3]), ord(pkt[4]), ord(pkt[5]), ord(pkt[6]), ord(pkt[7]))
                            time = struct.unpack("<BBBBBBH", pkt)
                            print ("time %4d-%02d-%02d %02d:%02d:%02d" % (time[6], time[5], time[4], time[2], time[1], time[0]))
                        if type == 0:
                            nmea = pkt.decode('utf-8')
                            print pos, " nmea ", nmea[:-2]
                        if type == 1:
                            ubx = struct.unpack("<BBBBH", pkt[0:6]);
                            #print "ubx " , ubx
                        if type == 7:
                            text = pkt.decode('utf-8')
                            text = text[:-1]
                            print "text : ", text
                        if type == 5:
                            # print("pkt " , ord(pkt[0]), ord(pkt[1]), ord(pkt[2]), ord(pkt[3]))
                            adc = struct.unpack("<i", pkt)
                            #print "adc ", adc[0]
                        if type == 6:
                            # print("pkt " , ord(pkt[0]), ord(pkt[1]), ord(pkt[2]), ord(pkt[3]))
                            count = struct.unpack("<HHHHHHHH", pkt)
                            #print "count ", count
                            #print "found ", typeCount
                        if type == 3:
                            imu = struct.unpack("<hhhhhhhhhhlllll", pkt)
                            #print "imu", imu
                            #print imu[0], imu[1], imu[2], imu[3], imu[4], imu[5], imu[6], imu[7], imu[8]
                    except (UnicodeDecodeError, struct.error):
                        print pos, " Error : ", pkt;
                        errorCount = errorCount + 1
                else:
                    print pos, " sum error ", checkB, s
                    f.seek(pos)

print "error count", errorCount                    
