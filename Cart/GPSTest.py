from gps3 import agps3threaded as gps

GPS = gps.AGPS3mechanism()
GPS.stream_data()
GPS.run_thread()

while True:
    try:
        print(float(GPS.data_stream.lat))
    except:
        pass
