# -*- coding: UTF-8 -*-

"""
Capture data locally from QTM and by UDP using OSC protocol from a IMU unit.
The main function opens the local port to listen the QTM and monitores its events until the capture begins.
Once it receives a QTM package, the osc listener is called. This process repeats until the QTM capture ends.
If both devices have the same acquisition sample rate, the acquired data will be syncronized.
At the final, a mat file is saved with the captured frames, and the markers and imu data received at each frame

--------------------
Command line inputs:
name_capture: the name given to the capture (this will be the filename of the stored mat data)
ip_server: the IP to which the IMU data has being sent
port_server: the port to listen the IMU data
pattern_data: the OSC pattern of the data of interest
--------------------

Data from the IMU and QTM are stored in different lists and after the QTM capture has stopped, these data are converted to a structure and saved as a mat file.
---------------------
Structure:
name: name_capture
fields: each acquired frame
    frame fields: each marker id and `imu`
---------------------
"""


from osc4py3.as_eventloop import *
from osc4py3 import oscmethod as osm
import asyncio, qtm, sys
import numpy as np
from scipy.io import *

   
async def main(ip_server, port_server, pattern_data):
    """ Main function """
    connection = await qtm.connect("127.0.0.1")
    if connection is None:
        return

    state = await connection.get_state()
    print(state)
    if state != qtm.QRTEvent.EventConnected:
            try:
                await connection.await_event(qtm.QRTEvent.EventConnected, timeout=60)
            except asyncio.TimeoutError:
                print("Failed to start new measurement")
                return -1
    if state != qtm.QRTEvent.EventCaptureStarted:
            try:
                await connection.await_event(qtm.QRTEvent.EventCaptureStarted, timeout=60)
            except asyncio.TimeoutError:
                print("Failed to start new capture")
                return -1

    t0 = time.time()*1000
    
    def on_packet(packet):
        """ Callback function that is called everytime a data packet arrives from QTM """
        print("Framenumber: {}".format(packet.framenumber))
        #header, markers = packet.get_3d_markers_no_label()
        meta.append([t0, time.time()*1000, packet.framenumber])    
        #print("Component info: {}".format(header))
        #markers_list.append(markers)
    
    def store_data(*args):
        args = list(args)
        print(args)
        imu_data.append([t0, time.time()*1000]+args[1:])
        

    disp = dispatcher.Dispatcher()
    disp.map(pattern_data, store_data)
    server = osc_server.ThreadingOSCUDPServer((ip_server, ip_server), disp) 
    
    await connection.stream_frames(components=["timecode"], on_packet=on_packet)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.start() 
    
    state = await connection.get_state()
    print(state)
    while state != qtm.QRTEvent.EventCaptureStopped:
        await asyncio.sleep(1)
        state = await connection.get_state()
        print(state)
    await connection.stream_frames_stop()
    server.shutdown() 


if __name__ == "__main__":
    name_capture = sys.argv[1]
    ip_server = sys.argv[2]
    port_server = int(sys.argv[3])
    pattern_data = sys.argv[4]
    
    meta, markers_list, imu_data = [], [], []
    asyncio.get_event_loop().run_until_complete(main(ip_server, port_server, pattern_data))


    # construct the structure and save the mat file
    data = {}
    data[name_capture] = {}
    ids_frames = np.array([meta[i][-1] for i in range(len(meta))])
    n = np.where(ids_frames==ids_frames[-1])[0][0]+1
    for i in range(n):
        data[name_capture]['frame'+str(ids_frames[i])] = {'qtm': meta[i]}
        if(len(markers_list) > i and len(markers_list[i]) > 0):
            for k in range(len(markers_list[i])):
                data[name_capture]['frame'+str(ids_frames[i])]['marker'+str(markers_list[i][k].id)] = [markers_list[i][k].x, markers_list[i][k].y, markers_list[i][k].z]
        if(len(imu_data) > i and len(imu_data[i]) > 0): 
            data[name_capture]['frame'+str(ids_frames[i])]['imu'] = imu_data[i]
    
    savemat(name_capture+'.mat', data)
    with open(name_capture+'.json', 'w') as f:
        json.dump(data, f, indent=1)
