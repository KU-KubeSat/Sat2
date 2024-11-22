from time import sleep
import serial
import os
import hashlib
import datetime
#  ------  payload scripts commented out for testing purposes right now ----------
#import hical.py
#import pcrd.py

# -----    globals   --------
readFolder = "./NewInfo/"
writeFolder = "./OldInfo/"
port = None #serial.Serial("/dev/ttyAMA0")    #pi 4b

# --------    initialization  -------------
#port.open()

# -------     function definitions     -------------
def sendFiles():
    global readFolder
    global writeFolder
    global port
    for root, _, files in os.walk(readFolder):
        for file in files:
            path = os.path.join(root, file)
            contents = open(path, "rb").read()
            #clean this up? check if we need to send some kind of start and stop to defined when one file stops and another starts
            #maybe do: 
            # filename
            # size of file
            # timestamp or something?
            # file contents
            # hash
            port.write(file) #file name
            port.write(os.path.getsize(path)) #file size
            port.write(datetime.timestamp(datetime.now())) #UNIX Timestamp
            port.write(contents) #file contents
            h = hashlib.sha256(contents)
            port.write(bytes(h.hexdigest())) #hash of file contents
            open(os.path.join(writeFolder, file), "wb").write(contents)
            os.remove(path)

def checkMessages():
    global port
    if port.in_waiting > 0:
        print("input waiting")
    else:
        print("run pcrd")
        #run the pcrd if nothing sent
    return



# main loop
while True:
    #send any files and then move it to another folder
    sendFiles()
    
    #check for message
    checkMessages()

    #sleep for some time? Could be changed later
    sleep(5)
    break



#messages
# reset stats (per script) (Note: ask Daniel about what stats we need to keep)
# run hical (x2)
# run camera
# idle
# resend last file(s)? (I send a hash of the file as a checksum rn, this may or may not be necessary if we dont care to check that stuff)


#write up test cases