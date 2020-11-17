import libscrc
import os

FileName = str(input("Enter file name:"))
try:
    file = open(FileName,"rb")
    if "CRC".encode() in file.read():
        print("Checksum is already exist!")
        NumOfBytes = os.path.getsize(FileName) #get file size
        file.close()
        file = open(FileName,"rb")
        DataBytes = file.read()
        print("CRC: 0x",end="")
        for i in range(4,0,-1):
            print(hex(DataBytes[NumOfBytes-5+i]).lstrip('0x'),end="")
        print()
        print("Number of bytes(with CRC + 4 bytes): " + str(NumOfBytes))
        file.close()
    else:
        file = open(FileName, "rb")
        BinData = file.read()
        NumOfBytes = os.path.getsize(FileName)
        CRC32 = libscrc.fsc(BinData) #Ethernet Frame Sequence
        print("CRC: " + hex(CRC32))
        print("Number of bytes: " + str(NumOfBytes))
        NewFile = open(FileName,"ab")
        NewFile.write("CRC".encode())
        NewFile.write(CRC32.to_bytes(4,byteorder= 'little')) # little endian
        print("CRC added successfully.")
        NewFile.close()
        NewFile = open(FileName,"rb")
        NewBinData = NewFile.read()
        print("End of the file -> ....",end="")
        for i in range(0,7):
            if i < 3:
                print(chr(NewBinData[NumOfBytes+i]),end=" ")
            else:
                print(hex(NewBinData[NumOfBytes + i]), end=" ")

except OSError as error:
    print("OS error: {0}".format(error))
except:
    print("Unexpected error!")
