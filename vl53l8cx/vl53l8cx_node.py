#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time

import numpy as np
import os

import serial
import math       # import math for math.pi constant

#ROS2 libraries include
import rclpy                    #use ROS2 python library 
from rclpy.node import Node     #import ROS2 Node class

#Import ROS2 PCL message format
from sensor_msgs.msg import PointCloud2, PointField 

print("")
print("")
print("Start of VL53L8CX Node")
print("----------------------")
print("")

# #Try to connect to the STM NUCLEO Board using an assigned Serial Port
# try:
#     print("STEP 2: Searching for STM Nucleo Serial/USB Port.")
#     #Configure serial port. Make it global
#     #serial_port = serial.Serial(port="/dev/ttyACM0",baudrate=460800,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5)    
#     serial_port = serial.Serial(port="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF484971754867124037-if02",baudrate=460800,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5)
#     #/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF484971754867124037-if02
#     #/dev/serial/by-path/platform-3610000.xhci-usb-0:2.2:1.2

#     print('Serial Port/USB Connection acquired')
#     print("")
#     # Wait a 0.5 second to let the port initialize
#     time.sleep(0.5)
#     hardware_connected = True
# except:
#     hardware_connected = False
#     print ("ERROR: Serial Port Communication not present.")
#     exit()  #Exit if not USB COM Present

VL53L5_Zone_Pitch8x8 = [
		59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00,
		64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
		67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
		70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
		70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
		67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
		64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
		59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00    ]

VL53L5_Zone_Yaw8x8 = [
		135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
		144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
		156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
		171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
		188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
		203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
		203.20,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
		225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00  ]

class RingBuffer:
    """ class that implements a not-yet-full buffer (Circular Buffer)
        This buffer is used with the serial port interface class
    """

    def __init__(self, size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """

        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max

        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self, x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data

#Create class
class VL53L8CX (Node):  #Class inherits from ROS node object

    print('In VL53L8CX Hardware Driver Class')
    print(' ')

    #define class constructor
    def __init__(self, sensor_number):    
    #def __init__(self):
        """
        Do any necessary configuration and connect to the STM Nucleo EVM.
        """
        print('Initializing VL53L8CX Device.\n\r')      

        #Include Class object attributes in this section

        ####
        ####  Class attributes 
        ####

        #super function is used to give access to methods and properties of a parent class
        #functions in python are outside of a class. Methods are inside of a class
        #call the super function. The node name is initialize here, "vl53l8cx_Node".
        super().__init__("vl53l8cx_Node")  
        self.counter_ = 0
        self.get_logger().info("1. In ROS2 Node.")

        self.node_args = sensor_number.copy()

        self.get_logger().info(str(self.node_args))  

        #logger.info('Passed args = ', str(self.node_args))    
        #self.get_logger().info((type(sensor_number))

        try:
            print("STEP 2: Searching for STM Nucleo Serial/USB Port.")
            self.get_logger().info("STEP 2: Searching for STM Nucleo Serial/USB Port.") 
            #Configure serial port. 
            #serial_port = serial.Serial(port="/dev/ttyACM0",baudrate=460800,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5)    
            if sensor_number[1] == '1':
                self.serial_port = serial.Serial(port="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF555185754867104419-if02",
                                                            baudrate=460800,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5)
                #/dev/serial/by-path/platform-3610000.xhci-usb-0:2.2:1.2
                self.get_logger().info('Serial Port/USB 1 Connection acquired')            
                print('Serial Port/USB 1 Connection acquired')
                print("")
                # Wait a 0.5 second to let the port initialize
                time.sleep(0.5)
                self.hardware_connected = True
            if sensor_number[1] == '2':
                self.serial_port = serial.Serial(port="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF484971754867124037-if02",
                                                            baudrate=460800,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5)
                #/dev/serial/by-path/platform-3610000.xhci-usb-0:2.2:1.2
                self.get_logger().info('Serial Port/USB 2 Connection acquired')
                print('Serial Port/USB 2 Connection acquired')
                print("")
                # Wait a 0.5 second to let the port initialize
                time.sleep(0.5)               
                self.hardware_connected = True                                 
        except:
            self.hardware_connected = False
            print ("ERROR: Serial Port Communication not present.")
            sys.exit(1)   #Exit if not USB COM Present

        #Create a ring buffer of size (1000). VL53L8CX max buffer data size = 96 bytes per row x 8 rows = 768 bytes  
        self.UART_ReceiveBuffer = RingBuffer(1000)   
        #This ring buffer is global for the instance of this class.
        #Globals in the namespace are accessible to all objects. 
        self.get_logger().info('Created VL53L8CX Received Buffer Object.\n\r')
        #self.get_logger().info("Data Name = ", sensor_number[2])
        print('Created VL53L8CX Received Buffer Object.\n\r')    

        self.rx_counter = 0
        self.HeaderFound = False
        self.tof_data = []

        #Define the various tables that are used to calculate the x, y, x coordinate values    
        self.SinOfPitch = []
        self.CosOfPitch = []
        self.SinOfYaw = []
        self.CosOfYaw = []

        #Compute sin/cos tables at startup. This tables will be use on the program to calculate
        #x, y and z coordinates
        self.computeSinCosTables()
        
        try:
            #Create a publisher to publish the Point Cloud data.
            self.pcl_data_publisher_ = self.create_publisher(PointCloud2, sensor_number[2], 10)       
            #self.pcl_data_publisher_ = self.create_publisher(PointCloud2, 'pcl_data', 10)    

            self.timer_ = self.create_timer(0.02, self.publish_pcl_data)  #0.1 second interval
            self.get_logger().info("PCL Data Publisher has started.")
            #print('Instantiated VL53L8CX class.\n\r')
        
        except:
            self.get_logger().info("Error.")
            print ("ERROR: Serial Port Communication not present.")
            #sys.exit(1)   #Exit if not USB COM Present

    def pcl_to_ros(self, pcl_array, frame_id="", is_intensity=False):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                pcl_array (PointCloud_PointXYZI): A PCL XYZI point cloud

            Returns:
                PointCloud2: A ROS point cloud
        """
        ros_msg = PointCloud2()

        # Timestamp should be passed in at the callback time and used consistently throughout the entire chain
        t = self.get_clock().now()
        ros_msg.header.stamp = t.to_msg()
        ros_msg.header.frame_id = frame_id

        ros_msg.height = 1   
        ros_msg.width = 64   #Point Cloud has 64 points in total

        ros_msg.fields.append(PointField(
                                name="x",
                                offset=0,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="y",
                                offset=4,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="z",
                                offset=8,
                                datatype=PointField.FLOAT32, count=1))
        if is_intensity:
            ros_msg.fields.append(PointField(
                                name="intensity",
                                offset=12,
                                datatype=PointField.FLOAT32, count=1))
        
            ros_msg.is_bigendian = False
            ros_msg.point_step = 16     #4 bytes per coordinate, 4 bytes for intensity 4+4+4+4 = 16 
            ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height               
        else:
            ros_msg.is_bigendian = False
            ros_msg.point_step = 12     #4 bytes per coordinate, 4+4+4 = 12
            ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height           
            
        ros_msg.is_dense = False

        #Convert x,y,z python list into a numpy array. The Data has to be arrange in a 64, 3 shape
        #arry = ([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4], [xn, yn, zn]])
        newArray = np.reshape(pcl_array, (64, 3))
        print("Array Shape: ", newArray.shape)
        print("Numpy Array: ", newArray)

        dtype = np.float32        
        if is_intensity:
            ros_msg.data = newArray.astype(dtype).tobytes()             
        else:
            ros_msg.data = newArray.astype(dtype).tobytes()             

        print("PCL ROS Data: ", ros_msg.data)

        return ros_msg

    def publish_pcl_data(self):
        self.scaledData = [None] * 192   #Define a empty list of 192 elements

        try:
            #Retrive VL53L8CX data from STM's board
            self.VL53L8CX_Sensor_Data = self.usb_transfer_array()
            #Calculate Point Cloud x, y, z coordinates
            self.VL53L8CX_xyz_Sensor_Data = self.calc_xyz_coordinates(self.VL53L8CX_Sensor_Data)  
            print("\n\rxyz Data: \n\r\n\r", self.VL53L8CX_xyz_Sensor_Data)

            #Scale the distance reading to m from mm
            for x in range(192):
                self.scaledData[x] = self.VL53L8CX_xyz_Sensor_Data[x]/1000
            print("\n\rScaled xyz Data: \n\r\n\r", self.scaledData)       

            self.ros_msg = PointCloud2()
            #self.ros_msg = self.pcl_to_ros(self.VL53L8CX_xyz_Sensor_Data, "map", False)

            #self.ros_msg = self.pcl_to_ros(self.scaledData, "world", False)
            self.ros_msg = self.pcl_to_ros(self.scaledData, self.node_args[3], False)
            #self.get_logger().info(str(self.ros_msg))
            self.pcl_data_publisher_.publish(self.ros_msg)
            print("In Publish PCL Data")
        except:
            self.get_logger().info("Error.")
            print ("ERROR: Serial Port Communication not present.")
            #sys.exit(1)   #Exit if not USB COM Present

    def usb_transfer_array(self):
        """
        Conduct a USB/UART transaction
        Keyword arguments:
        This method (function) uses a USB/UART port to retrieve distance and status values from ST's VL53L8CX (8 x 8) ToF Sensor 
        Returns a 8 x 8 list of float values.
        """
                
        '''
        This function is use to read data from the serial port (UART)
        '''        
        PacketLength = 500          #Maximum packet length allowed
        BufferLength = 0
        PacketReady = False
        FoundHeader = False

        PacketData = []
        RawData = []
        VL53L8CX_Data = []        
        VL53L8CX_Packet_Data = []
        tof_Data = []        

        vl_ctr = 0
        rx_str = ' '
        
        Timeout_ctr = 0  #Use a time out timer in case PIC24 doesn't reply or USB/COM port error

        PacketReady = False
        self.UART_ReceiveBuffer.data.clear()   #---> Python 3 only

        try:
            while PacketReady == False:

                # Decode the bytes sent back from the PIC24
                if self.serial_port.inWaiting() > 1:
                    
                    Timeout_ctr = 0
                    Received_Data = self.serial_port.read(size = 1)
                    #print("Received USB Data: ", Received_Data)

                    #Look for the start header sequence "$$$"    
                    if Received_Data.decode('ASCII') == '$':
                        #print('Received $ character.')  
                        VL53L8CX_Data.append(Received_Data.decode('ASCII'))
                        self.rx_counter += 1 

                        #$ appears three times at the beginning of the packet
                        if self.rx_counter == 3:     
                            if VL53L8CX_Data[0] == '$' and VL53L8CX_Data[1] == '$' and VL53L8CX_Data[2] == '$': 
                                print('Header Found.')  

                                self.HeaderFound = True
                                self.rx_counter = 0

                                print('Start of Row.') 
                                VL53L8CX_Data.clear() 
                                rx_str = ' '
                                vl_ctr = 0   #reset row counter

                    if self.HeaderFound == True:

                        #Collect receive characters
                        rx_str += Received_Data.decode('ASCII')
                        #Start of row
                        if Received_Data.decode('ASCII') == ',':
                                
                            rx_str = rx_str.strip(",")      #Remove commas from the received data  

                            self.UART_ReceiveBuffer.append(rx_str)   #Collect data into a ring (circular) buffer                            
                            rx_str = ''

                        #End of row
                        if Received_Data.decode('ASCII') == ';':
                            rx_str = rx_str.strip(";")      #Remove semi colon from the received data
                            self.UART_ReceiveBuffer.append(rx_str)   #Collect data into a ring (circular) buffer                         
                            rx_str = ''
                        
                            vl_ctr += 1    #increment row counter.
                            #All 8 rows received
                            if vl_ctr >= 8:
                                #print('ToF Packet Data: \n\r ', VL53L8CX_ToF_Data)

                                self.HeaderFound = False
                                vl_ctr = 0    
                                self.rx_counter = 0

                                #VL53L8CX_ToF_Data.clear() 
                                VL53L8CX_Packet_Data.clear() 
                                VL53L8CX_Data.clear() 
                                self.tof_data.clear()   

                                RawData = self.UART_ReceiveBuffer.get()
                                RawData.pop(0)          #Delete first list element. Contains data delimiter values 

                                BufferLength = len(RawData)
                                #print("Buffer Length: ", BufferLength)

                                #Convert received data from string to float
                                i = 0
                                while i < len(RawData):
                                    tof_Data.append(RawData[i].strip(" "))      #Remove the blank spaces
                                    i = i + 1   

                                i = 0
                                while i < len(tof_Data):
                                    if tof_Data[i].isdecimal() == True:
                                        #print(tof_Data[i].isdecimal())
                                        PacketData.append(float(tof_Data[i]))   #Convert string to float
                                    else:
                                        #print(tof_Data[i].isdecimal())
                                        #PacketData.append(tof_Data[i])
                                        PacketData.append(0.0)   #Insert a 0.0 value for non valid receive sensor values
                                    i = i + 1   

                                #print("Raw Data: ", RawData)
                                #print("No Spaces Data: ", tof_Data)
                                #print("Float Data: ", PacketData)

                                # VL53L8CX_Packet_Data.append(RawData[0:16])
                                # VL53L8CX_Packet_Data.append(RawData[17:(17+16)])
                                # VL53L8CX_Packet_Data.append(RawData[34:(34+16)])   
                                # VL53L8CX_Packet_Data.append(RawData[51:(51+16)])   
                                # VL53L8CX_Packet_Data.append(RawData[68:(68+16)])  
                                # VL53L8CX_Packet_Data.append(RawData[85:(85+16)])   
                                # VL53L8CX_Packet_Data.append(RawData[102:(102+16)])
                                # VL53L8CX_Packet_Data.append(RawData[119:(119+16)])

                                #128 Byte package
                                VL53L8CX_Packet_Data.append(PacketData[0:16])
                                VL53L8CX_Packet_Data.append(PacketData[17:(17+16)])
                                VL53L8CX_Packet_Data.append(PacketData[34:(34+16)])   
                                VL53L8CX_Packet_Data.append(PacketData[51:(51+16)])   
                                VL53L8CX_Packet_Data.append(PacketData[68:(68+16)])  
                                VL53L8CX_Packet_Data.append(PacketData[85:(85+16)])   
                                VL53L8CX_Packet_Data.append(PacketData[102:(102+16)])
                                VL53L8CX_Packet_Data.append(PacketData[119:(119+16)])

                                # print("ToF Data:\n\r")
                                # print(VL53L8CX_Packet_Data[0], "\n\r")                                
                                # print(VL53L8CX_Packet_Data[1], "\n\r")
                                # print(VL53L8CX_Packet_Data[2], "\n\r")
                                # print(VL53L8CX_Packet_Data[3], "\n\r")
                                # print(VL53L8CX_Packet_Data[4], "\n\r")
                                # print(VL53L8CX_Packet_Data[5], "\n\r")
                                # print(VL53L8CX_Packet_Data[6], "\n\r")
                                # print(VL53L8CX_Packet_Data[7], "\n\r")

                                print('Last row.\n\r') 

                                tof_Data.clear()
                                PacketData.clear()
                                self.UART_ReceiveBuffer.data.clear()   #---> Python 3 only 

                                PacketReady = True

            #Flush input and output buffers before exiting
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            #print("Received Serial Data")
            #Clear the ring buffer. data is the list/array that collects the values      
            self.UART_ReceiveBuffer.data.clear()   #---> Python 3 only
            
            if PacketReady:               
                print("Returning values to calling function.")
                PacketReady = False
                return VL53L8CX_Packet_Data
            else:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                print("No Valid Data to return.")                         
                PacketReady = False                
                return ""

        except KeyboardInterrupt:
            self.serial_port.close()
            print("Exiting Program")
            exit()
                
        except Exception as exception_error:
            print("Receiving Error (USB). Exiting Program")
            print("Error: " + str(exception_error))

    def computeSinCosTables(self):
        """
	    #This function will save the math processing time of the code.  If the user wishes to not
	    #perform this function, these tables can be generated and saved as a constant.   
        """
        rx_str1 = ' '
        float_num = 0.0

        try:
            print('Computing sin/cos tables')
            for ZoneNum in range(0, 64):
                #print(math.sin(VL53L5_Zone_Pitch8x8[ZoneNum]*(math.pi/180)))

                float_num = (math.sin(VL53L5_Zone_Pitch8x8[ZoneNum]*(math.pi/180)))     
                self.SinOfPitch.append(float_num)      

                float_num = (math.cos(VL53L5_Zone_Pitch8x8[ZoneNum]*(math.pi/180)))
                self.CosOfPitch.append(float_num)

                float_num = (math.sin(VL53L5_Zone_Yaw8x8[ZoneNum]*(math.pi/180)))
                self.SinOfYaw.append(float_num)

                float_num = (math.cos(VL53L5_Zone_Yaw8x8[ZoneNum]*(math.pi/180)))
                self.CosOfYaw.append(float_num)

            print('Computed sin/cos tables')                
            #print("Sin of Pitch: \n\r", self.SinOfPitch) 
            #print("Cos of Pitch: \n\r", self.CosOfPitch) 
            #print("Sin of Yaw: \n\r", self.SinOfYaw) 
            #print("Cos of Yaw: \n\r", self.CosOfYaw) 

        except ValueError:
            print('Not a number')
            return ""
        except IOError:
            print('USB Connection closed.')
            #Try to reconnect with the PIC24 board via USB.

            return ""            
        except Exception as exception_error:
            print("Error in computeSinCosTables()")
            print("Error: " + str(exception_error))

    def calc_xyz_coordinates(self, vl53L8_distance_data):
        """
        Calculate x, y, z coordinates 
        """
        Hyp = 0.0
        SceneDistances = [None] * 64
        Scene_xyz_coordinates = [None] * 8
        xyz_coordinates = [None] * 192

        x_values = []
        y_values = []
        z_values = []

        w_length = 24

        try:
            print('Computing x, y, x coordinates')
            #Distance = vl53L8_distance_data[0]

            #Retrieve the entire 8 x 8 scene distance values
            t = 0    
            for RowNum in range(0, 8):
                i = 0
                x = 0                    
                while i < 8:
                    SceneDistances[t] = vl53L8_distance_data[RowNum][x]
                    #SceneDistances[t] = RowDistance[i]
                    i += 1
                    x += 2
                    t += 1

            #print("Scene Distance Values: ", SceneDistances)
            Scene_xyz_coordinates.clear()
            xyz_coordinates.clear()

            for ZoneNum in range(0, 64):
                Hyp = (SceneDistances[ZoneNum]/(self.SinOfPitch[ZoneNum]))
                #print("Hyp = \n\r", Hyp)
                #X Values
                xyz_coordinates.append(round((self.CosOfYaw[ZoneNum]*self.CosOfPitch[ZoneNum]*Hyp), 3))        # x value
                x_values.append(round((self.CosOfYaw[ZoneNum]*self.CosOfPitch[ZoneNum]*Hyp), 3)) 
                #Y Values
                xyz_coordinates.append(round((self.SinOfYaw[ZoneNum]*self.CosOfPitch[ZoneNum]*Hyp), 3))        # y value 
                y_values.append(round((self.SinOfYaw[ZoneNum]*self.CosOfPitch[ZoneNum]*Hyp), 3))    
                #Z Values
                xyz_coordinates.append(round(SceneDistances[ZoneNum], 3))                                      # z value
                z_values.append(round(SceneDistances[ZoneNum], 3))    
                
                #Insert the intensity value
                #xyz_coordinates.append(0.5)

            BufferLength = len(xyz_coordinates)
            print("------------")
            print("List Length: ", BufferLength)
            print("------------")

            print("\n\rPCL x, y, z and i Coordinates: \n\r", xyz_coordinates)

            Scene_xyz_coordinates.append(xyz_coordinates[0:w_length])
            Scene_xyz_coordinates.append(xyz_coordinates[w_length*1:(w_length*1 + w_length)])
            Scene_xyz_coordinates.append(xyz_coordinates[w_length*2:(w_length*2 + w_length)])   
            Scene_xyz_coordinates.append(xyz_coordinates[w_length*3:(w_length*3 + w_length)])   
            Scene_xyz_coordinates.append(xyz_coordinates[w_length*4:(w_length*4 + w_length)])  
            Scene_xyz_coordinates.append(xyz_coordinates[w_length*5:(w_length*5 + w_length)])   
            Scene_xyz_coordinates.append(xyz_coordinates[w_length*6:(w_length*6 + w_length)])
            Scene_xyz_coordinates.append(xyz_coordinates[w_length*7:(w_length*7 + w_length)])

            # print("Scene x, y, z Coordinates: \n\r")
            # print(Scene_xyz_coordinates[0], "\n\r")                                
            # print(Scene_xyz_coordinates[1], "\n\r")
            # print(Scene_xyz_coordinates[2], "\n\r")
            # print(Scene_xyz_coordinates[3], "\n\r")
            # print(Scene_xyz_coordinates[4], "\n\r")
            # print(Scene_xyz_coordinates[5], "\n\r")
            # print(Scene_xyz_coordinates[6], "\n\r")
            # print(Scene_xyz_coordinates[7], "\n\r")

            Scene_xyz_coordinates.clear()
            #xyz_coordinates.clear()

            print("x Coordinates: \n\r", x_values)  
            print("Y Coordinates: \n\r", y_values)  
            print("Z Coordinates: \n\r", z_values)  

            return xyz_coordinates

        except KeyboardInterrupt:
            #time.sleep(0.5)
            print("Exiting Program")
            exit()

        except Exception as exception_error:
            print("Error in calc_xyz_coordinates()")
            print("Error: " + str(exception_error))

    def xyz_coordinates_plot(self, i, x_vals, y_vals, z_vals):
        """
        Plot x, y, z coordinates 
        """
        #fig = plt.figure()
        #ax = plt.axes(projection='3d')

        try:
            
            print('Plotting x, y, z coordinates')

            #Retrive VL53L8CX data from STM's board
            VL53L8CX_Sensor_Data = self.usb_transfer_array()
            VL53L8CX_xyz_Sensor_Data = self.calc_xyz_coordinates(VL53L8CX_Sensor_Data)  

            # Data for three-dimensional scattered points
            zdata = z_vals #15 * np.random.random(100)
            xdata = x_vals #np.sin(zdata) + 0.1 * np.random.randn(100)
            ydata = y_vals #np.cos(zdata) + 0.1 * np.random.randn(100)

            self.ax.clear()
            self.ax.scatter3D(x_vals, y_vals, z_vals, c=z_vals, cmap='inferno')    #Greens

            #Clear the x, y, z tables
            self.x_values.clear()
            self.y_values.clear()
            self.z_values.clear()

            self.ax.set_title('VL53L8CX ToF Sensor')   
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z Distance (mm)')

            #self.ax.set_xlim(100,100)         

        except KeyboardInterrupt:
            #time.sleep(0.5)
            print("Exiting Program")
            exit()
        except Exception as exception_error:
            print("Error in xyz_coordinates_plot()")
            print("Error: " + str(exception_error))          

#def main(args=None):
def main():
    
    logger = rclpy.logging.get_logger('logger')

    # obtain parameters from command line arguments
    logger.info(str(sys.argv))
    #logger.info(str(len(sys.argv)))
    # if len(sys.argv) != 5:
    #     logger.info('Invalid number of parameters. Usage: \n'
    #                 'Enter VL53L8CX Sensor Number as 1 or 2')
    #     sys.exit(1)

    if len(sys.argv) > 1:
        if sys.argv[1] == '1':
            logger.info('VL53L8CX Sensor 1 Selected')

        if sys.argv[1] == '2':
            logger.info('VL53L8CX Sensor 2 Selected')
    else:
        logger.info('Single Sensor Operation.')

    try:
        while True:
            
            #rclpy.init(args=args)
            rclpy.init()

            node = VL53L8CX(sys.argv) #instantiate the VL53L8CX object
            #node = VL53L8CX() #instantiate the VL53L8CX object            
            rclpy.spin(node)

            print("")
            print("Exiting Application from main.")
            
            node.destroy_node()

            rclpy.shutdown()    #shutdown ROS2 communication by pressing ctrl + c

            time.sleep(0.1)

    except KeyboardInterrupt:
        time.sleep(0.5)
        print("Exiting Program")
        sys.exit(1) 

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

if __name__ == '__main__':
    main()    
