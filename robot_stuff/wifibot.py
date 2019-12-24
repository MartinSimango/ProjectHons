import socket
import numpy as np
import time
import codecs
from struct import *

class Wifibot():
	
	bot_state=[0,0,0,0]
	def __init__(self,ip):
		self.IP_ADDR= ip
		self.PORT = 15020
		self.counter=0
		self.skt= socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		#self.skt.settimeout(1.5)
		self.skt.connect((self.IP_ADDR,self.PORT))

	def crc16(self,data):
		Crc= 0xFFFF
		Polynome= 0xA001
		i=1
		Parity = 0
		while i<len(data):
			Crc^=(data[i])
			k=0
			while k<=7:
				Parity = Crc
				Crc>>=1
				if(Parity%2==True):
					Crc^=Polynome
				k+=1
			i+=1
		output_crc = (int(hex(Crc)[-2:],16),int(hex(Crc)[2:4],16))
		return output_crc
	def left_dist(self):
		return self.bot_state[2]
	def right_dist(self):
		return self.bot_state[3]
	def move(self,left_speed,right_speed):
                #basic format

                #to move forward = 128*0 + 64*1 + speed*1 = speedtosend
                #to move backward = 128*0 + 64*0 + speed*1= speedtosend
				#  128*0 = no motor control
				# 128 *1 = motor control
		data= [None]*2
		if(left_speed<0):
			left_speed= 128*1 + 64*0 + abs(left_speed)*1
		else:
			left_speed= 128*1 + 64*1 + abs(left_speed)*1
		if(right_speed<0):
			right_speed= 128*1 + 64*0 + abs(right_speed)*1
		else:
			right_speed= 128*1 + 64*1 + abs(right_speed)*1
		
		data[0]=left_speed
		data[1]=right_speed
		x= '02x'
		#data[0] =11 #255
		#data[1] =11 #7
		#data[6]= 0b01011111
		#if left_speed<0:
		#	data[6]&=0b11101111
		#if right_speed<0:
		#	data[6]&=0b11101111
		#data[2]= abs(left_speed)&0xff
		#data[3]= 0
		#data[4]= abs(right_speed)&0xff
		#data[5]= 0
		#data.extend(self.crc16(data))
		tempData=[]
		for i in range(len(data)):
			tempData.append(format(data[i],x))
			#print(data[i])
		tempData = ''.join(tempData)
		#tempData = tempData.encode("utf-8").encode("hex")
		#print(tempData)
		outputstr= codecs.decode(tempData,"hex") 
		#outputstr= ''.join(tempData).decode('hex')
		self.skt.send(outputstr)
		connected = self.receive_data()
		if(not connected):
			print("Reconnecting")
			self.skt.close()
			self.skt=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		
			self.skt.connect((self.IP_ADDR,self.PORT))

	def receive_data(self):
		spd_front_L= self.skt.recv(2)
		bat_level = self.skt.recv(1)
		IR1_L = self.skt.recv(1)
		IR2_L = self.skt.recv(1)
		odometry_L = self.skt.recv(4)

		spd_front_R= self.skt.recv(2)
		IR1_R = self.skt.recv(1)
		IR2_R = self.skt.recv(1)
		odometry_R = self.skt.recv(4)

		current= self.skt.recv(1)
		ver = self.skt.recv(1)
		CRcret=self.skt.recv(2)

		if len(spd_front_L) ==0 or len(spd_front_R)==0:
			return 0
		#self.bot_state=[unpack('h',spd_front_L)[0],unpack('h',spd_front_R)[0],unpack('l',odometry_L)[0],unpack('l',odometry_R)[0]]
		return 1	

