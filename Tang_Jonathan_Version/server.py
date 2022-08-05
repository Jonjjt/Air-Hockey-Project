# # Python UDP Receiver

# import socket
# BUFFER_LEN = 100 #in bytes

# def initUDP(IP, port):
#     #Create a datagram socket
#     sock = socket.socket(socket.AF_INET, # Internet
#                          socket.SOCK_DGRAM) # UDP
#     #Enable immediate reuse of IP address
#     sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
#     #Bind the socket to the port
#     sock.bind((IP, port))
#     #Set a timeout so the socket does not block indefinitely when trying to receive data
#     sock.settimeout(0.5)

#     return sock

# def readUDP(sock):
#     try:
#         data, addr = sock.recvfrom(BUFFER_LEN)
#     except socket.timeout as e:
#         return b'Error'
#     except Exception as e:
#         return b'Error'
#     else:
#         return data


# sock = initUDP("127.0.1.1",8888)
# while True:
#     data = readUDP(sock)
#     print(data)


import socket

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)      # For UDP

udp_host = socket.gethostname()		        # Host IP
udp_port = 59200			                # specified port to connect

#print type(sock) ============> 'type' can be used to see type 
				# of any variable ('sock' here)

sock.bind((udp_host,udp_port))
print (udp_host)
print (udp_port)

while True:
	print "Waiting for client..."
	data,addr = sock.recvfrom(1024)	        #receive data from client
	print "Received Messages:",data," from",addr

# udp_host = socket.gethostname()

# myIP = socket.gethostbyname(udp_host)
# print(myIP)
