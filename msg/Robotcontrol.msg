# the msg is used to store the data about our robot and we can make it run auto
#the cmd to the store node
uint8 cmd 									
#different cmd have different id_msg
string name                                 
#consecutively increasing ID
uint32 id
#the tiem about the msg
time  stamp                                 
#the data length about the data
uint8   length                           
#msg data
float32[] data                              
