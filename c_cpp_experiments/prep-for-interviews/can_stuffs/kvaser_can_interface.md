
* Get number of channels

* get cAN channel data 

* Open Channel

* Set bit rate and oter parameters

*  Linuxcan defaults to echo on, so if you've opened the same can channel
 from multiple interfaces they will receive the messages that each other  send.  Turn it off here if desired.

* Set output control


* Example CAN READ 


canStatus canRead	(	const int 	hnd,
long * 	id,
void * 	msg,
unsigned int * 	dlc,
unsigned int * 	flag,
unsigned long * 	time 
)	



[in]	hnd	A handle to an open circuit.
[out]	id	Pointer to a buffer which receives the CAN identifier. This buffer will only get the identifier. To determine whether this identifier was standard (11-bit) or extended (29-bit), and/or whether it was remote or not, or if it was an error frame, examine the contents of the flag argument.
[out]	msg	Pointer to the buffer which receives the message data. This buffer must be large enough (i.e. 8 bytes for classic CAN and up to 64 bytes for CAN FD).
[out]	dlc	Pointer to a buffer which receives the message length.
[out]	flag	Pointer to a buffer which receives the message flags, which is a combination of the canMSG_xxx (including canFDMSG_xxx if the CAN FD protocol is enabled) and canMSGERR_xxx values.
[out]	time	
[out]	time	Pointer to a buffer which receives the message time stamp


* Example CAN Write

 canWrite()
canStatus canWrite	(	const int 	hnd,
long 	id,
void * 	msg,
unsigned int 	dlc,
unsigned int 	flag 
)	


* Parameters
[in]	hnd	A handle to an open CAN circuit.
[in]	id	The identifier of the CAN message to send.
[in]	msg	A pointer to the message data, or NULL.
[in]	dlc	The length of the message in bytes.
For Classic CAN dlc can be at most 8, unless canOPEN_ACCEPT_LARGE_DLC is used.
For CAN FD dlc can be one of the following 0-8, 12, 16, 20, 24, 32, 48, 64.
[in]	flag	A combination of message flags, canMSG_xxx (including canFDMSG_xxx if the CAN FD protocol is enabled). Use this parameter to send extended (29-bit) frames and/or remote frames. Use canMSG_EXT and/or canMSG_RTR for this purpose.
