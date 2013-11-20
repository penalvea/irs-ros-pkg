#include "ARM5UJIONEComs.h"
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <string.h>

/** Open the RS232 communication. Returns the file descriptor (-1 if error) */
int ARM5Coms::OpenPort(std::string comPort)
{
    std::cerr << "Calling open on " << comPort << std::endl;
    rs232_fd = open(comPort.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (rs232_fd < 0)
    {
        std::cerr << "ARM5Coms::OpenPort ERROR: cannot open the RS232 port" << std::endl;
    }
    else
    {
        struct termios my_termios;
        std::cerr << "ARM5Coms::OpenPort: RS232 port opened" << std::endl;
        tcgetattr(rs232_fd, &my_termios);
     
        tcflush(rs232_fd, TCIFLUSH);
        
        my_termios.c_cflag = B115200 | CS8 |CREAD; // | CLOCAL | HUPCL;
	my_termios.c_iflag = IGNPAR;
	
	// set input mode (non-canonical, no echo,...), blocking*/
	my_termios.c_lflag = 0;
        my_termios.c_cc[VTIME]    = 5;   /* inter-character timer unused */
        my_termios.c_cc[VMIN]     = 51;   /* blocking read until 51 chars received */

	tcflush(rs232_fd, TCIFLUSH);
	tcsetattr(rs232_fd,TCSANOW,&my_termios);
    }
    return rs232_fd;
}

/** Reads from the serial port and leaves the message in RXData and ChanneliRX 
 *  Version reading a 51-byte block
**/
void ARM5Coms::readMessage()
{ 
	byte buffer[51];
	//std::cerr << "Reading data..." << std::endl;
	int bytes_read;
	do {
		bytes_read=read(rs232_fd,buffer,51); 
		//std::cerr << "Received package of " << bytes_read << " size." << std::endl;
		//Ugly hack. Sometimes the arm stops sending back messages and moving until we send zero velocities
		//So, if read fails with EAGAIN, sends a zero velocities message
		if (bytes_read==-1) {
			perror("Recovering from read error: ");
			Channel2.SpeedDemand(0, 0xffff, 0x0fff);
			Channel1.SpeedDemand(0, 0xffff, 0x0fff);
			Channel3.SpeedDemand(0, 0xffff, 0x0fff);
			Channel4.SpeedDemand(0, 0xffff, 0x0fff);
			Channel5.SpeedDemand(0, 0xffff, 0x0fff);
			sendMessage();
		}
		usleep(10000);
	} while (bytes_read==-1);

	//printf("%02X %02X\n",buffer[0], buffer[50]);
        if (bytes_read==datalength)
        {
            //std::cerr << "51-byte message received" << std::endl;
	    memcpy(RXDATA,buffer,datalength*sizeof(byte));
            if (RXDATA[datalength - 1] == EOM && RXDATA[0] == SOM)
            {
		//std::cerr << "EOM and SOM are correct" << std::endl;
                if (RXDATA[datalength - 2] == Checksum(RXDATA, datalength - 2))
                {
                    //std::cerr << "Checksum is correct. Storing." << std::endl;

                    int i = 1;
                   
                    int ii = 0; //Extract Channel0
                    while (ii < ChannelMasterLength)
                    {
                        Channel0RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    masterTemprature=Channel0RX[0];
                    masterVoltage=Channel0RX[1];
                    masterCurrent = Channel0RX[2];

                    ii = 0; //Extract Channel1
                    while (ii < 9)
                    {
                        Channel1RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel2
                    while (ii < 9)
                    {
                        Channel2RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel3
                    while (ii < 9)
                    {
                        Channel3RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel4
                    while (ii < 9)
                    {
                        Channel4RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel5
                    while (ii < 9)
                    {
                        Channel5RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    Channel1.RS232DataIn(Channel1RX);
                    Channel2.RS232DataIn(Channel2RX);
                    Channel3.RS232DataIn(Channel3RX);
                    Channel4.RS232DataIn(Channel4RX);
                    Channel5.RS232DataIn(Channel5RX);
		    msg_complete=true;
                } else msg_complete=false;
            } else msg_complete=false;
        }
}

/** Reads from the serial port and leaves the message in RXData and ChanneliRX 
 *  Version using a queue

void ARM5Coms::readMessage()
{ 	
        while (read(rs232_fd,&(RXDATA[datalength-1]),1)>0)
        {
            //std::cerr << "Read " << bytes_read << " bytes" << std::endl;
            if (RXDATA[datalength - 1] == EOM && RXDATA[0] == SOM)
            {
		std::cerr << "Full message received. Processing it..." << std::endl;
                if (RXDATA[datalength - 2] == Checksum(RXDATA, datalength - 2))
                {
                    std::cerr << "Checksum is correct. Storing." << std::endl;

                    int i = 1;
                   
                    int ii = 0; //Extract Channel0
                    while (ii < ChannelMasterLength)
                    {
                        Channel0RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    masterTemprature=Channel0RX[0];
                    masterVoltage=Channel0RX[1];
                    masterCurrent = Channel0RX[2];

                    ii = 0; //Extract Channel1
                    while (ii < 9)
                    {
                        Channel1RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel2
                    while (ii < 9)
                    {
                        Channel2RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel3
                    while (ii < 9)
                    {
                        Channel3RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel4
                    while (ii < 9)
                    {
                        Channel4RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    ii = 0; //Extract Channel5
                    while (ii < 9)
                    {
                        Channel5RX[ii] = RXDATA[i];
                        i++; ii++;
                    }

                    Channel1.RS232DataIn(Channel1RX);
                    Channel2.RS232DataIn(Channel2RX);
                    Channel3.RS232DataIn(Channel3RX);
                    Channel4.RS232DataIn(Channel4RX);
                    Channel5.RS232DataIn(Channel5RX);
		    msg_complete=true;
		    std::cerr << "Message OK" << std::endl;
                } else {
			 msg_complete=false;
			std::cerr << "Checksum error" << std::endl;
		}
            } else { 
		msg_complete=false;
		std::cerr << "Message incomplete" << std::endl;
	    }

            for (int i = 0; i < datalength-1; i++)//shift buffer
            { RXDATA[i] = RXDATA[i + 1]; }
        }
}
*/


/** send a message to the arm */ 
byte* ARM5Coms::sendMessage()
{
    byte* message = new byte[datalength];
    byte* channel1;
    byte* channel2;
    byte* channel3;
    byte* channel4;
    byte* channel5;

    // build message for master controller

    message[0] = SOM;// Start of message

//    message[1] = 0xff;//Master Current Limit
//    message[2] = 0xff;//Master UnderVoltage Limit
    message[1] = 0x00;//Master Current Limit
    message[2] = 0x00;//Master UnderVoltage Limit
    message[3] = 0x00; //Master OverVoltage Limit

    channel1=Channel1.RS232DataOut();
    message[4] = channel1[0]; //Reqest/Demand Byte
    message[5] = channel1[1]; //data0
    message[6] = channel1[2]; //data1
    message[7] = channel1[3]; //data2
    message[8] = channel1[4]; //data3
    message[9] = channel1[5]; //data4
    message[10] = channel1[6]; //data5
    message[11] = channel1[7]; //data6
    message[12] = channel1[8]; //data7
    delete [] channel1;


    channel2=Channel2.RS232DataOut();
    message[13] = channel2[0]; //Reqest/Demand Byte
    message[14] = channel2[1]; //data0
    message[15] = channel2[2]; //data1
    message[16] = channel2[3]; //data2
    message[17] = channel2[4]; //data3
    message[18] = channel2[5]; //data4
    message[19] = channel2[6]; //data5
    message[20] = channel2[7]; //data6
    message[21] = channel2[8]; //data7
    delete [] channel2;

    channel3=Channel3.RS232DataOut();
    message[22] = channel3[0]; //Reqest/Demand Byte
    message[23] = channel3[1]; //data0
    message[24] = channel3[2]; //data1
    message[25] = channel3[3]; //data2
    message[26] = channel3[4]; //data3
    message[27] = channel3[5]; //data4
    message[28] = channel3[6]; //data5
    message[29] = channel3[7]; //data6
    message[30] = channel3[8]; //data7
    //std::cerr << "Channel 3: " << (int)message[22] << " " << (int)message[23] << " [...] " << (int)message[30] << std::endl;
    delete [] channel3;

    channel4=Channel4.RS232DataOut();
    message[31] = channel4[0]; //Reqest/Demand Byte
    message[32] = channel4[1]; //data0
    message[33] = channel4[2]; //data1
    message[34] = channel4[3]; //data2
    message[35] = channel4[4]; //data3
    message[36] = channel4[5]; //data4
    message[37] = channel4[6]; //data5
    message[38] = channel4[7]; //data6
    message[39] = channel4[8]; //data7
    delete [] channel4;

    channel5=Channel5.RS232DataOut();
    message[40] = channel5[0]; //Reqest/Demand Byte
    message[41] = channel5[1]; //data0
    message[42] = channel5[2]; //data1
    message[43] = channel5[3]; //data2
    message[44] = channel5[4]; //data3
    message[45] = channel5[5]; //data4
    message[46] = channel5[6]; //data5
    message[47] = channel5[7]; //data6
    message[48] = channel5[8]; //data7
    delete [] channel5;

    message[datalength - 2] = Checksum(message, datalength - 2);
    message[datalength-1] = EOM;

    
    //send through RS232
    int result=write(rs232_fd,message,datalength);

    //std::cerr << "Sent package: " << result << " bytes writen" << std::endl;
 
    return message;
}



/** Compute the checksum of the message */
byte ARM5Coms::Checksum(byte* inputarray, byte LocationOfChecksum)
{
    byte ichk=0;
    for (int i = 0; i < LocationOfChecksum; i++)
    {
        ichk = (byte)(ichk + inputarray[i]);
    }
    
    return ichk;
}




