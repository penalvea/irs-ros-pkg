//Classes Motor and ARM5Coms for communication with CSIP ARM5E via RS232 on Linux
//Mario Prats. August 2010

#ifndef ARM5COMS_H
#define ARM5COMS_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

typedef char byte;
typedef unsigned long int UInt32;
typedef unsigned int UInt16;
typedef long int Int32;
typedef int Int16;
typedef unsigned int uint;



class Motor
{
	 bool PIDupdate;
	 bool PIDsent;

	 Int32 Demand;
	 byte DemandType; // 0 for Emergancy Stop/Brake, 1 voltage, 2 speed, 3 postion, 4 current
	 UInt16 SpeedLimit;
	 UInt16 CurrentLimit;
	

	//PID Values
	 byte SpeedP;
	 byte SpeedI;
	 byte SpeedD;
	 byte PostionP;
	 byte PostionI;
	 byte PostionD;

	 UInt16 Position;	//position ticks relative to the switch on position
	 UInt16 Speed;
	 UInt16 Current;
	 byte CurrentPeek;
	 byte CurrentAvg;
	 byte Water;
	 byte Temp;
	 byte Voltage;

	 //Variables for converting position range 0-65535 into a wider and signed range -65535 - 65535
	 int signedRelativePosition;	//position ticks relative to the switch on position in the range from -65535 - 65535
	 bool signedRelativePositionInit; //whether signedRelativePosition has been initialized or not

	public:
	


	Motor() {
		PIDupdate = true;
	 	PIDsent = false;
		SpeedLimit = 0xffff;
		CurrentLimit = 0xffff;
		CurrentPeek=0;
		CurrentAvg=0;
		signedRelativePosition=0;
		signedRelativePositionInit=false;
		//offsetInit=false;
		//signedAbsolutePosition=0;
		//limitOffset=0;
		//q=0;
		Position=0;
		Speed=0;
		Current=0;
	}

	double rawPosition() {
		return Position;
	}

	/// <summary>
	/// the display varable for position
	/// </summary>
	double disPosition()
	{
		return signedRelativePosition;
	    //if (offsetInit)
	    //	return q;
	    //else return signedRelativePosition;
	}

	 uint disSpeed()
	{
	    return Speed;
	}

	uint disCurrent()
	{
	   return Current;
	}
	
    Int32 disDemand()
	{
	   return Demand;
	}
	
	byte disDemandType()
	{
	   return DemandType;
	}
	
	UInt16 disSpeedLimit()
	{
	   return SpeedLimit;
	}
	
	UInt16 disCurrentLimit()
	{
	   return CurrentLimit;
	}	

	/// <summary>
	/// the display varable for peek current
	/// </summary>
	byte disCurrentPeek()
	{
	    return CurrentPeek;
	}

	/// <summary>
	/// the display varable for Avg current
	/// </summary>
	byte disCurrentAvg()
	{
	    return CurrentAvg;
	}

	/// <summary>
	/// the display varable for Water
	/// </summary>
	byte disWater()
	{
	    return Water;
	}

	/// <summary>
	/// the display varable for tempature
	/// </summary>
	byte disTemp()
	{	
	    return ((((double)Temp / 255) * 3.3) / 0.0066101694915254237288135593220339);
	}

	/// <summary>
	/// the display varable for Voltage
	/// </summary>
	byte disVoltage()
	{
	   return Voltage;
	}

	/// <summary>
	/// Load Demand values into the TX buffer
	/// Stop = 0|
	/// Voltage = 1|
	/// Speed = 2|
	/// Position = 3|
	/// Current = 4|
	/// </summary>
	/// <param name="Demand"></param>
	/// <param name="DemandType"></param>
	void ValueDemand(byte demandType, Int32 demand, UInt16 speedLimit, UInt16 currentLimit, UInt16 PositionMin, UInt16 PositionMax)
	{
	    if (demandType == 0)//Stop Demand
	    {
		this->Demand = 0;
		this->DemandType = 0;
	    }


	    if (demandType == 1)//Voltage Demand
	    {
		if (demand<0) {
			this->DemandType=1;
			this->Demand=(UInt16)(-demand);
		} else if (demand>0) {
			this->DemandType=2;
			this->Demand=(UInt16)(demand);
		} else {
			this->DemandType = 0; //brake
			this->Demand=0;
		}
	    }

	    if (demandType == 2)//speed Demand
	    {
		if (demand<0) {
			this->DemandType=4;
			this->Demand=(UInt16)(-demand);
		} else if (demand>0) {
			this->DemandType=3;
			this->Demand=(UInt16)(demand);
		} else {
			this->DemandType = 0; //brake
			this->Demand=0;
		}
	    }


	    //Position Demand. TODO
	    if (demandType == 3)
	    {
		std::cerr << "ERROR: Position demand still not implemented. Doing nothing." << std::endl;
		this->Demand = 0;
		this->DemandType = 0;

		/*
		int scale, percentOfScale;

		if (PositionMax < PositionMin) // this condition has rolled over on the joystick
		{
		    scale = PositionMax + (0xffff - PositionMin) + 1;
		    percentOfScale = (UInt16)(int)(((float)(demand >> 1) / (float)0xffff) * scale + PositionMin);
		}
		else
		{
		    scale = PositionMax - PositionMin;
		    percentOfScale = (UInt16)(((float)(demand >> 1) / (float)0xffff) * scale + PositionMin);

		}
		this->Demand = percentOfScale ;
	       // System.Console.WriteLine(percentOfScale);
		this->DemandType = 5;
		*/
	    }

	    if (demandType == 4)//Current Demand NOT DEFINED
	    {
		this->Demand = 0;
		this->DemandType = 6;
		
	    }
	    this->SpeedLimit = speedLimit;
	    this->CurrentLimit = currentLimit;
	}

	///< Easy way to set a speed demand
	void SpeedDemand(Int32 demand, UInt16 speedLimit=0xffff, UInt16 currentLimit=0xffff) {
		ValueDemand(2,demand,speedLimit,currentLimit,0,0);
	}

	/// <summary>
	/// Load PID values into the TX buffer
	/// </summary>
	/// <param name="speedP"></param>
	/// <param name="speedI"></param>
	/// <param name="speedD"></param>
	/// <param name="postionP"></param>
	/// <param name="postionI"></param>
	/// <param name="postionD"></param>
	void ValuePID(byte postionP, byte postionI, byte postionD, byte speedP, byte speedI, byte speedD)
	{
	    this->SpeedP = speedP;
	    this->SpeedI = speedI;
	    this->SpeedD = speedD;
	    this->PostionP = postionP;
	    this->PostionI = postionI;
	    this->PostionD = postionD;
	}

	void pidupdate()
	{
	    PIDupdate = true;
	}

	void RS232DataIn(byte *Value)
	{
	    if (PIDsent == true) // this persume we got a responce from the last send, if PID sent is true then we can reset the cycle
	    {
		PIDsent = false;
		PIDupdate = false;
	    }

	    if ((Value[0] & 0x0f) == 0) //demand has come back from the manipulator, this is unusal
	    {    
	    }

	    if ((Value[0] & 0x0f) == 1) //Sensors has come back from the manipulator
	    {
		UInt16 lastPosition=Position;
		Position = (UInt16)((Value[1] & 0xff) + ((Value[2]<<8) & 0xff00));
		if (!signedRelativePositionInit) {
			signedRelativePositionInit=true;
			signedRelativePosition=(int)Position;
		} else {
			int increment=(int)Position-(int)lastPosition;
			if (abs(increment)>63000) {
				//unsigned int overflow detected
				if (Position>lastPosition) { //overflow from 0 to 65535
					increment=-(lastPosition+(65536-Position));
					//signedRelativePosition-= lastPosition+(65536-Position);
				} else { //overflow from 65535 to 0
					increment=Position+(65536-lastPosition);
					//signedRelativePosition+= Position+(65536-lastPosition);
				}
			}
			if (abs(increment)>800) {
				std::cerr << "WARNING: High position jump detected (increment=" << increment << "). Possible zero peak. Ignoring..." << std::endl;
			} else {
				signedRelativePosition+=increment;
			}
			
		}
	
		
		Speed = (UInt16)((Value[3] & 0xff) + ((Value[4] << 8) & 0xff00));
		Current = (UInt16)((Value[7] & 0xff) + ((Value[8] << 8) & 0xff00));

	    //    System.Console.WriteLine("Current= " + Current);
	//          CurrentPeek = Value[3];
	//        CurrentAvg = Value[4];
		Water = Value[5];
		Temp  = Value[6];
		Voltage = Value[7];
		//std::cerr << "Voltage: " << (int)Voltage << std::endl;
	      //  System.Console.WriteLine("sensors" + Value[1] + " " + Value[2] + " " + Value[3] + " " + Value[4] + " " + Value[5] + " " + Value[6] + " " + Value[7] + " " + Value[8]);
	    }

	    if ((Value[0] & 0x0f) == 2) //CAL has come back from the manipulator (this Might not exist in the drive)
	    {
	    }
	    if ((Value[0] & 0x0f) == 3) //PID has come back from the manipulator
	    {
	    }
	    
	}

	byte *RS232DataOut()
	{
	    byte *message = new byte[9];
	    byte AddressUp = 0;

	    if (PIDupdate == true) // clears after update, high piorty return
	    {
		message[0] = (byte)(1 + AddressUp); // can buffer 2
		message[1] = PostionP; //Postion P
		message[2] = PostionI; //Postion I 
		message[3] = PostionD; //Postion D
		message[4] = SpeedP; //Speed P
		message[5] = SpeedI; //Speed I 
		message[6] = SpeedD; //Speed D
		message[7] = 0; //unused
		message[8] = 0; //unused
		PIDsent = true;//
		return message;
	    }

	    //std::cerr << "Limits: " << SpeedLimit << " " << CurrentLimit << std::endl;
	    //no specal messages, return the demand
	    message[0] = (byte)(0 + AddressUp); // can buffer 0
	    message[1] = DemandType;
	    
	    //Avoid speed demand with 0x0a in the LSM to be sent, send 0x0b instead
	    if((byte)(Demand & 0xff)==0x0a)
			Demand +=1;
	    
			
	    message[2] = (byte)(Demand >> 8 & 0xff); //Demand MSB 
	    message[3] = (byte)(Demand & 0xff); //Demand LSB
	    message[4] = (byte)(SpeedLimit >> 8 & 0xff); //Speed Limit MSB
	    message[5] = (byte)(SpeedLimit & 0xff); //Speed Limit LSB
	    message[6] = (byte)(CurrentLimit >> 8 & 0xff); //Speed Limit MSB
	    message[7] = (byte)(CurrentLimit & 0xff); //Demand Type and staus
	    message[8] = 0;          //not defined for future use
	    PIDsent = false;//
	    return message;
	}
};


class ARM5Coms {
	const static byte EOM = 0xE5; //EOM
	const static byte SOM = 0xE7; //SOM
	const static int ChannelMasterLength = 3;
	const static byte Channels = 5; //Number of Channels
	const static byte datalength = 1 + ChannelMasterLength + (Channels * 9) + 1 + 1; //4 + 45 +2    
	byte RXDATA[datalength];

	byte Channel0buffer[3];
	byte Channel1buffer[9];
	byte Channel2buffer[9];
	byte Channel3buffer[9];
	byte Channel4buffer[9];
	byte Channel5buffer[9];

	byte Channel0RX[3];
	byte Channel1RX[9];
	byte Channel2RX[9];
	byte Channel3RX[9];
	byte Channel4RX[9];
	byte Channel5RX[9];

	byte masterVoltage;
	byte masterCurrent;
	byte masterTemprature;

	int rs232_fd;
	bool msg_complete;	//Complete msg received from the arm

public:
	Motor Channel1, Channel2, Channel3, Channel4, Channel5;

	ARM5Coms() {
		rs232_fd=-1;
		msg_complete=false;
	}

	int OpenPort(std::string comPort);

	byte Checksum(byte *inputarray, byte LocationOfChecksum);

	/** Receive a message from the arm */
	void readMessage();

	/** Send a message to the arm */ 
	byte *sendMessage();

	double MasterVoltage()
        {
                return ((double)masterVoltage / (double)255 * (double)3.3) / ((double)6800 / (double)111500);
        }

        double MasterCurrent()
        {
                return ((((double)masterCurrent / (double)511 * (double)3.3) / (double)((double)39 / (double)59)) / (double)0.625) * (double)6-0.2;
        }

        double MasterTemprature()
        {
                return ((((double)masterTemprature / 255) * (double)3.3) / (double)0.0066101694915254237288135593220339);
        }

	~ARM5Coms() {if (rs232_fd) { close(rs232_fd); std::cerr << "RS232 port closed" << std::endl;} }
};

#endif
