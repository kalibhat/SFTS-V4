/*
 * cl_datamanager.h
 *
 * Created: 6/17/2016 5:15:03 PM
 *  Author: user
 */ 


#ifndef CL_DATAMANAGER_H_
#define CL_DATAMANAGER_H_


typedef union 
{
	Cl_Uint8Type bytearray[4];
	Cl_Uint16Type twoByte;
	Cl_Uint32Type word;
	
}rxstreamtype;



#endif /* CL_DATAMANAGER_H_ */