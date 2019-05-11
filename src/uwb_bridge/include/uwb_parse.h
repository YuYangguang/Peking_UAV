#include "stdio.h"
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include "uwb_bridge/uwbMsg.h"

typedef struct
{
    int32_t stampID;
    int32_t sequenceID;
    int8_t IsValid;   //1为有效,0为无效
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double timeStamp;
} uwbData_t;



int32_t char2int32(char* origin,int size,bool invFlag)
{
    if(size != 4)
    {
        printf("Error! the length of data is not correct!");
        return(-99999);
    }
    char *outputMem = new char[size];
    for (int i=0;i<size;i++)
    {
        if(invFlag == true)
        {
            *(outputMem+i)=*(origin+size-1-i);
        }
        else
        {
            *(outputMem+i)=*(origin+i);
        }
    }
    int32_t outputNum;
    memcpy(&outputNum, outputMem, size);
    free(outputMem);
    return(outputNum);
}


int8_t char2int8(char* origin,int size,bool invFlag)
{
    if(size != 1)
    {
        printf("Error! the length of data is not correct!");
        return(-999);  //此处有风险
    }
    char *outputMem = new char[size];
    for (int i=0;i<size;i++)
    {
        if(invFlag == true)
        {
            *(outputMem+i)=*(origin+size-1-i);
        }
        else
        {
            *(outputMem+i)=*(origin+i);
        }
    }
    int8_t outputNum;
    memcpy(&outputNum, outputMem, size);
    free(outputMem);
    return(outputNum);
}


double char2double(char* origin,int size,bool invFlag)
{
    if(size != 8)
    {
        printf("Error! the length of data is not correct!");
        return(-999);
    }
    char *outputMem = new char[size];
    for (int i=0;i<size;i++)
    {
        if(invFlag == true)
        {
            *(outputMem+i)=*(origin+size-1-i);
        }
        else
        {
            *(outputMem+i)=*(origin+i);
        }

    }
    double outputNum;
    memcpy(&outputNum, outputMem, size);
    free(outputMem);
    return(outputNum);
}

void data_parse(char* rawdata, uwbData_t* uwbdata)
{
    char int32buff[4];
    char doublebuff[8];



    memcpy(&int32buff, rawdata, 4);//提取标签ID
    uwbdata->stampID = char2int32(int32buff,sizeof(int32buff),false);

    memcpy(&int32buff, rawdata+4, 4);//提取帧序号
    uwbdata->sequenceID = char2int32(int32buff,sizeof(int32buff),false);
    int8_t temp8;
    memcpy(&temp8, rawdata+8, 1);//提取是否有效
    uwbdata->IsValid = temp8;


    memcpy(&doublebuff, rawdata+9, 8);//提取x
    uwbdata->x = char2double(doublebuff,sizeof(doublebuff),false);

    memcpy(&doublebuff, rawdata+17, 8);//提取y
    uwbdata->y = char2double(doublebuff,sizeof(doublebuff),false);

    memcpy(&doublebuff, rawdata+25, 8);//提取z
    uwbdata->z = char2double(doublebuff,sizeof(doublebuff),false);

    memcpy(&doublebuff, rawdata+33, 8);//提取vx
    uwbdata->vx = char2double(doublebuff,sizeof(doublebuff),false);

    memcpy(&doublebuff, rawdata+41, 8);//提取vy
    uwbdata->vy = char2double(doublebuff,sizeof(doublebuff),false);

    memcpy(&doublebuff, rawdata+49, 8);//提取vz
    uwbdata->vz = char2double(doublebuff,sizeof(doublebuff),false);

    memcpy(&doublebuff, rawdata+57, 8);//提取timeStamp
    uwbdata->timeStamp = char2double(doublebuff,sizeof(doublebuff),false);

}

uwb_bridge::uwbMsg trans2ros(uwbData_t uwbdata)
{
    uwb_bridge::uwbMsg rosMsg;
    rosMsg.stampID = uwbdata.stampID;
    rosMsg.sequenceID = uwbdata.sequenceID;
    rosMsg.IsValid = uwbdata.IsValid;

    rosMsg.x = uwbdata.x;
    rosMsg.y = uwbdata.y;
    rosMsg.z = uwbdata.z;
    rosMsg.vx = uwbdata.vx;
    rosMsg.vy = uwbdata.vy;
    rosMsg.vz = uwbdata.vz;
    rosMsg.timestamp = uwbdata.timeStamp;

    return rosMsg;

}

