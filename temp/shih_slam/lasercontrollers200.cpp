#include "lasercontrollers200.h"

using namespace std;

bool LaserControllerS200::Open(QString comPort, int baudRate)
{

    BaudRateType baudRateSymbol;
    switch(baudRate){
        case 9600:
            baudRateSymbol=BAUD9600;
            break;

        case 19200:
            baudRateSymbol=BAUD19200;
            break;

        case 38400:
            baudRateSymbol=BAUD38400;
            break;

        case 76800:
            baudRateSymbol=BAUD76800;
            break;

        case 115200:
            baudRateSymbol=BAUD115200;
            break;

        default:
            baudRateSymbol=BAUD9600;

    };

    port->close();  //reset用
    port->setPortName(comPort);
    port->setBaudRate(baudRateSymbol);  //包率
    port->setFlowControl(FLOW_OFF);  //滿載控制
    port->setParity(PAR_NONE);  //同位元
    port->setDataBits(DATA_8);  //資料大小
    port->setStopBits(STOP_1);  //停止bit
    port->open(QIODevice::ReadWrite);
    //Get token (Block 25)
    char tokenCmd[] = {0x00, 0x00, 0x41, 0x44, 0x19,
                           0x00, 0x00, 0x05, 0xff, 0x07,
                           0x19, 0x00, 0x00, 0x05, 0xff,
                           0x07, 0x07, 0x0f, 0x9f, 0xd0};


    port->write(tokenCmd,20);


    return port->isOpen();
}

void LaserControllerS200::TriggerLaser()
{
    char alpha[] = {0x00, 0x00, 0x45, 0x44, 0x0c,
                                0x00, 0x02, 0x22, 0xff, 0x07};
    port->write(alpha, 10);


}

bool LaserControllerS200::ReadData(std::vector<double> &scanData)
{
    bool state=false;
    scanData.clear();
    scanData.reserve(540);

    int num=port->bytesAvailable();


    if(num>0){

        if(num > 1096)
              num = 1096;


    //char buf[1096] = {0};

    QByteArray buf=port->read(num);

   // qint64 lineLength = port->read(buf, sizeof(buf));

    if(buf.size()==1096)
    {
        state=true;
        int count=0;
        for(int i=0;i<buf.size();i+=2)  //the first 12 bytes are not range data
        {

            if(count < 540)                         //Collect the data
            {


                double r;
                double xx=buf[i];

                if(xx>=0)
                {

                    r=buf[i+1]*256.0+buf[i];//2 bytes one data

                }
                else
                {
                    r=buf[i+1]*256.0+(buf[i]+256.0);//+256.0;//2 bytes one data

                }



                 if(r<=0)
                     r=0;

                if (r>=S200_MAX_RANGE)
                    r=S200_MAX_RANGE;

                scanData.push_back(r);
           }


            count++;


        }

    }

    }
    //clear buffer
    QByteArray buf;
    do {
        buf = port->readAll();
    } while (buf.size() > 0);

    return state;
}

bool LaserControllerS200::Close()
{

    port->close();
    return port->isOpen();
}
