#include "lasercontroller.h"


bool LaserController::Open(QString comPort,int baudRate)
{

    port->close();  //reset用
    port->setPortName(comPort);
    port->setBaudRate(BAUD9600);  //包率
    port->setFlowControl(FLOW_OFF);  //滿載控制
    port->setParity(PAR_NONE);  //同位元
    port->setDataBits(DATA_8);  //資料大小
    port->setStopBits(STOP_1);  //停止bit
    port->open(QIODevice::ReadWrite);

    char LMS38400cmd[8] = {0x02,0x00,0x02,0x00,0x20,0x40,0x50,0x08};
    port->write(LMS38400cmd,8);

    cv::waitKey(300);
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
    port->open(QIODevice::ReadWrite|QIODevice::Unbuffered);

    return port->isOpen();
}

void LaserController::TriggerLaser()
{

    char alpha[8] = {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
    port->write(alpha, 8);
}

bool LaserController::ReadData(std::vector<double> &scanData)
{
    bool state=false;
    scanData.clear();

    int num=port->bytesAvailable();

    if(num > 0 && num < 1024)
    {
        QByteArray temp=port->read(num);

        if( temp.size() == 733 ){

            std::vector<unsigned char> data;
            for(int i = 10; i < 733; i++)
            {
                unsigned char tp = temp[i];
                data.push_back(tp);

            }

            for(int j=0; j < data.size() - 1; j+=2)
            {
                double r = data[j+1] * 256 + data[j];
                scanData.push_back(r);
            }

            state==true;

        }

    }

    return state;
}

bool LaserController::Close()
{

    port->close();
    return port->isOpen();
}
