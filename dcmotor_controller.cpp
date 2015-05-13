#include "dcmotor_controller.h"

DCMotor_Controller::DCMotor_Controller()
{
    connect(this, SIGNAL(readyRead()), this, SLOT(Response()));
}

DCMotor_Controller::~DCMotor_Controller()
{

}

// Setting encoder to zero
void DCMotor_Controller::SetHome()
{
    std::stringstream cmd;
    cmd << "HO\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
    command_queue.push(command::OK);
    std::cout << "command_queue size = " << command_queue.size() << std::endl;
}

void DCMotor_Controller::GoHome()
{
//    const char* temp;
//    temp = "LA0\n";
//    port->write(temp,strlen(temp));
//    temp = "M\n";
//    port->write(temp,strlen(temp));
}

void DCMotor_Controller::Open(const QString &comport, const int baudrate)
{
    this->QSerialPort::close();                 // reset the comport
    this->QSerialPort::setPortName(comport);
    switch (baudrate) {
    case 9600:
        this->QSerialPort::setBaudRate(QSerialPort::Baud9600);
        break;
    case 19200:
        this->QSerialPort::setBaudRate(QSerialPort::Baud19200);
        break;
    case 38400:
        this->QSerialPort::setBaudRate(QSerialPort::Baud38400);
        break;
    case 115200:
        this->QSerialPort::setBaudRate(QSerialPort::Baud115200);
        break;
    default:
        this->QSerialPort::setBaudRate(QSerialPort::Baud9600);
    }
    this->QSerialPort::setDataBits(QSerialPort::Data8);
    this->QSerialPort::setParity(QSerialPort::NoParity);
    this->QSerialPort::setStopBits(QSerialPort::OneStop);
    this->QSerialPort::setFlowControl(QSerialPort::NoFlowControl);
    this->QSerialPort::open(QSerialPort::ReadWrite);
}

void DCMotor_Controller::SetMaxVelocity(double v_rpm)
{
    std::stringstream cmd;
    cmd << "SP" << v_rpm << "\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
    command_queue.push(command::OK);
    std::cout << "command_queue size = " << command_queue.size() << std::endl;
}

void DCMotor_Controller::SetMaxVelocityThread(double v_rpm)
{
    QFuture<void> future = QtConcurrent::run(this, &DCMotor_Controller::SetVelocity, v_rpm);
    future.waitForFinished();
}

void DCMotor_Controller::RotateRelativeDistancce(int value)
{
    std::stringstream cmd;
    cmd << "LR" << value << "\n" << "M\r";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
    command_queue.push(command::OK);
    std::cout << "command_queue size = " << command_queue.size() << std::endl;
    command_queue.push(command::NP);
    std::cout << "command_queue size = " << command_queue.size() << std::endl;
}

void DCMotor_Controller::SetNotifyPosition(double _distance)
{

    //[encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:325]
    double distance = (4096*3.333*14)*(_distance/32.5)/(pi);

    std::cout << _distance << " " << distance << std::endl;
    std::stringstream  Position;  //使用字串串流
    char str[25];
    itoa((int)distance, str, 10);  //為了避免科學記號的產生

    Position <<"NP" << str <<"\n";

    std::cout << "position " << Position.str() << std::endl;
    ////////////////////////////////////////////////////
    // 使用DC motor要下的控制命令 V  (+:正轉  -:反轉)  --->命令下一次就可以維持轉動
    ///////////////////////////////////////////////////
    this->write(Position.str().c_str(), strlen(Position.str().c_str()));
}

void DCMotor_Controller::SetNPmode()
{
    std::stringstream cmd;
    cmd << "NP\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
    command_queue.push(command::OK);
    std::cout << "command_queue size = " << command_queue.size() << std::endl;
}

void DCMotor_Controller::SetVelocity(double speed)
{
    std::stringstream  velocity;  //使用字串串流
    velocity << "V" << speed << "\n";
    ////////////////////////////////////////////////////
    // 使用DC motor要下的控制命令 V  (+:正轉  -:反轉)  --->命令下一次就可以維持轉動
    ///////////////////////////////////////////////////
    this->write(velocity.str().c_str(), strlen(velocity.str().c_str()));
    command_queue.push(command::OK);
}

void DCMotor_Controller::Stop()
{
    std::cout << "Stop" << std::endl;
    std::stringstream cmd;
    cmd << "V0\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
    command_queue.push(command::OK);
}

void DCMotor_Controller::SetANSWmode(int mode)
{
    std::stringstream  command;
    command << "ANSW" << mode << "\n";
    this->write(command.str().c_str(), strlen(command.str().c_str()));
    command_queue.push(command::OK);
    std::cout << "command_queue size = " << command_queue.size() << std::endl;
}

void DCMotor_Controller::Disconnect()
{
    disconnect(this, SIGNAL(readyRead()), this, SLOT(Response()));
}

int DCMotor_Controller::GetPose()
{
    std::cout << "GetPose()" << std::endl;
    std::stringstream  cmd;
    cmd << "POS\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
//    command_queue.push(command::POS);
    while(this->waitForReadyRead(100))
    {
    }
    QByteArray temp = this->peek(this->bytesAvailable());
//    std::cout << "ready! " << temp.size() << std::endl;
//    for (int i = 0; i < temp.size(); i++)
//    {
//        std::cout << int(temp.at(i)) << " ";
//    }
//    std::cout << std::endl;
    // Debug message
    QByteArray test;
    while (temp.at(temp.size() - 1) == 10)
    {
        test = this->readLine();


//        emit ReturnPosition(str.toInt());

        temp = this->peek(this->bytesAvailable());
    }
    QString str(test);
    return str.toInt();


//    std::cout << "command_queue size = " << command_queue.size() << std::endl;
}

void DCMotor_Controller::Response()
{
    std::cout << "response" << std::endl;
    QByteArray temp = this->peek(this->bytesAvailable());
//    std::cout << "ready! " << temp.size() << std::endl;
    // Debug message
    while (temp.at(temp.size() - 1) == 10)
    {


        QByteArray test = this->readLine();
        QString str = QString(test);
        emit ReturnPosition(str.toInt());

        temp = this->peek(this->bytesAvailable());
    }


//    while (temp.at(temp.size() - 1) == 10)
//    {
////        std::cout << "temp size = " << temp.size() << std::endl;
////        for (int i = 0; i < temp.size(); i++)
////        {
////            std::cout << int(temp.at(i)) << " ";
////        }
////        std::cout << std::endl << "--------" << std::endl;

//        QByteArray test = this->readLine();

//        for (int i = 0; i < test.size(); i++)
//        {
//            std::cout << int(test.at(i)) << " ";
//        }
//        std::cout << std::endl;

//        switch (command_queue.front()) {
//        case command::OK:
//            std::cout << "command OK" << std::endl;
//            if (test.at(0) == 'O' && test.at(1) == 'K')
//            {
//                command_queue.pop();
//                std::cout << "size after pop = " << command_queue.size() << std::endl;
//            }
//            break;
//        case command::NP:
//            std::cout << "command NP" << std::endl;
//            if (test.at(0) == 'p')
//            {
//                command_queue.pop();
//                std::cout << "size after pop = " << command_queue.size() << std::endl;
//                emit PositionAttained();
//            }
//            break;
//        case command::POS:
//            std::cout << "POS return ----" << std::endl;
//            for (int i = 0; i < test.size(); i++)
//            {
//                std::cout << int(test.at(i)) << " ";
//            }
//            std::cout << std::endl;

//            // convert QByteArray to Int
//            QString str = QString(test);
////            std::cout << "To Int = " << str.toInt() << std::endl;
//            emit ReturnPosition(str.toInt());
//            command_queue.pop();
////            emit ReturnPosition();
//            break;
//        }
////        if (test.at(0) == 'p')
////        {
//////            this->write("POS\n");

////        }
//        temp = this->peek(this->bytesAvailable());
//    }

}

bool DCMotor_Controller::ReadData(QString &data, int num)
{
    QByteArray temp = this->read(num);

    data = QString(temp);

    return !temp.isEmpty();
}
