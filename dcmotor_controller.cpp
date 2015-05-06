#include "dcmotor_controller.h"

DCMotor_Controller::DCMotor_Controller()
{
}

DCMotor_Controller::~DCMotor_Controller()
{

}


void DCMotor_Controller::sethome()
{
    std::stringstream cmd;
    cmd << "H0\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
}

void DCMotor_Controller::gohome()
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
}

void DCMotor_Controller::RotateRelativeDistancce(int value)
{
    std::stringstream cmd;
    cmd << "LR" << value << "\n" << "M\r";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
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

void DCMotor_Controller::SetVelocity(double speed)
{
    std::stringstream  velocity;  //使用字串串流
    velocity << "V" << speed << "\n";
    ////////////////////////////////////////////////////
    // 使用DC motor要下的控制命令 V  (+:正轉  -:反轉)  --->命令下一次就可以維持轉動
    ///////////////////////////////////////////////////
    this->write(velocity.str().c_str(), strlen(velocity.str().c_str()));
}

void DCMotor_Controller::Stop()
{
    std::stringstream cmd;
    cmd << "V0\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
}

void DCMotor_Controller::SetANSWmode(int mode)
{
    std::stringstream  command;
    command << "ANSW" << mode << "\n";
    this->write(command.str().c_str(), strlen(command.str().c_str()));
}

void DCMotor_Controller::GetPose()
{
    std::stringstream  cmd;
    cmd << "POS\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
}

bool DCMotor_Controller::ReadData(QString &data, int num)
{
    QByteArray temp = this->read(num);

    data = QString(temp);

    return !temp.isEmpty();
}
