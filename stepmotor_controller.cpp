#include "stepmotor_controller.h"

StepMotor_Controller::StepMotor_Controller() : QSerialPort(),
    mode(Mode::SpeedMode), stop_flag(false)
{
    // connect stepmotor quary to response singals
    connect(this, SIGNAL(readyRead()), this, SLOT(Response()));
}

StepMotor_Controller::~StepMotor_Controller()
{
    if (this->isOpen())
        this->close();
}

void StepMotor_Controller::Open(const QString &comport, const int baudrate)
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

void StepMotor_Controller::RotateAbsoluteAngle(const double abs_ang)
{
    this->write("NP\n");
    // convert angle in degree to command angle
    const double comm_ang = abs_ang * 60.0 / 360.0 * 3000.0;
    // using std::stringstream save command text
    std::stringstream command_abs_ang;
    command_abs_ang << "LA" << comm_ang << "\n"; // "LA" absolute angle
    this->write(command_abs_ang.str().c_str(), std::strlen(command_abs_ang.str().c_str()));
    command_abs_ang.clear();
    command_abs_ang << "M\r";
    this->write(command_abs_ang.str().c_str(), std::strlen(command_abs_ang.str().c_str()));
//    this->MySerialPort::write()
//            absoluteAngle.clear();
//            absoluteAngle<<"M\r";
//            port->write(absoluteAngle.str().c_str(),strlen(absoluteAngle.str().c_str()));

//            std::stringstream  absoluteAngle;  //�ϥΦr�����y
//    absoluteAngle << "LA" <<angle<<"\n";
//    port->write(absoluteAngle.str().c_str(),strlen(absoluteAngle.str().c_str()));
//    absoluteAngle.clear();
//    absoluteAngle<<"M\r";
//    port->write(absoluteAngle.str().c_str(),strlen(absoluteAngle.str().c_str()));
}

void StepMotor_Controller::RotateRelativeAngle(const double rel_ang)
{
    // degree to command angle
    const double comm_ang = rel_ang * 60.0 / 360.0 * 3000.0;
    std::stringstream command_rel_ang;
    // spin speed in rpm
//    command_rel_ang << "SP" << 15000 << "\n";
//    this->write(command_rel_ang.str().c_str(), std::strlen(command_rel_ang.str().c_str()));
//    command_rel_ang.clear();
//    // deacceration
//    command_rel_ang << "DEC" << 10 << "\n";
//    this->write(command_rel_ang.str().c_str(), std::strlen(command_rel_ang.str().c_str()));
//    command_rel_ang.clear();

    // relative rotation in

    this->write("NP\n");

    command_rel_ang << "LR" << comm_ang << "\n"; // "LA" absolute angle
    this->write(command_rel_ang.str().c_str(), std::strlen(command_rel_ang.str().c_str()));
    command_rel_ang.clear();
    command_rel_ang << "M\r";
    this->write(command_rel_ang.str().c_str(), std::strlen(command_rel_ang.str().c_str()));

    //    stringstream  relatedAngle;  //�ϥΦr�����y
    //    relatedAngle<<"LR"<<angle<<"\n";
    //    port->write(relatedAngle.str().c_str(),strlen(relatedAngle.str().c_str()));
    //    relatedAngle.clear();
    //    relatedAngle<<"M\r";
    //    port->write(relatedAngle.str().c_str(),strlen(relatedAngle.str().c_str()));
}

void StepMotor_Controller::SetHome()
{
    const char* temp;
    if(mode == Mode::AngleMode){   //step motor have to enable to invoke the function
        temp = "EN\n";
        this->write(temp, strlen(temp));
    }
    temp = "HO\n";
    this->write(temp, strlen(temp));
}

void StepMotor_Controller::OpenANSWMode()
{
    if (this->isOpen())
    {
        this->write("ANSW2\n");
    }
}

void StepMotor_Controller::SetMaxVelocity(double v_rpm)
{
    std::stringstream cmd;
    cmd << "SP" << v_rpm << "\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
}

void StepMotor_Controller::rotatethread()
{
    QFuture<void> future = QtConcurrent::run(this, &StepMotor_Controller::RotateRelativeAngle, 360);
//    future.waitForFinished();
}

void StepMotor_Controller::Response()
{

    QByteArray temp = this->peek(this->bytesAvailable());
//    std::cout << "ready! " << temp.size() << std::endl;
    // Debug message
//    for (int i = 0; i < temp.size(); i++)
//        std::cout << "Step: " << std::dec << int(temp[i]);
//    if (this->bytesAvailable())
//        std::cout << std::endl;

//    std::cout << temp.size() << std::endl;

//    std::cout << std::endl;

    while (temp.at(temp.size() - 1) == 10)
    {

//        std::cout << temp.size() << std::endl;
//        for (int i = 0; i < temp.size(); i++)
//        {
//            std::cout << int(temp.at(i)) << " ";
//        }
//        std::cout << std::endl;

        QByteArray test = this->readLine();

//        for (int i = 0; i < test.size(); i++)
//        {
//            std::cout << int(test.at(i)) << " ";
//        }
//        std::cout << std::endl;

        if (test.at(0) == 'p')
        {
            this->write("POS\n");
            emit PositionAttained();
        }
        temp = this->peek(this->bytesAvailable());
    }

//    response.append(this->read(temp.size()));
//    std::cout << "response size = " << response.size() << std::endl;
//    if (response.size() == 1)
//    {
//        std::cout << " test1 = " << char(response.at(0)) << std::endl;
//        if (response.at(0) == 'p')
//        {
//            emit PositionAttained();
//        }
//    }
//    else if (response.size() == 2
//             && response.at(0) == 'O' && response.at(1) == 'K')
//    {
//        response.clear();
//    }
//    else {
//        response.clear();
//    }
}

void StepMotor_Controller::GoHome()
{
    const char* temp;
    temp = "LA0\n";
    this->write(temp, strlen(temp));
    temp = "M\n";
    this->write(temp, strlen(temp));
}

void StepMotor_Controller::Stop()
{
    std::stringstream  velocity;  //�ϥΦr�����y
    velocity << "V0\n";
    this->write(velocity.str().c_str(), strlen(velocity.str().c_str()));
}


