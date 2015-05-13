#include "laser_lms291_controller.h"

Laser_LMS291_Controller::Laser_LMS291_Controller() : QSerialPort()
  , dataheaderflag(false), Laser_count(0), onescandataflag(false)
  , segmentflag(false), contiscan_dataflag(false), headercount(0)
  , first_header_flag(false), conti_output(false)
{
    // connect reponse queue updating and haddling data thread
    connect(this, SIGNAL(readyRead()), this, SLOT(ReceiveDataThread()));
    // connect scan data to show scan data
//    connect(this, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)), this, SLOT(ProcessScanThread(boost::shared_ptr<QByteArray>)));
}

void Laser_LMS291_Controller::Open(const QString &comport, const int baudrate, const int prebaudrate)
{
    // the default setting of LMS291 after power down is Baud Rate 9600
    // but when the LMS291 was not power down, the user must set the
    // previous baud rate the connect the device
    // note: use 9600 to open 38400 is ok
    std::cout << "current: " << baudrate << " pre: " << prebaudrate << std::endl;

    // [parameter] examine  the input baudrates for LMS291
    switch (baudrate) {
    case QSerialPort::Baud9600:
        break;
    case QSerialPort::Baud19200:
        break;
    case QSerialPort::Baud38400:
        break;
    case QSerialPort::Baud500000:   // this is self-add enum in QSerialPort.h
        break;
    default:
        std::cerr << "Please input right baudrate for lms291!";
        return;
    }
    switch (prebaudrate) {
    case QSerialPort::Baud9600:
        break;
    case QSerialPort::Baud19200:
        break;
    case QSerialPort::Baud38400:
        break;
    case QSerialPort::Baud500000:   // this is self-add enum in QSerialPort.h
        break;
    default:
        std::cerr << "Please input right baudrate for lms291!";
        return;
    }

    // need to connect LMS291 first, in case of switch mode
    this->QSerialPort::setPortName(comport);
    this->QSerialPort::setDataBits(QSerialPort::Data8);
    this->QSerialPort::setParity(QSerialPort::NoParity);
    this->QSerialPort::setStopBits(QSerialPort::OneStop);
    this->QSerialPort::setFlowControl(QSerialPort::NoFlowControl);
    this->QSerialPort::setBaudRate(prebaudrate);
    this->QSerialPort::open(QSerialPort::ReadWrite);
    std::cout << this->QSerialPort::baudRate() << " " << baudrate << " " << prebaudrate << std::endl;

    // if baudrate changed, go to switch LMS291 I/O mode
    if (this->QSerialPort::isOpen() && (this->QSerialPort::baudRate() != baudrate))
    {
        std::cout << comport.toStdString() << std::endl;
        char LMS291cmd[8] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x42, 0x52, 0x08};
        switch (baudrate) {
        case QSerialPort::Baud9600:
            break;
        case QSerialPort::Baud19200:
            LMS291cmd[5] = 0x41;
            LMS291cmd[6] = 0x51;
            break;
        case QSerialPort::Baud38400:
            LMS291cmd[5] = 0x40;
            LMS291cmd[6] = 0x50;
            break;
        case QSerialPort::Baud500000:
            LMS291cmd[5] = 0x48;
            LMS291cmd[6] = 0x58;
            break;
        }
        this->QSerialPort::write(LMS291cmd, 8);
        // push command header to queue
        comm_queue.push(HeaderType::CommBaudHeader);
        this->QSerialPort::waitForReadyRead(1000);

        // reconnect the comport
        this->QSerialPort::close();
        this->QSerialPort::setPortName(comport);
        this->QSerialPort::setDataBits(QSerialPort::Data8);
        this->QSerialPort::setParity(QSerialPort::NoParity);
        this->QSerialPort::setStopBits(QSerialPort::OneStop);
        this->QSerialPort::setFlowControl(QSerialPort::NoFlowControl);


        if (this->QSerialPort::setBaudRate(baudrate))
            std::cout << std::dec << baudrate << " : setup succeed!" << std::endl;
        this->QSerialPort::open(QSerialPort::ReadWrite);
    }



}

Laser_LMS291_Controller::~Laser_LMS291_Controller()
{
    delete[] responseA0h_withACK;
    delete[] responseB0h_withdatalegnth;
    delete[] responseB0h_withdatalegnth_withACK;

    if (this->isOpen())
        this->close();
}

void Laser_LMS291_Controller::DisconnectReadyRead()
{
    disconnect(this, SIGNAL(readyRead()), this, SLOT(ReceiveDataThread()));
}

int Laser_LMS291_Controller::TriggerLaser()
{
    if (!this->QSerialPort::isOpen())
        return RS_DeviceNotOpened;
    //
    char LMS291cmd[8] = {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
    this->write(LMS291cmd, 8);

//    emit TriggerLaser();
    return RS_ok;
}

bool Laser_LMS291_Controller::ReceiveLaserData(std::vector<double> &scandata)
{
    scandata.clear();

    TriggerLaser();
    waitForReadyRead(1000);
    cv::waitKey(10000);
    uint datanum = this->bytesAvailable();
    std::cout << datanum << std::endl;
    if (datanum > 0 && datanum < 1024)
    {
        QByteArray datatemp = QSerialPort::read(datanum);

//        for (int i = 0; i < datatemp.size(); i++)
//        {
//            std::cout << std::hex << int(datatemp[i] & 0xff) << " ";
//        }
//        std::cout << std::endl;


        for (int i = 8; i < length_header + length_data*2; i+=2)
        {
            // strip for the 13th, 14th, 15th bits
            // because the resolution of LMS291 is 2^13 - 1 = 8191
            double r = (datatemp.at(i) & 0xff) | ((datatemp.at(i + 1) & 0x1f) << 8);
//            std::cout << std::dec << i << ": r = " << r << " " << std::hex << (datatemp.at(i) & 0xff) << " " << (datatemp.at(i + 1) & 0xff) << std::endl;
            scandata.push_back(r);
        }
        std::cout << std::dec << "scandata size = " << scandata.size() << std::endl;
        return true;

    }
    return false;
}

uint16_t Laser_LMS291_Controller::computeCRC(const QByteArray &data) const {

    size_t data_length = data.size();
    uint16_t uCrc16;
    uint8_t abData[2];
    uCrc16 = abData[0] = 0;
//    size_t data_i = 0;
    int data_i = 0;
    while (data_length-- ) {
        abData[1] = abData[0];
        abData[0] = data[data_i++];
//        data_i++;
        if(uCrc16 & 0x8000) {
            uCrc16 = (uCrc16 & 0x7fff) << 1;
            uCrc16 ^= CRC16_GEN_POL;
        }
        else {
            uCrc16 <<= 1;
        }
        uCrc16 ^= MKSHORT(abData[0],abData[1]);
    }
    return uCrc16;
}

bool Laser_LMS291_Controller::DataHeaderParsing(const uint8_t *dataheader, uint64_t data_len)
{
//    uint8_t buf[6];
//    if (this->peek(buf, sizeof(buf)) == sizeof(dataheader))
//        if (buf == dataheader)
//            return true;
    return false;

}

void Laser_LMS291_Controller::ReceiveContinuousData(const uint8_t *commheader, const uint8_t *dataheader, uint64_t data_len)
{
//    uint8_t tmp[];
//    char temp[8];
    this->TriggerContinuousMode();
    QByteArray temp;
    if (this->waitForBytesWritten(5000))
    {
        temp = this->read(1);
        while (this->waitForReadyRead(5000))
        {
            temp = this->read(1);
            std::cout << std::hex << int(temp[0]) << " ";
            //        for (int i = 0; i < 8; i++)
            //            std::cout << std::hex << int(temp[i]) << " ";
        }
    }
//    this->read(temp, 8);
//    for (int i = 0; i < 8; i++)
//        std::cout << std::hex << int(temp[i]) << std::endl;

}

void Laser_LMS291_Controller::testReceiveContinuousData()
{
//    uint8_t tmp[];
//    char temp[733];
    this->TriggerContinuousMode();
    if (this->waitForBytesWritten(5000))
    {
        QByteArray commheader = this->readAll();
        std::cout << commheader.size() << std::endl;
        while (this->waitForReadyRead(5000))
        {
            commheader.append(this->readAll());
            std::cout << commheader.size() << std::endl;
        }
        for (int i = 0; i < commheader.size(); i++)
            std::cout << std::hex << int(commheader[i] & 0xff) << " ";
        std::cout << std::endl;

//        QByteArray temp = this->read();
//        while (true)
//        {
//            temp.append(this->readAll());
//            for (int i = 0; i < temp.size(); i++)
//            {
//                qDebug() << temp[i];
////                std::cout << temp[i] << " ";
//            }
//            cv::waitKey(1000);
//        }
        QThread::sleep(1);
        std::cout << this->waitForReadyRead(5000) << std::endl;
        std::cout << "ba = " << this->bytesAvailable() << std::endl;
        QByteArray readData = this->readAll();
        std::cout << readData.size() << std::endl;
        while (this->waitForReadyRead(5000))
        {
            readData.append(this->readAll());
            std::cout << readData.size() << std::endl;
        }
        for (int i = 0; i < readData.size(); i++)
            std::cout << std::hex << int(readData[i] & 0xff) << " ";
        std::cout << std::endl;
    }



//    waitForReadyRead(1000);
//    QByteArray temp = this->read(733);
//    std::cout << temp.size() << std::endl;
//    while (temp.size() < 733)
//    {
//        temp += this->read(733);
//        std::cout << temp.size() << std::endl;
//    }

//    uint64_t b = this->bytesAvailable();
//    std::cout << "byte available = " << b << std::endl;
//    QByteArray temp = this->read(733);

//    for (int i = 0; i < temp.size(); i++)
//        std::cout << std::hex << int(temp[i] & 0xff) << " ";

//    this->read(temp, 8);
//    for (int i = 0; i < 8; i++)
//        std::cout << std::hex << int(temp[i]) << std::endl;

}

void Laser_LMS291_Controller::TriggerContinuousMode()
{
    if (!this->isOpen())
        return;
    char LMS291cmd[8] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08};
    this->write(LMS291cmd, 8);
    std::cout << "start continuous mode" << std::endl;
    // push continuous scan data header to queque
    comm_queue.push(HeaderType::CommContiDataHeader);
    std::cout << "comm_queue size = " << comm_queue.size() << std::endl;
}

void Laser_LMS291_Controller::TriggerOneScan()
{
    if (!this->isOpen())
        return;
    char LMS291cmd[8] = {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
    this->write(LMS291cmd, 8);
    // push one scan data header to queque
    comm_queue.push(HeaderType::CommOneScanDataHeader);
    std::cout << "comm_queue size = " << comm_queue.size() << std::endl;
    std::cout << "Trigger One Scan" << std::endl;
}

void Laser_LMS291_Controller::StopContinuousMode()
{
    if (!this->isOpen())
        return;
    char LMS291cmd[8] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x35, 0x08};
    this->write(LMS291cmd, 8);
    std::cout << "stop continuous mode" << std::endl;
    // push continuous scan data header to queque
    comm_queue.push(HeaderType::CommStopContiDataHeader);
    std::cout << "comm_queue size = " << comm_queue.size() << std::endl;
}

void Laser_LMS291_Controller::pushtobuffer()
{
//    if (!this->MySerialPort::isOpen())
//        return RS_DeviceNotOpened;


    std::cout << "ready!" << std::endl;
    QByteArray temp = this->readAll();
    buffer.append(temp);
    for (int i = 0; i < buffer.size(); i++)
        std::cout << std::hex << int(buffer[i] & 0xff) << std::dec << " ";
    std::cout << std::endl;

}

bool Laser_LMS291_Controller::CommHeaderCheck()
{
//    int N = 0;
//    const uint8_t *checkedheader;
//    switch (Headertype) {
//    case HeaderType::CommHeader:
//        std::cout << "baud comm" << std::endl;
//        checkedheader = baudcommheader;
//        N = sizeof(baudcommheader);
//        break;
//    case HeaderType::DataHeader:
//        checkedheader = dataheader;
//        N = sizeof(dataheader);
//        break;
//    default:
//        break;
//    }


//    std::cout << std::dec << "N = " << N << std::endl;


//    if (this->bytesAvailable() < sizeof(N))
//        return false;

////    char *temp;
////    this->peek(temp, N);
//    buffer = this->peek(N);
//    // Debug
//    for (int i = 0; i < sizeof(buffer); i++)
//        std::cout << std::hex << int(buffer[i] & 0xff) << std::dec << " ";
//    std::cout << std::endl;

//    std::cout << "buffer size = " << buffer.size() << std::endl;
//    if (buffer.size()!= N)
//        return false;
//    for (int i = 0; i < N; i++)
//    {
//        if (int(buffer[i] & 0xff) != checkedheader[i])
//            return false;
//    }
//    std::cout << "header validated!" << std::endl;
//    // progress the read pointer
//    this->read(N);
//    // enable the continuous data retrieve
//    if (Headertype == HeaderType::DataHeader)
//        dataheaderflag = true;
//    else
//        buffer.clear();
    return true;

}

void Laser_LMS291_Controller::CheckCommBaud()
{
    QFuture<bool> future = QtConcurrent::run(this, &Laser_LMS291_Controller::CommHeaderCheck);
    future.waitForFinished();
    if (future.result())
        emit headcorrect();
}

void Laser_LMS291_Controller::DataRetrieve()
{

    if (dataheaderflag)
    {

        buffer.append(this->readAll());



        if (buffer.size() == length_raw_data_complete)
        {
            uint16_t crc_right = ((buffer.at(length_raw_data_complete - 2) & 0xff) | ((buffer.at(length_raw_data_complete - 1) & 0xff) << 8));
            buffer.truncate(length_raw_data_complete - length_checksum);
            if (computeCRC(buffer) == crc_right)
            {
                std::cout << "time = " << testTime.elapsed() << std::endl;
                testTime.start();
                buffer = buffer.remove(length_raw_data_complete - length_checksum - length_status, length_status);
//                boost::shared_ptr<QByteArray> scan (new QByteArray(buffer.remove(0, length_header)));
//                emit GetOneLaserScan(scan);
//                LaserData.push_back(buffer.remove(0, length_header));
            }
//            for (int i = 0; i < buffer.size(); i++)
//            {
//                std::cout << std::hex << int(buffer[i] & 0xff) << " ";
//            }
//            std::cout << std::endl;
//            std::cout << std::hex << "crc right = " << crc_right
//                      << " computed = " << computeCRC(buffer)
//                      << " equal = " << (computeCRC(buffer) == crc_right)
//                      << std::endl;
//            std::cout << std::dec;

            //            dataheaderflag = false;
            buffer.clear();
        }
    }

//    if (this->bytesAvailable() < sizeof(N))
//        return false;
//    char *temp;
    //    this->peek(temp, N);
}



void Laser_LMS291_Controller::DataSegment()
{
    segmentflag = true;
    // reset parameter
    this->clear();
    buffer.clear();
    headercount = 0;
}

void Laser_LMS291_Controller::DisDataSegment()
{
    segmentflag = false;
}

//void LaserSerialController::getNextDataStream(size_t len_data)
//{
////    if (buffer.size())

//}

void Laser_LMS291_Controller::DataThread()
{
//    QtConcurrent::run(mylaserserial, &LaserSerialController::pushtobuffer);
    QFuture<void> future = QtConcurrent::run(this, &Laser_LMS291_Controller::pushtobuffer);
    future.waitForFinished();
}

void Laser_LMS291_Controller::ProcessScanThread(boost::shared_ptr<QByteArray> scan)
{
    QFuture<void> future = QtConcurrent::run(this, &Laser_LMS291_Controller::ProcessScan, scan);
    future.waitForFinished();
}

void Laser_LMS291_Controller::ProcessScan(boost::shared_ptr<QByteArray> scan)
{
    cv::Mat showimg = cv::Mat::zeros(1000, 1000, CV_8UC3);
    double angle = 0;
    for (int i = 0; i < scan->size(); i += 2)
    {
        // range information
        uint16_t r = (scan->at(i) & 0xff) | ((scan->at(i + 1) & 0x1f) << 8);
        // show point
        double x = r*cos(angle);
        double y = r*sin(angle);
        cv::circle(showimg, cv::Point2i(x*1.5 + 500, y*1.5 + 200), 1 ,cv::Scalar(0, 0, 255));
        angle += angle_resolution;
    }
    cv::imshow("Laser", showimg);
    cv::waitKey(1);
}

bool Laser_LMS291_Controller::GetOneScan()
{
//    disconnect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckOpenHeader()));
//    connect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckCommBaud()));
//    connect(myrobot->laser_lms291, SIGNAL(headcorrect()), this, SLOT(StartToGetData()));
//    this->TriggerContinuousMode();

//    disconnect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckCommBaud()));
//    connect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(DataSegment()));
//    connect(myrobot->laser_lms291, SIGNAL(GetOneLaserScan(QByteArray*)), myrobot->laser_lms291, SLOT(ProcessScanThread(QByteArray*)));
    return true;
}

bool Laser_LMS291_Controller::CheckHeader() {
//    // get header size
//    int N = 0;
//    const uint8_t *src;
//    switch (comm_queue.front())
//    {
//    case HeaderType::CommHeader:
//        N = sizeof(commheader);
//        src = commheader;
//        break;
//    case HeaderType::DataHeader:
//        N = sizeof(dataheader);
//        src = dataheader;
//        break;
//    case HeaderType::OneScanDataHeader:
//        N = sizeof(one_scan_dataheader);
//        src = one_scan_dataheader;
//        break;
//    }

//    // if input size is less than length of header, then return false
//    if (arr.size() < N)
//        return false;

//    // if not equal, return false
//    for (int offset = 0; offset <= (arr.size() - N); offset++)
//    {
//        int count = 0;
//        for (int i = 0; i < N; i++)
//        {
//            if ((0xff & src[i]) == (0xff & arr.at(i + offset)))
//                count++;
//        }
//        if (count == N)
//        {
//            comm_queue.pop();
//            return true;
//        }
//    }
    // no matching, return false
    return false;
}

bool Laser_LMS291_Controller::ReadData(std::vector<double> &scanData)
{
    bool state = false;
    scanData.clear();

    int num = this->bytesAvailable();

    if(num > 0 && num < 1024)
    {
        QByteArray temp = this->read(num);

        if( temp.size() == 733 ){

            for (int j = 8; j < temp.size() - 4; j += 2)
            {
                scanData.push_back(uint16_t((temp.at(j) & 0xff) | ((temp.at(j + 1) & 0x1f) << 8)));
            }

            state==true;

        }

    }

    return state;
}

void Laser_LMS291_Controller::GetOneLaserData(std::vector<double> &data)
{
    this->TriggerLaser();
    while(this->waitForReadyRead(400))
    {
    }
    int N = this->bytesAvailable();
    if (N == 733)
    {
        QByteArray temp = this->read(N);

        for (int j = 8; j < temp.size() - 4; j += 2)
        {
            data.push_back(uint16_t((temp.at(j) & 0xff) | ((temp.at(j + 1) & 0x1f) << 8)));
        }
    }

}

void Laser_LMS291_Controller::showReadyRead()
{
    QByteArray temp = this->peek(8);
    // Debug
    for (int i = 0; i < sizeof(temp); i++)
        std::cout << std::hex << int(temp[i] & 0xff) << std::dec << " ";
    std::cout << std::endl;
    std::cout << "ready!" << std::endl;
    this->flush();
}

void Laser_LMS291_Controller::CheckOpenHeader()
{
    std::cout << "ready!" << std::endl;

    QByteArray temp = this->peek(25);
    // Debug
    for (int i = 0; i < sizeof(temp); i++)
        std::cout << std::hex << int(temp[i] & 0xff) << std::dec << " ";
    std::cout << std::endl;
}

void Laser_LMS291_Controller::StartOneScan()
{
    std::cout << "position attained" << std::endl;
//    disconnect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckOpenHeader()));
//    connect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckCommBaud()));
//    connect(myrobot->laser_lms291, SIGNAL(headcorrect()), this, SLOT(StartToGetData()));
//    disconnect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckCommBaud()));
//    connect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(DataSegment()));
//    connect(myrobot->laser_lms291, SIGNAL(GetOneLaserScan(QByteArray*)), myrobot->laser_lms291, SLOT(ProcessScanThread(QByteArray*)));
    //    myrobot->laser_lms291->TriggerContinuousMode();
}

void Laser_LMS291_Controller::ReceiveDataThread()
{
    QFuture<bool> future = QtConcurrent::run(this, &Laser_LMS291_Controller::ReceiveData);
    future.waitForFinished();
//    if (future.result())
    //        emit headcorrect();
}

void Laser_LMS291_Controller::ShowScan(boost::shared_ptr<QByteArray> scan)
{
    std::cout << "Show" << std::endl;
    cv::Mat showimg = cv::Mat::zeros(1000, 1000, CV_8UC3);
    double angle = 0;
    for (int i = 0; i < scan->size(); i += 2)
    {
        // range information
        uint16_t r = (scan->at(i) & 0xff) | ((scan->at(i + 1) & 0x1f) << 8);
        // show point
        double x = r*cos(angle);
        double y = r*sin(angle);
        cv::circle(showimg, cv::Point2i(x*1.5 + 500, y*1.5 + 200), 1 ,cv::Scalar(0, 0, 255));
        angle += angle_resolution;
    }
    cv::imshow("Laser", showimg);
    cv::waitKey(1);
}

bool Laser_LMS291_Controller::ReceiveData()
{
    QByteArray temp = this->peek(this->bytesAvailable());
    for (int i = 0; i < temp.size(); i++)
    {
        std::cout << std::hex << (0xff & temp.at(i)) << " ";
    }
    std::cout << std::endl;

    if (!contiscan_dataflag && !onescandataflag && comm_queue.size() != 0)
    {
        // get header size
        int N = 0;
        const uint8_t *src;
        switch (comm_queue.front())
        {
        // command 20h
        // ACK + response A0h
        case HeaderType::CommBaudHeader:
        case HeaderType::CommContiDataHeader:
        case HeaderType::CommStopContiDataHeader:
            N = sizeof(responseA0h_withACK);
            src = responseA0h_withACK;
            break;
        // command 30h and continuous data header
        // ACK + reponse B0h
        case HeaderType::CommOneScanDataHeader:
            N = sizeof(responseB0h_withdatalegnth_withACK);
            src = responseB0h_withdatalegnth_withACK;
            break;
        // continuous data header (response 30h)
        case HeaderType::DataHeader:
            N = sizeof(responseB0h_withdatalegnth);
            src = responseB0h_withdatalegnth;
            break;
        }

        QByteArray temp = this->peek(this->bytesAvailable());
        // if input size is less than length of header, then return false
        if (temp.size() < N)
            return false;

//        std::cout << "test: " << Buffer.size() << std::endl;

        // search for the correct data header segment
        // if not equal, return false
        for (int offset = 0; offset <= (temp.size() - N); offset++)
        {
            int count = 0;
            for (int i = 0; i < N; i++)
            {
                if ((0xff & src[i]) == (0xff & temp.at(i + offset)))
                    count++;
            }
            if (count == N)
            {
                // debug message
                switch (comm_queue.front())
                {
                case HeaderType::CommBaudHeader:
                    std::cout << "CommBaudHeader succeed!" << std::endl;
                    // clear the buffer
                    this->read(N + offset);
                    break;
                case HeaderType::CommContiDataHeader:
                    std::cout << "CommContiDataHeader succeed!" << std::endl;
                    // clear the buffer
                    this->read(N + offset);
                    contiscan_dataflag = true;
                    break;
                case HeaderType::CommStopContiDataHeader:
                    std::cout << "CommStopContiDataHeader succeed!" << std::endl;
                    // clear the buffer
                    this->read(N + offset);
                    contiscan_dataflag = false;
                    break;
                case HeaderType::CommOneScanDataHeader:
                    std::cout << "CommOneScanDataHeader succeed!" << std::endl;
                    // truncate the data before
                    this->read(offset);
                    onescandataflag = true;
                    break;
                case HeaderType::DataHeader:
                    std::cout << "DataHeader succeed!" << std::endl;
                    comm_queue.push(HeaderType::DataHeader);
                    // truncate the data before
                    this->read(offset);
                    break;
                }
                comm_queue.pop();

//                if (N == sizeof(responseB0h))
//                {
//                    std::cout << "command 20h validate!" << std::endl;
//                    // truncate the data before
//                    this->read(offset);
//                    dataflag = true;
//                }
//                else
//                {
//                    std::cout << "command header validate!" << std::endl;
//                    // clear the buffer
//                    this->read(N + offset);
//                }
                return true;
            }
        }
        // no matching, return false
        return false;
    }
    else if (onescandataflag)
    {
        // dataflag: true, receive range data

        buffer.append(this->readAll());
//        std::cout << std::dec << "datasize: " << buffer.size() << std::endl;
//        for (int i = 0; i < buffer.size(); i++)
//        {
//            std::cout << std::hex << (0xff & buffer.at(i)) << " ";
//        }
//        std::cout << std::endl;

//        // check N - 3 is status: 10,
//        if (buffer.at((buffer.size() - 3)) == 10
//                && computeCRC(buffer.truncate(buffer.size() - length_checksum)) == ((buffer.at(buffer.size() - 2) & 0xff) | ((buffer.at(buffer.size() - 1) & 0xff) << 8)))
//        {
//            if (computeCRC(buffer.truncate(buffer.size() - length_checksum)) == ((buffer.at(buffer.size() - 2) & 0xff) | ((buffer.at(buffer.size() - 1) & 0xff) << 8)))
//            {

//            }
//            // truncate out length of checksum for computing transmitted data crc
//            buffer.truncate(length_onescan_raw_data - length_checksum);
//            // remove "Acknowledge" (ACK) flag, i.e. 0x06
//            buffer = buffer.remove(0, length_status);
//            // if no data missed
//            if (computeCRC(buffer) == crc_right)
//            {
//                std::cout << "crc right" << std::endl;
//                // attain the data segment
//                // remove the last checksum, status
//                buffer = buffer.remove((length_onescan_raw_data - 1) - length_checksum - length_status, length_status);
//                // remove length header - 1 (0x06)
//                boost::shared_ptr<QByteArray> scan (new QByteArray(buffer.remove(0, one_scan_length_header - 1)));
//                emit GetOneLaserScan(scan);
////                emit GetOneLaserScanP(new QByteArray(buffer.remove(0, one_scan_length_header - 1)));
//            }
//            // reset
//            buffer.clear();
//            dataflag = false;
//            return true;
//        }

        // append to buffer until length reach length_raw_data
        if (buffer.size() == length_onescan_raw_data)
        {
//            std::cout << std::dec << "datasize: " << buffer.size() << std::endl;
//            std::cout << std::dec << "datasize: " << buffer.size() << std::endl;
            // crc code transmitted
            uint16_t crc_right = ((buffer.at(length_onescan_raw_data - 2) & 0xff) | ((buffer.at(length_onescan_raw_data - 1) & 0xff) << 8));

            // truncate out length of checksum for computing transmitted data crc
            buffer.truncate(length_onescan_raw_data - length_checksum);
            // remove response first 0x06
            buffer = buffer.remove(0, length_status);
            // if no data missed
            if (computeCRC(buffer) == crc_right)
            {

                std::cout << "CRC ok" << std::endl;
                // attain the data segment
                // remove the last checksum, status
                buffer = buffer.remove((length_onescan_raw_data - 1) - length_checksum - length_status, length_status);
                // remove length header - 1 (0x06)

                boost::shared_ptr<QByteArray> scan (new QByteArray(buffer.remove(0, one_scan_length_header - 1)));
                emit GetOneLaserScan(scan);
//                emit GetOneLaserScanP(new QByteArray(buffer.remove(0, one_scan_length_header - 1)));
            }
            // reset
            buffer.clear();
            onescandataflag = false;
            return true;
        }
        return false;
    }
    else if (segmentflag && contiscan_dataflag)
    {
//        //---------------Test For Spin----------------
//        buffer.append(this->readAll());
//        // if input size is less than length of header, then return false
//        if (buffer.size() < length_header)
//            return false;
//        // search for the correct data header segment
//        // if not equal, return false
//        int offset;
//        int count;
//        for (offset = 0; offset <= (buffer.size() - length_header); offset++)
//        {
//            count = 0;
//            for (int i = 0; i < length_header; i++)
//            {
//                if ((0xff & responseB0h_withdatalegnth[i])
//                        == (0xff & buffer.at(i + offset)))
//                    count++;
//            }
//            if (count == length_header)
//            {

//                break;
//            }
//        }

//        if (count == length_header)
//        {
//            std::cout << "data header succeed!" << std::endl;
//            emit HeaderOK();

//            if (first_header_flag)
//            {

//                if (offset == length_raw_data_withoutheader)
//                {
//                    std::cout << "raw data length ok" << std::endl;
//                    for (int i = 0; i < length_header; i++)
//                        buffer.prepend(responseB0h_withdatalegnth[length_header - i - 1]);
//                    // get the crc transmitted
//                    uint16_t crc_checked = (buffer.at(length_raw_data - 2) & 0xff) | ((buffer.at(length_raw_data - 1) & 0xff) << 8);
//                    // truncate out other segment
//                    buffer = buffer.left(length_raw_data_withoutCRC);

//                    if ((computeCRC(buffer) == crc_checked))
//                    {
//                        // remove header, 7
//                        buffer = buffer.remove(0, length_header);
//                        // remove status, 1
//                        buffer.remove(length_data*2, length_status);
//                        boost::shared_ptr<QByteArray> scan (new QByteArray(buffer));
//                        emit GetContiOneScan(scan);
//                        // reset parameter
//                    }
//                }

//            }
//            else
//            {
//                first_header_flag = true;
//                buffer.remove(0, offset + length_header);
//            }
//        }




        //---------------------------

//        std::cout << "getin " << segmentflag << " " << contiscan_dataflag << std::endl;
        // read all data
        buffer.append(this->readAll());
        // if input size is less than length of header, then return false
        if (buffer.size() < length_header)
            return false;

        // search for the correct data header segment
        // if not equal, return false
        int offset;
        int count;
        for (offset = 0; offset <= (buffer.size() - length_header); offset++)
        {
            count = 0;
            for (int i = 0; i < length_header; i++)
            {
                if ((0xff & responseB0h_withdatalegnth[i])
                        == (0xff & buffer.at(i + offset)))
                    count++;
            }
            if (count == length_header)
                break;
        }

        if (count == length_header)
        {
            std::cout << "data header succeed!" << std::endl;
            headercount++;

            switch (headercount) {
            // first header: clear header
            case 1:
//                std::cout << "case 1 before:" << std::endl;
//                for (int i = 0; i < buffer.size(); i++)
//                {
//                    std::cout << std::hex << int(0xff & buffer.at(i)) << " ";
//                }
//                std::cout << std::dec << std::endl;

                buffer.remove(0, offset + length_header);
//                std::cout << "case 1 data:" << std::endl;
//                for (int i = 0; i < buffer.size(); i++)
//                {
//                    std::cout << std::hex << int(0xff & buffer.at(i)) << " ";
//                }
//                std::cout << std::dec << std::endl;
                break;
                //
            case 2:
//                std::cout << "case 2 offset = " << std::dec << offset << " buffer size = " << buffer.size()<< std::endl;
//                for (int i = 0; i < buffer.size(); i++)
//                {
//                    std::cout << std::hex << int(0xff & buffer.at(i)) << " ";
//                }
//                std::cout << std::dec << std::endl;

                if (offset == length_raw_data_withoutheader)
                {
                    std::cout << "raw data length ok" << std::endl;
                    for (int i = 0; i < length_header; i++)
                        buffer.prepend(responseB0h_withdatalegnth[length_header - i - 1]);
                    // get the crc transmitted
                    uint16_t crc_checked = (buffer.at(length_raw_data_complete - 2) & 0xff) | ((buffer.at(length_raw_data_complete - 1) & 0xff) << 8);
                    // truncate out other segment
                    buffer = buffer.left(length_raw_data_withoutCRC);

                    if ((computeCRC(buffer) == crc_checked))
                    {
                        // remove header, 7
                        buffer = buffer.remove(0, length_header);
                        // remove status, 1
                        buffer.remove(length_data*2, length_status);
                        boost::shared_ptr<QByteArray> scan (new QByteArray(buffer));
                        emit GetContiOneScan(scan);
                        // reset parameter
                        segmentflag = false;
                        buffer.clear();
                        headercount = 0;
                    }
                }
                else
                {
                    headercount = 1;
                    buffer.remove(0, offset + length_header);
                }
                break;
            }
        }

    }
    else if (conti_output && contiscan_dataflag)
    {
        //---------------Test For Spin----------------
        buffer.append(this->readAll());
        // if input size is less than length of header, then return false
        if (buffer.size() < length_header)
            return false;
        // search for the correct data header segment
        // if not equal, return false
        int offset;
        int count;
        for (offset = 0; offset <= (buffer.size() - length_header); offset++)
        {
            count = 0;
            for (int i = 0; i < length_header; i++)
            {
                if ((0xff & responseB0h_withdatalegnth[i])
                        == (0xff & buffer.at(i + offset)))
                    count++;
            }
            if (count == length_header)
            {

                break;
            }
        }

        if (count == length_header)
        {
            std::cout << "data header succeed! offset = " << std::dec << offset << std::endl;
//            for (int i = 0; i < buffer.size(); i++)
//            {
//                std::cout << std::hex << (0xff & buffer.at(i)) << " ";
//            }
//            std::cout << std::endl;

            emit HeaderOK();

            if (first_header_flag)
            {
                if (offset == length_raw_data_withoutheader)
                {
                    std::cout << "raw data length ok" << std::endl;
//                    for (int i = 0; i < length_header; i++)
//                        buffer.prepend(responseB0h_withdatalegnth[length_header - i - 1]);

                    // truncate out other segment
                    QByteArray temp;
                    for (int i = 0; i < length_header; i++)
                        temp.push_back(responseB0h_withdatalegnth[i]);
                    temp.push_back(buffer.left(length_raw_data_withoutheader));
//                    std::cout << "temp show" << std::endl;
//                    for (int i = 0; i < temp.size(); i++)
//                    {
//                        std::cout << std::hex << (0xff & temp.at(i)) << " ";
//                    }
//                    std::cout << std::endl;

                    buffer.remove(0, offset + length_header);
//                    std::cout << "Buffer Removed" << std::endl;
//                    for (int i = 0; i < buffer.size(); i++)
//                    {
//                        std::cout << std::hex << (0xff & buffer.at(i)) << " ";
//                    }
//                    std::cout << std::endl;

                    // get the crc transmitted
                    uint16_t crc_checked = (temp.at(length_raw_data_complete - 2) & 0xff) | ((temp.at(length_raw_data_complete - 1) & 0xff) << 8);

                    temp.remove(length_raw_data_withoutCRC, length_checksum);
//                    std::cout << "temp removed" << std::endl;
//                    for (int i = 0; i < temp.size(); i++)
//                    {
//                        std::cout << std::hex << (0xff & temp.at(i)) << " ";
//                    }
//                    std::cout << std::endl;

                    if ((computeCRC(temp) == crc_checked))
                    {
                        std::cout << "CRC OK!" << std::endl;
                        // remove header, 7
                        temp = temp.remove(0, length_header);
                        // remove status, 1
                        temp.truncate(length_raw_data);
//                        std::cout << "Data : " << std::endl;
//                        for (int i = 0; i < temp.size(); i++)
//                        {
//                            std::cout << std::hex << (0xff & temp.at(i)) << " ";
//                        }
//                        std::cout << std::endl;
                        boost::shared_ptr<QByteArray> scan (new QByteArray(temp));
                        emit GetContiOneScan(scan);
                        // reset parameter
                    }
                }

            }
            else
            {
                std::cout << "first_header_flag = true" << std::endl;
                first_header_flag = true;
                buffer.remove(0, offset + length_header);
            }
        }

    }






//    std::size_t N = this->bytesAvailable();
//    QByteArray abc = this->peek(N);
//    for (int i = 0; i < abc.size(); i++)
//        std::cout << std::hex << int(abc[i] & 0xff) << " ";
//    std::cout << std::endl;


//    if (!dataflag)
//    {

//        // dataflag: false, check header type
//        // quit when data lenth < minimun length of header
//        std::size_t N = this->bytesAvailable();
//        if (N < sizeof(dataheader))
//            return false;
//        // continuous data header
//        if (N == sizeof(dataheader))
//        {
//            std::cout << "data header" << std::endl;
//            QByteArray temp = this->peek(N);
//            for (int i = 0; i < temp.size(); i++)
//            {
//                std::cout << std::hex << (temp[i] & 0xff) << " ";
//            }
//            std::cout << std::endl;
//            if (CheckHeader(temp, HeaderType::DataHeader))
//            {
//                // read
//                // trigger the following data segmenting
//                dataflag = true;
//            }
//        }
//        // one scan data header
//        if (N == sizeof(one_scan_dataheader))
//        {
//            std::cout << "one scan data header" << std::endl;
//            QByteArray temp = this->peek(N);
//            for (int i = 0; i < temp.size(); i++)
//            {
//                std::cout << std::hex << (temp[i] & 0xff) << " ";
//            }
//            std::cout << std::endl;
//            if (CheckHeader(temp, HeaderType::OneScanDataHeader))
//            {
//                // read
//                // trigger the following data segmenting
//                dataflag = true;
//                buffer.append(this->readAll());
//                std::cout << "dataheader correct" << std::endl;
//            }
//        }
//        // command response header
//        if (N == sizeof(commheader))
//        {
//            QByteArray temp = this->peek(N);
//            if (CheckHeader(temp, HeaderType::CommHeader))
//            {
//                // refresh response queue
//                this->read(N);
//                // emit signal for command succeed
//                emit CommandSuccess();
//            }
//        }
//        return false;
//    }
//    else
//    {

//        // dataflag: true, receive range data
//        buffer.append(this->readAll());
//        // append to buffer until length reach length_raw_data
//        if (buffer.size() == length_onescan_raw_data)
//        {
//            std::cout << std::dec << "datasize: " << buffer.size() << std::endl;
//            // crc code transmitted
//            uint16_t crc_right = ((buffer.at(length_onescan_raw_data - 2) & 0xff) | ((buffer.at(length_onescan_raw_data - 1) & 0xff) << 8));

//            // truncate out length of checksum for computing transmitted data crc
//            buffer.truncate(length_onescan_raw_data - length_checksum);
//            // remove response first 0x06
//            buffer = buffer.remove(0, length_status);
//            // if no data missed
//            if (computeCRC(buffer) == crc_right)
//            {
//                std::cout << "crc right" << std::endl;
//                // attain the data segment
//                // remove the last checksum, status
//                buffer = buffer.remove((length_onescan_raw_data - 1) - length_checksum - length_status, length_status);
//                // remove length header - 1 (0x06)
//                boost::shared_ptr<QByteArray> scan (new QByteArray(buffer.remove(0, one_scan_length_header - 1)));
//                emit GetOneLaserScan(scan);
////                emit GetOneLaserScanP(new QByteArray(buffer.remove(0, one_scan_length_header - 1)));
//            }
//            // reset
//            buffer.clear();
//            dataflag = false;
//            return true;

//        }
//        return false;
//    }
}

