#ifndef LASERSERIALCONTROLLER_H
#define LASERSERIALCONTROLLER_H
#include <QSerialPort>
#include "RS_ret.h"
#include "boost/array.hpp"      // for boost::array
#include <boost/cstdint.hpp>    // for boost::uint16_t
#include <cstddef>              // for std::size_t
#include <fstream>              // for std::ofstream
#include "boost/shared_ptr.hpp" // for boost::shared_ptr
#include <queue>                // for std::queue
#include <QDebug>
#include <opencv2/opencv.hpp>
#include <QtConcurrent>
#include "opencv2/opencv.hpp"
#include "QTime"


#define CRC16_GEN_POL 0x8005
#define MKSHORT(a,b) ((unsigned short) (a) | ((unsigned short)(b) << 8))

const std::size_t length_data = 361;
const std::size_t length_status = 1;
const std::size_t length_checksum = 2;
const std::size_t length_header = 7; // 0x02 0x80 0xD6 0x02 0xB0 0x61 0x01
const std::size_t one_scan_length_header = 8; // 0x06 0x02 0x80 0xD6 0x02 0xB0 0x69 0x01

//const int length_length_raw_data
const int length_raw_data = length_data*2;
const int length_raw_data_withoutheader = length_data*2 + length_status + length_checksum;
const int length_raw_data_complete = length_header + length_data*2 + length_status + length_checksum;
const int length_raw_data_withoutCRC = length_header + length_data*2 + length_status;
const int length_onescan_raw_data = one_scan_length_header + length_data*2 + length_status + length_checksum;
// data: 8(header) + 361*2 + 1(status) + 2(checksum) = 733

const double angle_resolution = 0.5*3.1415926535897932384626433832795/180;

// if command correct, LMS291 will response ACK: 0x06 (acknowledgement)
// else command error, LMS291 will only response 0x15

// data header list

// command 20h: switch operating modes (start continuous & stop continuous)
// monitoring mode: (one of modes (20h))
// data: 24h: continuous data
//       25h: stop continuous data
//       40h: setting to 38400  Bd
//       41h: setting to 19200  Bd
//       42h: setting to 9600   Bd
//       48h: setting to 500000 Bd

// length of 3, A0h response (response to 20h)
const uint8_t responseA0h_withACK[10] =
    {0x06, 0x02, 0x80, 0x03, 0x00, 0xA0, 0x00, 0x10, 0x16, 0x0A};

// command 30h: request individual scan

// data length of 726, B0h response (reponse to 30h) ** continuous data header is this, too
// data: 726 = 0x0169 (361) [1st data] + 361*2 [2nd-Nth data]
const uint8_t responseB0h_withdatalegnth[7] = {0x02, 0x80, 0xD6, 0x02, 0xB0, 0x69, 0x01};
const uint8_t responseB0h_withdatalegnth_withACK[8] = {0x06, 0x02, 0x80, 0xD6, 0x02, 0xB0, 0x69, 0x01};

const size_t Laser_count_expect = 1200;

enum HeaderType
{
    CommBaudHeader = 0,
    CommOneScanDataHeader = 1,
    CommContiDataHeader = 2,
    CommStopContiDataHeader = 3,
    DataHeader = 4,
};

enum LMSBaudRateType
{
    Baud9600 = 9600,
    Baud19200 = 19200,
    Baud38400 = 38400,
    Baud500000 = 500000
};

class Laser_LMS291_Controller : public QSerialPort
{

    Q_OBJECT

public:
    Laser_LMS291_Controller();

    ~Laser_LMS291_Controller();

    void DisconnectReadyRead();

    //    void ReceiveData();
    void Open(const QString &comport, const int baudrate, const int prebaudrate = 9600);

    uint16_t computeCRC(const QByteArray &data) const;

    bool DataHeaderParsing(const uint8_t *dataheader, uint64_t data_len);

    void ReceiveContinuousData(const uint8_t *commheader, const uint8_t *dataheader, uint64_t data_len);

    int TriggerLaser();

    void testReceiveContinuousData();

    bool ReceiveLaserData(std::vector<double> &scandata);

    void ProcessScan(boost::shared_ptr<QByteArray> scan);
    // get one scan data, return status
    bool GetOneScan();

    std::ofstream ofile;

    static bool CheckHeader();

    bool ReadData(std::vector<double> &scanData);

signals:

    void headcorrect();

    void GetOneLaserScan(boost::shared_ptr<QByteArray> scan);

    void CommandSuccess();

    void SuccessTrigger();

    // 2015.01.30
    void GetContiOneScan(boost::shared_ptr<QByteArray> scan);

    // 2015.03.18
    void HeaderOK();

public slots:



    void pushtobuffer();

    void DataThread();

    void CheckCommBaud();



    void TriggerOneScan();



    void ProcessScanThread(boost::shared_ptr<QByteArray> scan);

    void showReadyRead();

    void CheckOpenHeader();

    void StartOneScan();

    // 2015.01.26
    void TriggerContinuousMode();

    void StopContinuousMode();

    void DataSegment();

    void DisDataSegment();

    void SetContiOutput(bool temp)
    {
        conti_output = temp;
    }

private slots:

    // recieve data thread
    void ReceiveDataThread();

    void ShowScan(boost::shared_ptr<QByteArray> scan);




private:

    // data buffer aimed to store
    // 7(header) + 361*2(data) + 1(status) + 2(checksum) + 1(0x06)
    QByteArray buffer;

    // receive data function
    bool ReceiveData();

    bool CommHeaderCheck();


//    std::vector<double> LaserData;
    std::vector<QByteArray> LaserData;

    size_t Laser_count;

    void DataRetrieve();

    bool dataheaderflag;

    QTime testTime;

    bool onescandataflag;

    bool contiscan_dataflag;

    bool conti_output;

    // 2015.01.30
    bool segmentflag;

    int headercount;

    // command queue for response
    std::queue<HeaderType> comm_queue;

    // 2015/03/18
    bool first_header_flag;

};

#endif // LASERSERIALCONTROLLER_H
