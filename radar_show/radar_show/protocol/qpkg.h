#ifndef QPKG_H
#define QPKG_H
#include <QSerialPort>
#include <QObject>
#include <stdint.h>
#include "pro_v2.h"
#include "QQueue"

class QPkg : public QObject
{
    Q_OBJECT

public:
    explicit QPkg(QSerialPort *serialPort);
    bool sendCmd(uint8_t id, int16_t config);
    bool sendCmd(uint8_t id, uint32_t config);
    bool sendData(uint8_t id , uint8_t *data , uint32_t len);    
    void parsePkg(const QByteArray &data);
    void waitWork();
    void waitWork(int ms);

private:
    QSerialPort *serialPort;
    bool isWaiting = false;
    uint8_t txBuffer[128];
    uint8_t mLidarRawData[8192];
    QQueue<uint8_t> dataQueue;
    uint32_t mLidarRawDataCounter = 0;
    PackageDataStruct mRecPackage;
    uint8_t mAckType;


private:

signals:
    void motorDataReady(PackageDataStruct *mRecPackage);
};

#endif // QPKG_H
