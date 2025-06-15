#include "qpkg.h"
#include "QTime"
#include "QCoreApplication"
#include "QDebug"

QPkg::QPkg(QSerialPort *serialPort): QObject(serialPort),
    mAckType(0xff)
{
    if(serialPort == NULL)
        throw "SerialPort is NULL";

    this->serialPort = serialPort;
}

//16位
bool QPkg::sendCmd(uint8_t id, int16_t config)
{
    if(isWaiting == true)
    {
        return false;
    }

    if(!serialPort->isOpen())
    {
        return false;
    }

    PackageDataStruct package;
    uint32_t len;

    package.data_id = id;
    package.data_in_buff = (uint8_t *)&config;
    package.data_in_len = sizeof(config);
    package.data_out_buff = txBuffer;
    package.data_out_len = &len;
    Package(package);

    serialPort->write((char *)txBuffer,len);
    waitWork();
    return true;
}

//32位
bool QPkg::sendCmd(uint8_t id, uint32_t config)
{
    if(isWaiting == true)
    {
        return false;
    }

    if(!serialPort->isOpen())
    {
        return false;
    }

    PackageDataStruct package;
    uint32_t len;

    package.data_id = id;
    package.data_in_buff = (uint8_t *)&config;
    package.data_in_len = sizeof(config);
    package.data_out_buff = txBuffer;
    package.data_out_len = &len;
    Package(package);

    serialPort->write((char *)txBuffer,len);
    waitWork();
    return true;
}

//不定长
bool QPkg::sendData(uint8_t id, uint8_t *data, uint32_t len)
{
    if(isWaiting == true)
    {
        return false;
    }

    if(!serialPort->isOpen())
    {
        return false;
    }

    PackageDataStruct package;
    package.data_id = id;
    package.data_in_buff = data;
    package.data_in_len = len;
    package.data_out_buff = txBuffer;
    package.data_out_len = &len;
    Package(package);

    serialPort->write((char *)txBuffer,len);
    waitWork();
    return true;
}

void QPkg::parsePkg(const QByteArray &data)
{
    foreach(char c,data)
    {
        dataQueue<<(uint8_t)c;
    }

    uint32_t outLen = 0;

    int size = dataQueue.size();

    for(int i=0;i<size;i++)
    {
        mLidarRawData[mLidarRawDataCounter++] = dataQueue.dequeue();
        mRecPackage.data_in_buff = mLidarRawData;
        mRecPackage.data_in_len = mLidarRawDataCounter;
        mRecPackage.data_out_len = &outLen;
        // qDebug() << "mLidarRawDataCounter:" << mLidarRawDataCounter;
        if(Unpacking(&mRecPackage) == true)
        {
            //在这里处理任务分发
            //qDebug() << "dataQueue:" << dataQueue.length();
            mLidarRawDataCounter = 0;
            emit motorDataReady(&mRecPackage);
            break;
        }

        if(mLidarRawDataCounter >= sizeof(mLidarRawData) || dataQueue.length() > 128)
        {
            dataQueue.clear();
            mLidarRawDataCounter = 0;
            break;
        }
    }
}

void QPkg::waitWork(void)
{
    if(isWaiting == true)
        return;

    QTime t;
    t.start();
    isWaiting = true;
    while(t.elapsed()<50)
        QCoreApplication::processEvents();
    isWaiting = false;
}

void QPkg::waitWork(int ms)
{
    if(isWaiting == true)
        return;

    QTime t;
    t.start();
    isWaiting = true;
    while(t.elapsed()<ms)
        QCoreApplication::processEvents();
    isWaiting = false;
}


