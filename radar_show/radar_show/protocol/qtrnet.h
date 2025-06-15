#ifndef QTRNET_H
#define QTRNET_H
#include <QtSerialPort/QSerialPort>
#include <QObject>
#include <stdint.h>
#include "trnet.h"
#include "QQueue"
#include <QDebug>

typedef enum
{
    TYPE_NULL           = 0x00,

    TYPE_SSL_20N3 	    = 0x14,
    TYPE_SSL_20L3    	= 0x15,


    TYPE_SSL_TOTAL
} SSLTypeDef;

typedef enum
{
    /*0x00*/PACK_NULL = 0,
    /*0x01*/PACK_GET_VIDEO,
    /*0x02*/PACK_GET_DISTANCE,
    /*0x03*/PACK_GET_CENTRIOD,
    /*0x04*/PACK_AUTO_LASER_DUTY,
    /*0x05*/PACK_LASER_TOGGLE,
    /*0x06*/PACK_LASER_CURRENT,
    /*0x07*/PACK_DOWNLOAD_LUT_START,
    /*0x08*/PACK_DOWNLOAD_LUT_TRANSMIT,
    /*0x09*/PACK_DOWNLOAD_LUT_COMPLETE,
    /*0x0A*/PACK_UPGRADE_START,
    /*0x0B*/PACK_UPGRADE_TRANSMIT_DATA,
    /*0x0C*/PACK_UPGRADE_COMPLETE,
    /*0x0D*/PACK_RESET_SYSTEM,
    /*0x0E*/PACK_ENABLE_DEBUG,           //可使用
    /*0x0F*/PACK_STOP,
    /*0x10*/PACK_ACK,
    /*0x11*/PACK_CONFIG_REGISTER,
    /*0x12*/PACK_GET_COE,
    /*0x13*/PACK_SET_COE,
    /*0x14*/PACK_VERSION,
    /*0x15*/PACK_VIDEO_SIZE,
    /*0x16*/PACK_CONFIG_ADDRESS,
    /*0x17*/PACK_DISTANCE_RANGE,
    /*0x18*/PACK_SET_DATA,
    /*0x19*/PACK_SET_VIDEO,
    /*0x1A*/PACK_SET_Dynamic_Gain_Adjustment,
    /*0x1B*/PACK_SELF_TEST,
    /*0x1C*/PACK_SET_FILTER_GROUP_SIZE,
    /*0x1D*/PACK_GET_FILTER_GROUP_SIZE,
    /*0x1E*/PACK_TRANSMISSION_MODE,
    /*0x1F*/PACK_SCREENSHOT,
    /*0x20*/PACK_SET_SN,
    /*0x21*/PACK_GET_SN,
    /*0x22*/PACK_SET_ANGLE,
    /*0x23*/PACK_GET_ANGLE,
    /*0x24*/PACK_GET_TAB,    //可使用
    /*0x25*/PACK_GET_SPI_VIDEO,
    /*0x26*/PACK_GET_DISTANCE_TRIG_MODE,
    /*0x27*/PACK_SET_FARTHEST_DISTANCE,
    /*0x28*/PACK_SET_GROUND_DISTANCE,
    /*0x29*/PACK_GET_GROUND_DISTANCE,
    /*0x2A*/PACK_SYNC,
    /*0x2B*/PACK_SET_LASER_DUTY,
    /*0x2C*/PACK_GET_LASER_DUTY,
    /*0x2D*/PACK_SET_ENABLE_EX_TRIGGER,
    /*0x2E*/PACK_SET_DISENABLE_EX_TRIGGER,
    /*0x2F*/PACK_AUTO_ADJUST_BRIGHTNESS,
    /*0x30*/PACK_SENSOR_ERROR,			   //sensor错误----工作模式下长时间没有收到帧同步信号

    /*0x31*/PACK_SET_SN_NEW,
    /*0x32*/PACK_GET_SN_NEW,
    /*0x33*/PACK_GET_CLOS_DISTRIBUTION,
    /*0x34*/PACK_SET_SSL_TYPE,

    /*0xFE*/PACK_LOG = 0xFE,
} PackageIDTypeDef;

#define SENSOR_ANGLE_FROM_CAMERA_LENS       (80)
#define SENSOR_ANGLE_HALF                   (SENSOR_ANGLE_FROM_CAMERA_LENS/2)

#pragma pack(push) //保存对齐状态
#pragma pack(1)    //设定为1字节对齐
struct MapData
{
    double  angle;
    double distance;
    uint16_t intensity;
    uint16_t index;
    uint16_t real_index;
    double centriod;
    uint16_t dis_source;
};
#pragma pack(pop)  //恢复对齐状态

class QTRNet : public QObject
{
    Q_OBJECT

public:
    int ackResult = -1;
    uint16_t data[35] = {0};

    explicit QTRNet(QSerialPort *serialPort);
    bool sendCmd(uint8_t address, uint8_t id, uint32_t config);
    bool sendWords(uint8_t address, uint8_t id, uint32_t *data, uint32_t len, uint16_t offset);
    bool sendBytes(uint8_t address, uint8_t id, uint8_t *data, uint32_t len, uint16_t offset);
    void parsePkg(const QByteArray &data);

    QString lidarType = "SSL-20L";

    void waitWork(int ms);
    bool waitAck(int timeout , int type);
    QList<MapData> getMap() const;

private:
    std::vector<uint8_t> lidarDataQueue;
    TRData mRecPackage;
    QList<MapData> mRecMap;

    int32_t coe_20[3][42];
    uint32_t coe_07[3][4];
    const double pi = 3.141592653;
    uint8_t last_centriod = 0;
    uint8_t last_index = 0;
    double last_anger= 0;
    int ackType = -1;

    double measureDistance[2000] = {0};
    double measureAngle[2000]    = {0};
    double realDistance[2000]    = {0};
    double realAngle[2000]       = {0};

    QSerialPort *serialPort;
    bool isWaiting = false;

    void parseMapData(TRData *pack);
    void angleTransform(uint16_t dist, uint8_t centriod, int n, double *dstTheta, double *dstDist, uint8_t address);

    void underwaterCorrect(uint16_t measureDis, double measureAng, uint16_t *realDis, double *realAng);

signals:
    void coeReady(void);
    void lidarDataReady(TRData *pack);
    void mapReady1(const QList<MapData> &map);
    void mapReady2(const QList<MapData> &map);
    void mapReady3(const QList<MapData> &map);

    void paraReady(int max_in_70, int max_out_70, int min_70, int min_out_70, double angle_resolution, double average, double left_angle, double right_angle, int exposure, int run_time);
};

#endif // QTRNET_H
