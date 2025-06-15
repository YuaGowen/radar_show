#ifndef LIDARCONTROL_H
#define LIDARCONTROL_H

#include <QWidget>
#include <QButtonGroup>

#include "qtrnet.h"
#include "lidarmapview.h"

namespace Ui {
class LidarControl;
}
#define PACK_STRING_LEN 16

typedef enum
{
    /*0*/DISTANCE_MODE = 0,
    /*1*/CENTRIOD_MODE,
    /*2*/VIDEO_MODE,
    /*3*/SELF_TEST_MODE,
    /*4*/TRIG_DISTANCE_MODE,
} LidarMode;

class LidarControl : public QWidget
{
    Q_OBJECT

public:
    explicit LidarControl(QWidget *parent = nullptr, QTRNet *lidarPkg = nullptr, LidarMapView *lidarMapView = nullptr);
    ~LidarControl();

    int lidarMode = -1;

private:
    Ui::LidarControl *ui;

    uint8_t lidarNum = 1;
    struct LidarInfo
    {
        QString firmware_version;
        QString hardware_version;
        QString type;
        QString product_date;
        QString product_time;
        QString sn;
        QString pitch;
        QString roll;
        QString yaw;
        QString mcu_id;
        QString sn32;
        QString ssl_type;
    };
    LidarInfo lidarInfo;
    QList<QString> LIDARPRODUCT = {"ERROR", "SSL-20L", "SSL-20N", "LD07N", "SSL-20P", "SSL-20PL", "SSL-20LB2", "SSL-20NB2", "LD07BH", "SSL-20PT1", "SSL-20L2", "SSL-20N2","ERROR","ERROR",
                                   "ERROR","ERROR","ERROR","ERROR","ERROR","ERROR","ERROR","SSL-20L3","SSL-20N3BF","SSL-20L3F7"};
    bool isCoeReady = false;
    bool isLaserOpen = true;

    QTRNet *lidarPkg;
    LidarMapView *lidarMap;
    QButtonGroup *modeButtonGroup;
    QButtonGroup *numButtonGroup;

    void connectInit(void);
    void uiInit(void);
    void clearInfoTable(void);  //清空雷达信息表内容
signals:
    void lidarNumChanged(int num);

private slots:
    void updateMode(int mode);
    void updateNum(int num);
    void stopLidar(void);
    void restartLidar(void);
    void getLidarInfo(void);
    void setLidarType(void);

    void lidarInfoPrecess(TRData *pack);
    void updateCoeFlag(void);
    void switchLaser(void);
    void setLaserCurrent(void);
    void setRegister(void);
    void setAutoDutyThreshold(void);
    void getDutyThreshold(void);
    void setDutyThreshold(void);
    void getGroundDis(void);
    void setGroundDis(void);
    void getImageSize(void);
    void setImageSize(void);
    void getFilterGroupSize(void);
    void setFilterGroupSize(void);
    void setWorking(void);
    void setSleep(void);
	void getCalibrationTab(void);
    void getGroundDistance(void);
    void setGroundDistance(void);
    void setBrightness(void);
    void onlineCalibration(void);
    void disableOnlineCalibration(void);
    void enableOnlineCalibration(void);
    void on_sync_Button_clicked();
    void on_addr_Button_clicked();
};

#endif // LIDARCONTROL_H
