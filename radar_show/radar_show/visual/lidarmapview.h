#ifndef LIDARMAPVIEW_H
#define LIDARMAPVIEW_H

#include <QWidget>
#include <QTime>
#include <QPushButton>
#include <QLineEdit>
#include "qtrnet.h"
#include "trnet.h"

namespace Ui {
class LidarMapView;
}


//#pragma pack(push) //保存对齐状态
//#pragma pack(1)    //设定为1字节对齐
//struct MapData
//{
//    double  angle;
//    uint16_t distance;
//    uint16_t intensity;
//    uint16_t index;
//    uint16_t real_index;
//    double centriod;
//    uint16_t dis_source;
//};
//#pragma pack(pop)  //恢复对齐状态

class LidarMapView : public QWidget
{
    Q_OBJECT

public:
    explicit LidarMapView(QWidget *parent = nullptr);
    ~LidarMapView();


    QList<MapData> map;

    void setPkg(QTRNet *pkg){this->pkg = pkg;}
    void isUseMapOffset(bool enable_map_offset){enableMapOffset = enable_map_offset;}
    void setMapOffset(QVector<double> map_coefficient, int map_offset_dis, int error_power){mapCoe = map_coefficient;
                                                                                            mapOffsetRealDis = map_offset_dis;
                                                                                            errorPower = error_power;}
private:
    Ui::LidarMapView *ui;

public slots:
    void replot();

protected:
    void paintEvent(QPaintEvent *);
    void resizeEvent(QResizeEvent *event);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);

public:
    QDateTime timestamp;
    double updateRate;
    double distanceMax_in_70;
    double distanceMax_out_70;
    double distanceMin_in_70;
    double distanceMin_out_70;
    double angularResolution = 0;
    double average = 0;
    double leftAngle = 0;
    double rightAngle = 0;
    double fov = 0;
    double angleResolution = 0;
    int exposureLevel;
    int runTime;

    double max_high;
    double min_high;

    int maxDis = 0;
    int minDis = 9999;

private:
    QPushButton *clearTextPushButton;
    QPushButton *zeroAnglePushButton;
    QLineEdit *saveFrameNumLineEdit;
    QPushButton *saveDataPushButton;
    bool isPointTextEnable;
    bool isSetZeroAngle;
    double zoomScale = 1.900;
    QPixmap renderData;

    QPoint spanStartPoint = QPoint(0,0);
    QPoint spanEndPoint = QPoint(0,0);
    QPoint spanOffset;          /* 拖拽偏差量 */
    QPoint spanOffsetLast;      /* 上次的拖拽偏差量 */
    qreal clickAngle = 0;
    QPoint mousePos=QPoint(0,0);
    bool moveFlag = false;

    //QColor backgroundColor=QColor(0x40,0x4A,0x59,0xff);
    QColor backgroundColor = QColor(235,235,235,255);
    QColor textColor=QColor(0x0,0xff,0x0,0xff);
    QColor pointColor=QColor(0xff,0xff,0xff,0xff);

    QString savedFilePath;
    uint8_t frame_cnt;
    QTime refreshTime;
    QTimer *monitor;
    QTRNet *pkg;
    QTimer *refresMapTimer;
    int validPointNum;
    int saveFrameNum = 1;
    int saveFrameNumTemp = 1;
    bool saveDataReadyFlag = false;
    QVector<double> mapCoe;//计算出来的在线标定系数
    int mapOffsetRealDis = 0;//在线标定的距离
    int errorPower = 0;//误差的指数
    bool enableMapOffset = false;

    QList<MapData> saveMap;
    bool startSaveFlag = false;

    void connectInit();
    void setAxes();
    void setMap();
    void setLegned(QPainter &p);
    uint16_t getAnglePos(float angle,QList<MapData> &mapTmp);

private slots:
    void updateMap(const QList<MapData> &map);
    void setPointColor();
    void setTextColor();
    void setBackgroundColor();

    void delPointText();
    void setZeroAngle();
    void frameNumChanged(QString num);
    void saveLidarData();
    void updataPara(int max_in_70, int max_out_70, int min_in_70, int min_out_70, double angle_resolution, double avr, double left_angle, double right_angle, int exposure, int run_time);
};

#endif // LIDARMAPVIEW_H
