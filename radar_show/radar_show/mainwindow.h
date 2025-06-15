#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include <QPushButton>
#include <QSerialPort>
#include <QLabel>
#include <QVBoxLayout>
#include "lidarmapview.h"
#include "qtrnet.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; class RadarWidget;}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    const QString SETTINGS_NAME = "minishow_pro.ini";
    const QString TITLE = QStringLiteral("LDROBOT_线激光调试上位机_v0.0.1");

  //  QList<QString> LIDARPRODUCT = {"SSL-20L", "SSL-20N", "SSL-20P", "SSL-20PT1", "SSL-20PL", "LD07", "LD07N", "SSL-20LB2", "SSL-20NB2", "SSL-20L2", "SSL-20N2","SSL-20L3","SSL-20L3F7"};
    QList<QString> LIDARPRODUCT = {"RADAR-1"};
    QString lidarType = "SSL-20L";
    QString  baudRate = "921600";


    QComboBox *lidarSelectBox;
    QComboBox *comBox;
    QComboBox * baudRateComboBox;
    QPushButton *portSwitchButton;
    QPushButton *jigPageButton;
    QVBoxLayout *vLayout;
    QHBoxLayout *hLayout;
    QVBoxLayout *lidarMapLayout;

    QSerialPort *serialPort;
    QTimer *refreshTimer;
    QLabel *status;

    QTRNet *lidarPkg;
    LidarMapView *lidarmapview1;
//    LidarMapView *lidarmapview2;
//    LidarMapView *lidarmapview3;

//    void recoverCustom(void);
//    void saveCustom(void);
     void statusBarInit(void);
    void toolBarInit(void);
     void connectInit(void);
      void layoutInit(void);
//    void setShortcut(void);

    void openSerialPort(void);
    void closeSerialPort(void);

private slots:
    void refreshCom();
    void switchSeiralPort();
    void lidarTypeChanged(QString lidar);
    void handleError(QSerialPort::SerialPortError error);

    void readLidarData();
//    void updateLidarMapUi(int num);
//    void updateUiLayout(void);

 //   void openJigPage(void);

};


#include <QWidget>
#include <QPainter>
#include <QPointF>
#include <QPolygonF>
#include <cmath>

#include <QWidget>
#include <QPainter>
#include <QPointF>
#include <QPolygonF>
#include <cmath>
#include <QVector>
#include <QString>

#define PI 3.1415926

class RadarWidget : public QWidget {
public:
    explicit RadarWidget(QWidget *parent = nullptr) : QWidget(parent) {}

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // 设置背景颜色为黑色
        painter.fillRect(rect(), Qt::black);

        // 绘制雷达图背景（多边形网格）
        int sides = 6;  // 边数（对应数据维度）
        QPointF center(width() / 2, height() / 2);
        qreal radius = qMin(width(), height()) / 2 * 0.8;

        // 绘制坐标轴
        painter.setPen(Qt::gray);
        for (int i = 1; i <= sides; ++i) {
           painter.drawEllipse(center, radius * i / 5, radius * i / 5);
       }

        // 绘制角度刻度
        painter.setPen(Qt::white);
        for (int i = -sides; i <= sides; ++i) {  // -90 度到 90 度，共13个刻度
           qreal angle = (180.0 / sides) * i * PI / 180;
           qreal x = center.x() + radius * cos(angle);
           qreal y = center.y() + radius * sin(angle);
           painter.drawLine(center, QPointF(x, y));

           // 添加角度刻度
           QString angleText = QString::number((180.0 / sides) * i) + "°";
           qreal textX = center.x() + (radius + 10) * cos(angle);
           qreal textY = center.y() - (radius + 10) * sin(angle);  // 调整文本位置
           painter.drawText(QPointF(textX, textY), angleText);
       }

        // 只绘制雷达图的上半部分
        painter.setPen(Qt::blue);
        painter.setBrush(QColor(0, 0, 255, 50));
        QVector<QPointF> points;
        QVector<qreal> values = {1, 3, 2, 4};  // 示例数据
        for (int i = 0; i < sides / 2; ++i) {  // 只绘制上半部分
           qreal angle = (180.0 / sides) * i * PI / 180;
           qreal x = center.x() + radius * values[i] / 5 * cos(angle);
           qreal y = center.y() - radius * values[i] / 5 * sin(angle);  // 调整y坐标为负
           points.append(QPointF(x, y));
       }
        points.append(center);  // 添加中心点以闭合多边形
        painter.drawPolygon(QPolygonF(points));
    }
};

//#include <QPainter>
//#include <QColorDialog>

//#define  PI   3.1415926
//class RadarWidget : public QWidget {
//public:
//    explicit RadarWidget(QWidget *parent = nullptr) : QWidget(parent) {}

//protected:
//    void paintEvent(QPaintEvent *) override {
//        QPainter painter(this);
//        painter.setRenderHint(QPainter::Antialiasing);

//        // 绘制雷达图背景（多边形网格）
//        int sides = 6;  // 边数（对应数据维度）
//        QPointF center(width()/2, height()/2);
//        qreal radius = qMin(width(), height())/2 * 0.8;

//        // 绘制坐标轴
//        painter.setPen(Qt::gray);
//        for(int i=1; i<=sides; ++i){
//            painter.drawEllipse(center, radius*i/5, radius*i/5);
//        }

////        // 绘制数据多边形（示例数据）
////         QVector<QPointF> points;
////        QVector<qreal> values = {1, 3, 2, 4};  // 对应4个维度的值
////        for(int i=0; i<sides; ++i){
////            qreal angle = (360.0/sides)*i * PI/180;
////            qreal x = center.x() + radius * values[i]/5 * cos(angle);
////            qreal y = center.y() + radius * values[i]/5 * sin(angle);
////            points.append(QPointF(x, y));
////        }

//        // 绘制数据连线
////        painter.setPen(Qt::blue);
////        painter.setBrush(QColor(0,0,255,50));
////        painter.drawPolygon(QPolygonF(points));
//    }
//};
#endif // MAINWINDOW_H
