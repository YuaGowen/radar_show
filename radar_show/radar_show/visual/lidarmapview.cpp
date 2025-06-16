#include "lidarmapview.h"
#include "ui_lidarmapview.h"
#include <QPainter>
#include <QColorDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <qmath.h>
#include <QMouseEvent>
#include <QIntValidator>

LidarMapView::LidarMapView(QWidget *parent) : QWidget(parent)
    , ui(new Ui::LidarMapView)
    , renderData(size())
    , monitor(new QTimer(this))
    , refresMapTimer(new QTimer(this))
{
    ui->setupUi(this);

    connectInit();
    monitor->start(1000);
    refresMapTimer->start(33);
    frame_cnt = 0;
}

LidarMapView::~LidarMapView()
{
    delete ui;
}

void LidarMapView::connectInit()
{
    connect(refresMapTimer, SIGNAL(timeout()), this, SLOT(replot()));
}


void LidarMapView::paintEvent(QPaintEvent *)
{
    QPainter p(this);

    if(p.isActive() == false)
    {
        return;
    }
    p.setRenderHint(QPainter::Antialiasing,false);
    p.drawPixmap(0,0,renderData);
}

void LidarMapView::resizeEvent(QResizeEvent *event)
{
    renderData = QPixmap(event->size());
    replot();
}

void LidarMapView::replot()
{
    if(saveDataReadyFlag == true)
    {
        saveDataReadyFlag = false;
        //saveMapData(saveMap);
    }
    renderData.fill(backgroundColor);
    setAxes();
    setMap();
    update();
}

void LidarMapView::setAxes()
{
    QPainter p(&renderData);

    if(p.isActive() == false)
        return;

    p.rotate(-90);
    p.setRenderHint(QPainter::Antialiasing, true);

    //窗口的高度和宽度
    int h = geometry().height();
    int w = geometry().width();

    double circleDiameter = h / 2 - 10;
    double circleCenter = -circleDiameter / 2;

    p.setWindow(-w/2, -h/2, w, h);
    p.translate(spanOffset);
    p.scale(zoomScale, -zoomScale);

    QPen renderPen;
    renderPen.setWidthF((float)1 / zoomScale);    //放大倍数大 笔宽越细  放大倍数小 笔宽越大
    QColor inverse = QColor(255 - backgroundColor.red(), 255 - backgroundColor.green(), 255 - backgroundColor.blue(), 80);
    renderPen.setColor(inverse);
    p.setPen(renderPen);
    //绘制最外圈
    p.save();
    p.drawEllipse(circleCenter, circleCenter, circleDiameter, circleDiameter);
    p.restore();

    renderPen.setStyle(Qt::DotLine);
    p.setPen(renderPen);
    //绘制内圈和角度分割线
    p.save();
    for(int i = 1; i < 10; i++)
    {
        p.drawEllipse(QPoint(0,0), (int)(circleDiameter/20*i), (int)(circleDiameter/20*i));
    }

    for(int i = 0; i < 3; i++)
    {
        p.drawLine(-circleDiameter/2, 0, circleDiameter/2, 0);
        p.drawLine(0, -circleDiameter/2, 0, circleDiameter/2);
        p.rotate(30);
    }
    p.restore();

    QFont f("Times", 12, QFont::Thin);
    p.setFont(f);
    p.scale(1 / zoomScale, -1 / zoomScale);
    inverse.setAlpha(200);
    renderPen.setColor(inverse);
    p.setPen(renderPen);
    //绘制角度标识
    p.save();
    for(int i = 0; i < 12; i++)
    {
        if(i < 7)
        {
            p.drawText(QPoint(circleDiameter / 2 * zoomScale + 10, 0), QString("%1").arg(i * 30));
        }
        else
        {
            p.drawText(QPoint(circleDiameter / 2 * zoomScale + 10, 0), QString("%1").arg(-(12 - i) * 30));
        }
        p.rotate(30);
    }
    p.restore();
}

void LidarMapView::setLegned(QPainter &p)
{
    int w = geometry().width();

    QFont font("Times", 12, QFont::Thin);
    p.setFont(font);
    QPen renderPen(textColor);
    p.setPen(renderPen);

    const int shiftY = 30;
    const int shiftX = 250;
    p.drawText(QPointF(w - shiftX, shiftY), QString("%1: %2Hz").arg(QStringLiteral("帧率")).arg(updateRate, 0, 'f', 1));
    p.drawText(QPointF(w - shiftX, shiftY * 2), QString("%1: %2/160").arg(QStringLiteral("有效点")).arg(validPointNum));
//    p.drawText(QPointF(w - shiftX, shiftY * 3), QString("%1: %2").arg(QStringLiteral("最大值")).arg(maxDis));
//    p.drawText(QPointF(w - shiftX, shiftY * 4), QString("%1: %2").arg(QStringLiteral("最小值")).arg(minDis));
//    p.drawText(QPointF(w - shiftX, shiftY * 5), QString("%1: %2").arg(QStringLiteral("角分辨率")).arg(angleResolution));
//    p.drawText(QPointF(w - shiftX, shiftY * 6), QString("%1: %2").arg(QStringLiteral("±35°内最大值")).arg(distanceMax_in_70));
//    p.drawText(QPointF(w - shiftX, shiftY * 7), QString("%1: %2").arg(QStringLiteral("±35°内最小值")).arg(distanceMin_in_70));
//    p.drawText(QPointF(w - shiftX, shiftY * 8), QString("%1: %2").arg(QStringLiteral("±35°外最大值")).arg(distanceMax_out_70));
//    p.drawText(QPointF(w - shiftX, shiftY * 9), QString("%1: %2").arg(QStringLiteral("±35°外最小值")).arg(distanceMin_out_70));
//    p.drawText(QPointF(w - shiftX, shiftY * 10), QString("%1: %2").arg(QStringLiteral("平均值")).arg(average));
//    p.drawText(QPointF(w - shiftX, shiftY * 11), QString("%1: %2").arg(QStringLiteral("左视角")).arg(leftAngle));
//    p.drawText(QPointF(w - shiftX, shiftY * 12), QString("%1: %2").arg(QStringLiteral("右视角")).arg(rightAngle));
//    p.drawText(QPointF(w - shiftX, shiftY * 13), QString("%1: %2").arg(QStringLiteral("FOV")).arg(fov));
//    p.drawText(QPointF(w - shiftX, shiftY * 14), QString("%1: %2").arg(QStringLiteral("曝光等级")).arg(exposureLevel));
//    p.drawText(QPointF(w - shiftX, shiftY * 15), QString("%1: %2").arg(QStringLiteral("运行时间")).arg(runTime));
//    p.drawText(QPointF(w - shiftX, shiftY * 16), QString("%1: %2").arg(QStringLiteral("最大高度")).arg(max_high));
//    p.drawText(QPointF(w - shiftX, shiftY * 17), QString("%1: %2").arg(QStringLiteral("最小高度")).arg(min_high));
}

void LidarMapView::setMap()
{
    if(map.isEmpty())
    {
        return;
    }

    QList<MapData> mapTmp;

    QPainter p(&renderData);
    if(p.isActive() == false)
        return;

    int h = geometry().height();
    int w = geometry().width();

    maxDis = 0;
    minDis = 9999;
    angleResolution = 0;
    double last_angle = -99;
    double angle_threshold = 55.0;
    int max_threshold = 9999;

    max_high = 0;
    min_high = 65535;
    double high_temp = 0;
//    for(int i = 0; i < map.size(); i++)
//    {
//        int tmp = this->map[i].dis_source;

//        if((-angle_threshold) < this->map[i].angle && this->map[i].angle < angle_threshold)
//        {

//            high_temp = tan(abs(this->map[i].angle * 3.1415926/180.0)) * this->map[i].dis_source;
////            qDebug() << "this->map[i].angle" <<this->map[i].angle<< "this->map[i].dis_source" <<this->map[i].dis_source
////                        << "high_temp" <<high_temp;
//            if(high_temp > max_high)
//            {
//               max_high = high_temp;
//            }

//            if(high_temp < min_high && high_temp != 0)
//            {
//                min_high = high_temp;
//            }

//            if(last_angle != -99)
//            {
//                if(this->map[i].angle - last_angle > angleResolution)
//                {
// //                   qDebug() << "sn=" << i << this->map[i].centriod<<","<<angleResolution<< this->map[i].angle<<","<<last_angle;
//                    angleResolution = this->map[i].angle - last_angle;
////                    if(angleResolution > 1.0)
////                    {
////                         qDebug() << "sn=" << i-1 << this->map[i-1].centriod<<","<<angleResolution<< this->map[i-1].angle<<","<<last_angle;
////                         qDebug() << "sn=" << i << this->map[i].centriod<<","<<angleResolution<< this->map[i].angle<<","<<last_angle;
////                    }

//                 //   qDebug() << "sn=angleResolution" << angleResolution;
//                }
//                last_angle = this->map[i].angle;
//            }
//            else
//            {
//                last_angle = this->map[i].angle;
//            }

//            if(tmp > maxDis && tmp < max_threshold)
//            {
//                maxDis = tmp;
//            }
//            if(tmp < minDis && tmp > 5)
//            {
//                minDis = tmp;
//            }
//        }
//    }

    double circleDiameter = h / 2 - 10;
    setLegned(p);
    p.rotate(-90);
    p.setWindow(-w / 2, -h / 2, w, h);
    //平移缩放
    p.translate(spanOffset);
    p.scale(zoomScale, -zoomScale);

    //设置点云大小、连接样式、尾部样式
    QPen renderPen;
    renderPen.setWidthF(3 / zoomScale);
    renderPen.setJoinStyle(Qt::RoundJoin);
    renderPen.setCapStyle(Qt::RoundCap);
    p.setPen(renderPen);

    double angle;
    double dis;
    int    intensity;
    double indicatrixAngle = 0;
    double indicatrixDis = 0;
    int    indicatrixPoint = 0;
    bool   centriodMode = false;

    std::vector<double> x,y;
    std::vector<double> result;

//    map[0].angle = 5;
//    map[1].angle = 15;
//    map[2].angle = 25;
//    map[3].angle = 35;
    int valid_num = 0;
    for(int i = 0; i < map.size(); i++)
    {
        if(map[i].distance != 0)
        {
           // if(abs(map[i].angle) <= 60.0)
                valid_num++;
            angle = map[i].angle;
            if(map[i].distance > 2000)
            {
                centriodMode = true;
            }
            dis = map[i].distance * circleDiameter / 8 / 1000 / 2 * 1.9;
            QList<quint32> colorTable = {0xff100002, 0xff2b11a3, 0xff4c35d9, 0xff6251d9, 0xffa156bc, 0xffef4469, 0xffff7e3c, 0xffffbf28};
            intensity = (map[i].distance / 32);
            if(intensity > 7)
            {
                intensity = 7;
            }

            intensity = 8 - intensity;

            intensity = intensity % 8;

            renderPen.setColor(colorTable[intensity]);
            p.setPen(renderPen);

            double x=dis * qCos(M_PI / 180 * angle);
            double y=-dis * qSin(M_PI / 180 * angle);
            double radius = 0.5;
            // 计算圆的边界矩形
            QRectF circleRect(x- radius, y- radius, 2 * radius, 2 * radius);
            p.drawEllipse(circleRect); // 绘制圆

            //p.drawPoint(QPointF(dis * qCos(M_PI / 180 * angle), -dis * qSin(M_PI / 180 * angle)));
            mapTmp.push_back(map[i]);
        }
    }

    validPointNum = valid_num;

    if(isSetZeroAngle)
    {
        clickAngle = 0;
    }

    if(mapTmp.size() != 0)
    {
        indicatrixPoint = getAnglePos(clickAngle, mapTmp);
        indicatrixAngle = mapTmp[indicatrixPoint].angle;

        indicatrixDis = mapTmp[indicatrixPoint].distance * circleDiameter / 8 / 1000 / 2 * 1.9;
        QPointF indicatrixEndPoint = QPointF(indicatrixDis * qCos(indicatrixAngle * M_PI / 180), -indicatrixDis * qSin(indicatrixAngle * M_PI / 180));
   //        // 打印参数值
//        qDebug() << "indicatrixDis:" << indicatrixDis;
//        qDebug() << "indicatrixAngle:" << indicatrixAngle;
//        qDebug() << "indicatrixEndPoint:" << indicatrixEndPoint.x() << "," << indicatrixEndPoint.y();

        //绘制连接线
        p.setRenderHint(QPainter::Antialiasing, true);
        renderPen.setStyle(Qt::DotLine);
        QColor inverse = QColor(255 - backgroundColor.red(), 255 - backgroundColor.green(), 255 - backgroundColor.blue(),255);
        renderPen.setColor(inverse);
        p.setPen(renderPen);
        p.drawLine(QPoint(0,0), indicatrixEndPoint);

        //绘制角度 距离 置信度
        p.scale(1 / zoomScale, -1 / zoomScale);
        QFont f("Times", 15, QFont::Thin);
        p.setFont(f);
        renderPen.setColor(textColor);
        p.setPen(renderPen);
        p.rotate(90);

        indicatrixEndPoint = QPointF(-indicatrixEndPoint.y(), -indicatrixEndPoint.x());
        if(isPointTextEnable)
        {
         //   p.drawText(indicatrixEndPoint * zoomScale, QString("%1(%2),%3%4,%5(%6)mm,%7").arg(mapTmp[indicatrixPoint].index).arg(mapTmp[indicatrixPoint].real_index).arg(indicatrixAngle,0,'f',1).arg(QStringLiteral("°")).arg(mapTmp[indicatrixPoint].dis_source).arg(mapTmp[indicatrixPoint].distance).arg(mapTmp[indicatrixPoint].intensity));
            p.drawText(indicatrixEndPoint * zoomScale, QString("%1,%2c  m,%3%4,%5").arg(mapTmp[indicatrixPoint].index).arg(mapTmp[indicatrixPoint].distance).arg(indicatrixAngle,0,'f',1).arg(QStringLiteral("°")).arg(mapTmp[indicatrixPoint].centriod));

        }
    }
    //    if(mapTmp.size() != 0)
    //    {
    //        indicatrixPoint = getAnglePos(clickAngle, mapTmp);
    //        indicatrixAngle = mapTmp[indicatrixPoint].angle;

    //        QPointF indicatrixEndPoint = QPointF(indicatrixDis * qCos(indicatrixAngle * M_PI / 180), -indicatrixDis * qSin(indicatrixAngle * M_PI / 180));

    //        //绘制连接线
    //        p.setRenderHint(QPainter::Antialiasing, true);
    //        renderPen.setStyle(Qt::DotLine);
    //        QColor inverse = QColor(255 - backgroundColor.red(), 255 - backgroundColor.green(), 255 - backgroundColor.blue(),255);
    //        renderPen.setColor(inverse);
    //        p.setPen(renderPen);
    //        p.drawLine(QPoint(0,0), indicatrixEndPoint);

    //        //绘制角度 距离 置信度
    //        p.scale(1 / zoomScale, -1 / zoomScale);
    //        QFont f("Times", 15, QFont::Thin);
    //        p.setFont(f);
    //        renderPen.setColor(textColor);
    //        p.setPen(renderPen);
    //        p.rotate(90);

    //        indicatrixEndPoint = QPointF(-indicatrixEndPoint.y(), -indicatrixEndPoint.x());
    //        if(isPointTextEnable)
    //        {
    //         //   p.drawText(indicatrixEndPoint * zoomScale, QString("%1(%2),%3%4,%5(%6)mm,%7").arg(mapTmp[indicatrixPoint].index).arg(mapTmp[indicatrixPoint].real_index).arg(indicatrixAngle,0,'f',1).arg(QStringLiteral("°")).arg(mapTmp[indicatrixPoint].dis_source).arg(mapTmp[indicatrixPoint].distance).arg(mapTmp[indicatrixPoint].intensity));
    //        }
    //    }
}

double onlineOffsetFunction(double x, double coe, double power, int distance)
{
    return x-distance - coe * pow((x - 30), power);
}

double onlineOffsetFunctionDer(double x, double coe, double power)
{
    return 1- power * coe * pow((x - 30), power - 1);
}

void LidarMapView::updateMap(const QList<MapData> &map)
{
    this->map = map;


    if(frame_cnt >= 30)
    {
        frame_cnt = 0;
        if(refreshTime.isValid())
        {
            updateRate = 1000.0 / refreshTime.elapsed() * 30;
            refreshTime.restart();
        }
        else
        {
            refreshTime.start();
        }
    }
    else
    {
        frame_cnt++;
    }


//    //进行校正
//    if(enableMapOffset)
//    {
//        if(!mapCoe.empty())
//        {
//            if(errorPower > 1)
//            {
//                for(int i = 0; i < mapCoe.size(); i++)
//                {
//                    if(this->map[i].distance != 0)
//                    {
//                        double offset_x0 = mapOffsetRealDis;//牛顿迭代法的初始值，定为mapOffsetRealDis
//                        double offset_x1 = 0;
//                        double tmp = this->map[i].distance * cos(this->map[i].angle * 3.1415926 / 180);
//                        for(int k = 0; k < 2; k++)//牛顿迭代2次
//                        {
//                            offset_x1 = offset_x0 - onlineOffsetFunction(offset_x0, mapCoe[i], errorPower, tmp)
//                                        /onlineOffsetFunctionDer(offset_x0, mapCoe[i], errorPower);
//                            offset_x0 = offset_x1;
//                        }
//                        this->map[i].distance = offset_x1 / cos(this->map[i].angle * 3.1415926 / 180.0);
//                    }
//                }
//            }
//            else if(errorPower == 1)
//            {
//                for(int i = 0; i < mapCoe.size(); i++)
//                {
//                    if(this->map[i].distance != 0)
//                    {
//                        double tmp = this->map[i].distance * cos(this->map[i].angle * 3.1415926 / 180);
//                        tmp = (tmp - mapCoe[i] * 30)/(1 - mapCoe[i]);
//                        this->map[i].distance = tmp / cos(this->map[i].angle * 3.1415926 / 180.0);
//                    }
//                }
//            }
//        }
//    }

//    if(startSaveFlag == true)
//    {
//        if(saveFrameNumTemp > 0)
//        {
//            saveMap.append(map);
//            saveFrameNumTemp--;
//            QString num = QString("%1").arg(saveFrameNumTemp);

//        }
//        else
//        {
//            startSaveFlag = false;

//            saveDataReadyFlag = true;
//        }
//    }
}

void LidarMapView::setZeroAngle()
{
    isSetZeroAngle = true;
    isPointTextEnable = true;
}

void LidarMapView::delPointText()
{
    isPointTextEnable = false;
}

void LidarMapView::setPointColor()
{
    QColor color = QColorDialog::getColor(Qt::white, this, "select color");
    pointColor = color;
}

void LidarMapView::setTextColor()
{
    QColor color = QColorDialog::getColor(Qt::white, this, "select color");
    textColor = color;
}

void LidarMapView::setBackgroundColor()
{
    QColor color = QColorDialog::getColor(Qt::white, this, "select color");
    backgroundColor = color;
}

void LidarMapView::frameNumChanged(QString num)
{
    saveFrameNum = num.toInt();
}

void LidarMapView::saveLidarData()
{
    if(saveFrameNum > 0 && saveFrameNum < 1000)
    {

        saveFrameNumTemp = saveFrameNum;
        startSaveFlag = true;
        saveMap.clear();
    }
}


void LidarMapView::updataPara(int max_in_70, int max_out_70, int min_in_70, int min_out_70, double angle_resolution, double avr, double left_angle, double right_angle, int exposure, int run_time)
{
    distanceMax_in_70 = max_in_70;
    distanceMax_out_70 = max_out_70;
    distanceMin_in_70 = min_in_70;
    distanceMin_out_70 = min_out_70;
    angularResolution = angle_resolution;
    average = avr;
    leftAngle = left_angle;
    rightAngle = right_angle;
    fov =rightAngle-leftAngle;
    timestamp = QDateTime::currentDateTime();
    exposureLevel = exposure;
    runTime = run_time;
}

//获取鼠标选中角度的点云index
uint16_t LidarMapView::getAnglePos(float angle, QList<MapData> &mapTmp)
{
    uint16_t i =0;
    std::pair<float, int> angle_pair;
    std::map<float, int> angle_map;

    //将角度和点云index配对
    for(auto m : mapTmp)
    {
        angle_pair = std::make_pair(m.angle, i++);
        angle_map.insert(angle_pair);
    }

    //策略
    //1、鼠标点击角度无点云时，左半边选择第一个点，右半边选择最后一个点
    //2、鼠标点击角度有点云时，选择最近的后一个点；
    float lastAngle = 0;
    uint16_t lastPos = 0;
    for(auto data : angle_map)
    {
        if(data.first > angle)
        {
            if(data.first - angle > 2)
            {
                return lastPos;
            }
            else
            {
                return data.second;
            }
        }
        lastAngle = data.first;
        lastPos = data.second;
    }

    if(angle - lastAngle < 2)
    {
        return lastPos;
    }
    else
    {
        return mapTmp.size() - 1;
    }
}

//滚轮拨动事件
void LidarMapView::wheelEvent(QWheelEvent *event)
{
    //滚轮拨动一格 event->delta()为120
    //新版qt已废除event->delta()
    double wheelSteps = event->angleDelta().y() / 120.0;

    if(wheelSteps == 1)
    {
        zoomScale = zoomScale * 1.25;
    }
    if((wheelSteps == -1) && (zoomScale > 0.1))
    {
        zoomScale = zoomScale * 0.8;
    }
}

//鼠标点击事件
void LidarMapView:: mousePressEvent(QMouseEvent *event)
{
    int shiftX = this->size().width() / 2;
    int shiftY = this->size().height() / 2;

    int mouseX = event->pos().x();
    int mouseY = event->pos().y();

    int x = mouseX - shiftX;
    int y = shiftY - mouseY;

    x = x - spanOffset.y();
    y = y - spanOffset.x();

    //鼠标左键按下，更新角度
    //鼠标右键按下，记录移动初始位置
    if(event->button() == Qt::LeftButton)
    {
        isPointTextEnable = true;
        isSetZeroAngle = false;
        mousePos = QPoint(x,y);
        if(y != 0)
        {
            qreal v = (qreal)x / (qreal)y;
            clickAngle = qAtan(v) * 180 / M_PI;

            if(y < 0)
            {
                if(x > 0)
                {
                    clickAngle = clickAngle + 180;
                }
                else if(x < 0)
                {
                    clickAngle = clickAngle - 180;
                }
                else if(x == 0)
                {
                    clickAngle = 180;
                }
            }
        }
        else
        {
            clickAngle = (x < 0) ? -90 : 90;
        }
    }
    else
    {
        spanOffsetLast = spanOffset;

        /* 记录拖拽起始点 */
        spanStartPoint = QPoint(-event->pos().y() + this->size().height() / 2, event->pos().x() - this->size().width() / 2);
        setMouseTracking(true);
        moveFlag = true;
    }
}

//鼠标移动事件
void LidarMapView::mouseMoveEvent(QMouseEvent *event)
{
    if(event->buttons() & Qt::RightButton)//必须要用到buttons();
    {
        if(moveFlag)
        {
            spanEndPoint = QPoint(-event->pos().y() + this->size().height() / 2, event->pos().x() - this->size().width() / 2);
            spanOffset += spanEndPoint - spanStartPoint;
            spanStartPoint = spanEndPoint;
            replot();
        }
    }
}
