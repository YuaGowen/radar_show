#ifndef LIDARVIDEOVIEW_H
#define LIDARVIDEOVIEW_H

#include <QWidget>
#include <QButtonGroup>
#include "trnet.h"
#include "algorithm/algorithm.h"
#include "qtrnet.h"
#include "qcustomplot.h"

namespace Ui {
class LidarVideoView;
}

class LidarVideoView : public QWidget
{
    Q_OBJECT

public:
    explicit LidarVideoView(QWidget *parent = nullptr, QTRNet *lidarPkg = nullptr);
    ~LidarVideoView();

private:
    QCPItemTracer  *tracer;  //跟踪器
    Ui::LidarVideoView *ui;
    QButtonGroup *filterButtonGroup;
    Algorithm *algorithm;
    QTRNet *lidarPkg;

    uint8_t originImageData[1280 * 720];
    uint8_t tempImageData[1280 * 720];
    uint8_t imageData[1280 * 720];
    uint16_t imageWidth = 320;
    uint16_t imageHeight = 120;
 //   uint16_t imageHeight = 160;
    struct ImageInfo
    {
        double avg_value = 0.0;
        uint8_t max_value = 0;
    };

    uint8_t filterMode = 0;
    bool isShowRefline = false;
    bool isShowIndicate = false;
    uint16_t curveCols = 0;
    uint16_t indecateValue = 0;
    QString loadImagePath = NULL;
    bool playFlag = true;

    std::vector<uint8_t> refLine;
    std::vector<uint8_t> peakWidth;

    void uiInit();
    void connectInit();

    uint8_t getMiddleValue(uint8_t *start_addr, uint8_t *end_addr);
    void showImage(void);
    void showCurve(void);
    void showImageInfo(void);
private slots:
    void imageInfoPrecess(TRData *pack);
    void updateFilterMode(int mode);
    void switchReflineVisible(void);
    void switchIndicateVisible(void);
    void updateCurveCols(void);
    void updateIndicateValue(void);
    void savaImage(void);
    void loadImage(void);
    void showAndPlayImage(void);
    void stopShowImage(void);
    void mouseMoveEvent(QMouseEvent *event);  //鼠标移动事件
};

#endif // LIDARVIDEOVIEW_H
