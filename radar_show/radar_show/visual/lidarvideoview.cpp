#include "lidarvideoview.h"
#include "ui_lidarvideoview.h"

#include "qtrnet.h"
#include <QImage>

#pragma execution_character_set("utf-8")

LidarVideoView::LidarVideoView(QWidget *parent, QTRNet *lidarPkg) :
    QWidget(parent),
    ui(new Ui::LidarVideoView),
    algorithm(new Algorithm(this))
{
    ui->setupUi(this);
    this->lidarPkg = lidarPkg;

    uiInit();
    connectInit();
}

LidarVideoView::~LidarVideoView()
{
    delete ui;
}

void LidarVideoView::uiInit()
{
    ui->pixelCurveWidget->legend->setVisible(false);
    ui->pixelCurveWidget->xAxis->setRange(0, 240);
    ui->pixelCurveWidget->yAxis->setRange(0, 260);

    ui->pixelCurveWidget->addGraph();  //实际图像显示波形
    ui->pixelCurveWidget->graph(0)->setPen(QPen((Qt::blue)));
    ui->pixelCurveWidget->graph(0)->setAntialiased(false);//不抗锯齿
    ui->pixelCurveWidget->addGraph();  //参考线
    ui->pixelCurveWidget->graph(1)->setPen(QPen((Qt::red)));
    ui->pixelCurveWidget->graph(1)->setAntialiased(false);
    ui->pixelCurveWidget->addGraph();  //指示线
    ui->pixelCurveWidget->graph(2)->setPen(QPen((Qt::green)));
    ui->pixelCurveWidget->graph(2)->setAntialiased(false);
    ui->pixelCurveWidget->addGraph();  //原始图像显示波形
    ui->pixelCurveWidget->graph(3)->setPen(QPen((Qt::gray)));
    ui->pixelCurveWidget->graph(3)->setAntialiased(false);
    //允许用户用鼠标拖动轴的范围，用鼠标滚轮缩放，选择坐标轴和点击选择图形:
    ui->pixelCurveWidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    //初始化曲线跟踪器并设置描点参数
    tracer = new QCPItemTracer(ui->pixelCurveWidget);
    tracer->setInterpolating(false);
    tracer->setStyle(QCPItemTracer::tsCircle);
    tracer->setPen(QPen(Qt::red));
    tracer->setBrush(Qt::red);
    tracer->setSize(5);  //描点大小

    filterButtonGroup = new QButtonGroup(this);
    filterButtonGroup->addButton(ui->noneFilterButton, 0);
    filterButtonGroup->addButton(ui->midFilterButton, 1);
    filterButtonGroup->addButton(ui->avgFliterButton, 2);
    ui->noneFilterButton->setChecked(true);

    ui->imageInfoTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->imageInfoTable->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    ui->switchReflineButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");
    ui->switchIndicateButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");
}

void LidarVideoView::connectInit()
{
    connect(filterButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(updateFilterMode(int)));
    connect(ui->switchReflineButton, SIGNAL(clicked()), this, SLOT(switchReflineVisible()));
    connect(ui->switchIndicateButton, SIGNAL(clicked()), this, SLOT(switchIndicateVisible()));
    connect(ui->imageColsBox, SIGNAL(valueChanged(int)), this, SLOT(updateCurveCols()));
    connect(ui->indicateValueBox, SIGNAL(valueChanged(int)), this, SLOT(updateIndicateValue()));
    connect(ui->saveImageButton, SIGNAL(clicked()), this, SLOT(savaImage()));
    connect(ui->loadImageButton, SIGNAL(clicked()), this, SLOT(loadImage()));
    connect(ui->showImageButton, SIGNAL(clicked()), this, SLOT(showAndPlayImage()));
    connect(ui->stopShowImageButton, SIGNAL(clicked()), this, SLOT(stopShowImage()));
    connect(ui->pixelCurveWidget, SIGNAL(mouseMove(QMouseEvent *)), this, SLOT(mouseMoveEvent(QMouseEvent*)));
}


#define APP_LIDAR_WIDTH_TOTAL      160
#define APP_LIDAR_HEIGTH_TOTAL     160//320//(VIDEO_HEIGHT_HARD_SAMPLE / SOFT_SUBSAMPLE_HEIGTH)
#define LASER_IMAGE_FITER_POINTS     35
#define MAX(a,b) (a>b?a:b)
#define MIN(a,b) (a<b?a:b)
#define ABS(a,b) ((a>b)?(a-b):(b-a))

#define CALCULATE_INVALID_LINE   (30)    //不等距标定步数的最大值，当前标定参数分区最高支持18步----(240*4)*22/1024 = 20.625kB

#define SUNSHINE_INTERFERENCE_OFFSET   (5)    //背景帧的阳光区域判断阈值
#define SUNSHINE_ADD_OFFSET   (30)    //背景帧的阳光区域增加列数

#define PEAKS_NUM_MAX 10
#define PEAK_DIFF 15
#define PEAK_FILL 230
#define ONLY_MAX 20

typedef uint8_t  PixValueType;
/*峰的参数*/
typedef struct {
    uint8_t bulge_head;
    uint8_t bulge_tail;
    uint8_t head_index;
    uint8_t tail_index;
    uint8_t refline;
    uint8_t width;
    uint8_t extreme_index;
    uint8_t extreme_value;
    uint8_t centroid_value;
    uint8_t relative_altitude;
    uint16_t centroid;
} PeakDef;

typedef struct
{
    PeakDef peaks[PEAKS_NUM_MAX];
    uint16_t candidate[APP_LIDAR_WIDTH_TOTAL][2];
    uint8_t only_index[ONLY_MAX];//保存唯一且地线外的质心的列索引
    uint8_t only_count;//记录唯一且地线外的质心的数量
    uint8_t soft_subsample;
    uint8_t hard_subsample;
    uint16_t width;
    uint16_t height;
    uint16_t all_pix_num;
    uint32_t all_pix_sum;
    uint8_t *confidence;
    uint8_t *peak_width;
    uint16_t *dis_thr_centroid;
    uint16_t *dis_thr_centroid_max;
    uint16_t *distance;
    uint16_t *centriod;

    uint8_t *last_confidence;
    uint8_t *last_peak_width;
    uint16_t *last_centriod;

    uint8_t *sunshine_flag;
    uint16_t pixel_higlight_num;

    //背景计算参数
    uint8_t  background_sunshine_start[APP_LIDAR_WIDTH_TOTAL];
    uint16_t background_sunshine_end[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  background_sunshine_refline[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  background_sunshine_flag[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  background_sunshine_max_value[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  background_sunshine_max_index[APP_LIDAR_WIDTH_TOTAL];

    uint8_t* sunshine_filter_flag;
    uint8_t  over_30_count[LASER_IMAGE_FITER_POINTS];
    uint16_t sum_over_190;//中间部分区域灰度值超过190的像素个数
}PixCurveInfoTypedef;

typedef struct
{
    uint16_t index[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  peak_width[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  confidence[APP_LIDAR_WIDTH_TOTAL];


 //   uint8_t  lidar_tx_buffer[LIDAR_TX_COUNT][LIDAR_TX_BUFFER_SIZE];


    uint32_t all_pix_sum;
    uint32_t background_all_pix_sum;

    uint8_t  sunshine_flag;

    uint8_t background_frame;
    uint8_t  background_real_frame;

    uint8_t  exposure_gain_grade;

    uint16_t last_index[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  last_confidence[APP_LIDAR_WIDTH_TOTAL];
    uint8_t  last_peak_width[APP_LIDAR_WIDTH_TOTAL];

    bool change_flag[APP_LIDAR_WIDTH_TOTAL];
    uint16_t final_index[APP_LIDAR_WIDTH_TOTAL];

//    uint8_t mutil_peak_flag[APP_LIDAR_WIDTH_TOTAL];
    uint8_t refline[APP_LIDAR_WIDTH_TOTAL];
    uint8_t last_refline[APP_LIDAR_WIDTH_TOTAL];
    uint8_t cols_average[APP_LIDAR_WIDTH_TOTAL];
     uint8_t cols_max_value[APP_LIDAR_WIDTH_TOTAL];
}LiDARDataDef;

LiDARDataDef lidardata;
LiDARDataDef* LiDARData = &lidardata;
PixCurveInfoTypedef PixCurveInfo;
//= {
//    .width = APP_LIDAR_WIDTH_TOTAL,                  // PixCurveInfo.width
//    .height = APP_LIDAR_HEIGTH_TOTAL,                // PixCurveInfo.height
//    .confidence = LiDARData->confidence,             // PixCurveInfo.confidence
//    .dis_thr_centroid = LiDARData->distance_threshold_centroid, // PixCurveInfo.dis_thr_centroid
//    .dis_thr_centroid_max = LiDARData->distance_threshold_centroid_max, // PixCurveInfo.dis_thr_centroid_max
// //   .distance = LiDARData->distance,                  // PixCurveInfo.distance
//    .centriod = LiDARData->index,                    // PixCurveInfo.centriod
//    .peak_width = LiDARData->peak_width,              // PixCurveInfo.peak_width
//    .sunshine_flag = &LiDARData->sunshine_flag       // PixCurveInfo.sunshine_flag
//};

uint16_t last_centroid = 65535;
uint16_t last_five_centroid[5] = {0};
uint8_t last_fifth_index = 0;
uint8_t InvalidPointCounts = 0;
bool ClosedBindFlag = false;
bool ReflectFlag = false;
uint8_t GroundMutilPeakCounts = 0;

#define GROUND_MAX_INDEX         (70)   //大地点云最大索引值
#define LASER_IMAGE_START_INDEX  (70)   //激光器成像起始索引值   最少涵盖区间30
#define LASER_IMAGE_END_INDEX    (100)   //激光器成像结束索引值
#define GROUND_START_INDEX       (15)

void DiffCalc(PixValueType *data, uint8_t size, uint8_t height, int16_t* diff,uint8_t *max_vlaue,uint8_t *average_value,uint8_t *max_value_index)
{
    uint8_t max_value_temp = data[0];
    uint8_t max_value_index_temp = 0;
    uint16_t sunm = 0;
    diff[0] = (data[3] - data[0]) * size / 2;
    diff[height - 2] = (data[height - 1] - data[height - 4]) * size / 2;

    for(uint8_t i = 1; i < height - 2; i++)
    {
        diff[i] = data[i + 2] - data[i - 1];
        if(data[i] > max_value_temp)
        {
            max_value_index_temp = i;
            max_value_temp = data[i];
        }
        sunm += data[i];
    }
    *max_value_index = max_value_index_temp;
    *max_vlaue = max_value_temp;
    *average_value = sunm / (height - 3);
}

void HeadTailCalc(PixValueType *data, uint8_t *peak_count, uint8_t thres, uint8_t height, int16_t* diff, PeakDef *peaks)
{
    bool find_flag = false;//false代表找head，true代表找tail
    int16_t partial_sum = 0;
    bool decline_flag = false;
    uint8_t extreme_value = 0;

    for(uint8_t i = 0; i < height - 1; i++)
    {
        if (*peak_count == PEAKS_NUM_MAX)
        {
            break;
        }

        if (find_flag == true && i == height - 2)//查找到结尾记录为tail结束
        {
            peaks[(*peak_count)++].bulge_tail = i;
            find_flag = false;
            decline_flag = false;
        }
    //        else if (find_flag == true /*&& i > peaks[*peak_count].bulge_head + 50 */&& data[i+1] > data[i])
    //        {
    //            peaks[(*peak_count)++].bulge_tail = i;
    //            find_flag = false;
    //            decline_flag = false;
    //        }
        else if (find_flag == true && extreme_value > data[i] + 120 && data[i+1] > data[i])//峰下降太多时退出
        {
            peaks[(*peak_count)++].bulge_tail = i;
            find_flag = false;
            decline_flag = false;
        }
        else if (find_flag == true && ABS(diff[i],0) <= thres && (diff[i - 1] <= -thres || decline_flag == true) && data[i] < PEAK_FILL)//寻找peak的tail,带取消abs
        {
            partial_sum = 0;
            decline_flag = true;
            for (int j = i+1; j < MIN(i+4,height - 1); j++)
            {
                partial_sum += diff[j];
            }

            // 打印当前循环的 i 和 peak_count
//            qDebug() << "i:"<<i<<"find_flag:" << find_flag
//                     << "peaks[(*peak_count)].bulge_tail:" << peaks[(*peak_count)].bulge_tail
//                     << "peaks[*peak_count].bulge_head "<< peaks[*peak_count].bulge_head
//                     << "partial_sum"<< partial_sum;
            if ((ABS(partial_sum,0) < thres * 3 && ABS(data[i+1], data[i]) < 20) || (data[i + 1] < data[peaks[*peak_count].bulge_head]))
            {
                int j ;
                for(j = i+1; j>=i-3 ; --j)
                {
                    if((data[j] * 2 <= data[j-2] + data[j-1]))
                    {
                        break;
                    }
                }
                peaks[(*peak_count)++].bulge_tail = j;
                i = j-1;
                find_flag = false;
                decline_flag = false;
            }
        }

        if (find_flag == false && diff[i] >= thres /*&& i > 10 */&& i < height - 5)//寻找peak的head，L3
        {
            extreme_value = 0;
            peaks[*peak_count].bulge_head = i;
            find_flag = true;
        }
        if (data[i] >= extreme_value && find_flag == true)
        {
            extreme_value = data[i];//找极大值
        }
    }
}

void FilterCentroid(PixValueType *data, uint16_t frame_line, PixCurveInfoTypedef *pix, bool sunshine_flag)
{

    PixCurveInfo.width = APP_LIDAR_WIDTH_TOTAL;
    PixCurveInfo.height = APP_LIDAR_HEIGTH_TOTAL;
    PixCurveInfo.confidence = LiDARData->confidence;
    PixCurveInfo.centriod = LiDARData->index;
    PixCurveInfo.peak_width = LiDARData->peak_width;
    PixCurveInfo.sunshine_flag = &LiDARData->sunshine_flag;

    PixCurveInfo.last_confidence = LiDARData->last_confidence;
    PixCurveInfo.last_centriod = LiDARData->last_index;
    PixCurveInfo.last_peak_width = LiDARData->last_peak_width;

    uint8_t valley_index = 0;
    uint8_t valley_diff_left = 0;
    uint8_t valley_diff_right = 0;
    uint8_t peak_count = 0;//已使用峰的数量
    int8_t peak_count_copy = 0;
    uint8_t refline_diff = 0;
    uint8_t height = APP_LIDAR_HEIGTH_TOTAL; //120
    uint8_t width = APP_LIDAR_WIDTH_TOTAL;   //160
    int16_t diff[APP_LIDAR_HEIGTH_TOTAL - 1] = {0};
    uint8_t windowssize = 5;
    uint8_t peak_thresold = 5;//判定单个峰起始和终止的阈值
    uint16_t translation = 0;
    uint32_t weight_sum = 0;
    bool divide_peak = false;
    uint8_t divider_index = 0;
    uint8_t index = 0;
    uint16_t pixel_sum = 0;
    uint8_t refline_left = 0;
    uint8_t refline_right = 0;
    uint8_t centroid_round = 0;
    uint8_t max_index = 0;
    uint8_t max_value = 0;
    uint8_t max_value_temp = 0;
    uint8_t average_value = 0;
    uint8_t width_max = 60;
    int8_t refline_thres = 8;//查找参考线阈值
    uint8_t extreme_index_left = 0;
    uint8_t extreme_index_right = 0;
    bool decline_flag = false;
    uint8_t centroid_index = 0;
    uint16_t lut_blind_centroid = 7500;//30mm标定数据在标定表中的位置，盲区内加严滤波
    uint16_t lut_ground_centroid = MIN(pix->dis_thr_centroid[frame_line] - 300, height * 100);//设定为地线阈值
    uint8_t max_value_index_temp = 0;
    if (frame_line == 0)
    {
        memset(pix->only_index, 0, sizeof(pix->only_index));
        memset(last_five_centroid, 0, sizeof(last_five_centroid));
        memset(lidardata.change_flag, 0, sizeof(lidardata.change_flag));
        memset(lidardata.final_index, 0, sizeof(lidardata.final_index));
        memset(lidardata.index, 0, sizeof(lidardata.index));
        memset(lidardata.last_index, 0, sizeof(lidardata.last_index));
        pix->only_count = 0;
        pix->all_pix_num = 0;
        pix->all_pix_sum = 0;
        pix->pixel_higlight_num = 0;
        LiDARData->all_pix_sum = 0;
        last_centroid = 0;
    }

    memset(pix->peaks, 0, sizeof(PeakDef) * PEAKS_NUM_MAX);

    // if (pixel_sum < 20 * height)//峰太弱时减少窗口和阈值以找到更弱的峰
    {
        windowssize = 3;
        peak_thresold = 4;
    }

    //阳光场景下提高找突起的阈值
    if(sunshine_flag)
    {
        peak_thresold = 15;
    }


    DiffCalc(data, windowssize, height, diff,&max_value_temp,&average_value,&max_value_index_temp);//计算差分
    if(!sunshine_flag && frame_line > LASER_IMAGE_START_INDEX + 10  && LASER_IMAGE_END_INDEX-10 > frame_line &&  max_value_temp > 200)
    {
        peak_thresold = 7;
    }
    LiDARData->cols_average[frame_line] = average_value;
    LiDARData->cols_max_value[frame_line] = max_value_temp;
    HeadTailCalc(data, &peak_count, windowssize * peak_thresold, height, diff, pix->peaks);//计算多个峰的head和tail

    //非阳光环境下近距离盲区内，最大值灰度值数据在前几个数据中或者非阳光模式下data[0] > 150
    if(!sunshine_flag && ((max_value_index_temp < 5 /*&& max_value_index_temp > 32*/)|| ((data[0] > 100 || (data[0] > 70 && max_value_index_temp < (data[0] + 30))))) && frame_line < 80 )
    {
       InvalidPointCounts++;
       // return;
    }
    peak_count_copy = peak_count - 1;

    //通过前60列数据存在多峰来判断是否存在二次反射，与正常情况做区分
    if(frame_line < 60 && peak_count > 1 && !ReflectFlag)
    {
        GroundMutilPeakCounts++;
        if(GroundMutilPeakCounts > 5)
        {
            ReflectFlag = true;
        }
    }


//    if(35<frame_line && frame_line <40)
//    {
//        for(int i=0;i<peak_count;i++)
//        {
//            // 打印当前循环的 i 和 peak_count
//            qDebug() << "i:" << i << "peak_count:" << peak_count
//                     << "frame_line:" << frame_line
//                     << "pix->peaks[i].bulge_head"<< pix->peaks[i].bulge_head
//                     << "pix->peaks[i].bulge_tail"<< pix->peaks[i].bulge_tail;
//        }
//    }
    for (int8_t i = peak_count_copy; i >= 0; i--)//初始peak计算，从后往前遍历，优先保证后面的峰够用
    {
        valley_index = 0;
        valley_diff_left = 0;
        valley_diff_right = 0;
        divide_peak = false;
        for (uint8_t j = pix->peaks[i].bulge_head; j <= pix->peaks[i].bulge_tail; j++)
        {
            if (data[j] >= pix->peaks[i].extreme_value)
            {
                pix->peaks[i].extreme_index = j;
                pix->peaks[i].extreme_value = data[j];//找极大值
            }
            if (data[j] + MAX(PEAK_DIFF, valley_diff_left) < pix->peaks[i].extreme_value || (valley_index > 0 && data[j+1] > data[j] + valley_diff_left))//找峰内双峰
            {
                valley_index = j;
                valley_diff_left = pix->peaks[i].extreme_value - data[j];
            }
            else if (valley_index > 0 && data[j] > data[valley_index] + MAX(PEAK_DIFF, valley_diff_right))
            {
                divider_index = valley_index;
                divide_peak = true;
                valley_diff_right = data[j] - data[valley_index];
            }
        }
        extreme_index_left = pix->peaks[i].extreme_index;
        extreme_index_right = pix->peaks[i].extreme_index;
/*************************************************************************************************************/
        if (pix->peaks[i].extreme_value > max_value)
        {
            max_index = pix->peaks[i].extreme_index;
            max_value = pix->peaks[i].extreme_value;
        }
/*************************************************************************************************************/
        if(divide_peak == true)//计算分裂峰
        {
            pix->peaks[peak_count].extreme_index = 0;
            pix->peaks[peak_count].extreme_value = 0;
            if (peak_count < PEAKS_NUM_MAX && divider_index > pix->peaks[i].bulge_head + 1)
            {
                for (uint8_t j = pix->peaks[i].bulge_head; j <= divider_index; j++)
                {
                    if (data[j] >= pix->peaks[peak_count].extreme_value)
                    {
                        extreme_index_left = j;
                        pix->peaks[peak_count].extreme_index = j;
                        pix->peaks[peak_count].extreme_value = data[j];
                    }
                }

                if(pix->peaks[peak_count].extreme_index == divider_index)
                {
                    pix->peaks[peak_count].extreme_value = pix->peaks[i].extreme_value;
                }

                translation = 0;
                weight_sum = 0;
                decline_flag = false;
                refline_left = data[pix->peaks[i].bulge_head];
                refline_right = data[divider_index];
                for (int j = pix->peaks[peak_count].extreme_index; j > pix->peaks[i].bulge_head; j--)
                {
                    if ((data[j + 1] >= data[j] + refline_thres && data[j + 1] < pix->peaks[peak_count].extreme_value)\
                        || (data[j] + 5 * refline_thres < pix->peaks[peak_count].extreme_value))
                    {
                        decline_flag = true;
                    }
                    if (decline_flag == true && data[j - 1] > data[j])
                    {
                        refline_left = data[j];
                        break;
                    }
                }
                decline_flag = false;
                for (int j = pix->peaks[peak_count].extreme_index; j < divider_index; j++)
                {
                    if ((data[j] + refline_thres <= data[j - 1] && data[j - 1] < pix->peaks[peak_count].extreme_value)\
                        || (data[j] + 5 * refline_thres < pix->peaks[peak_count].extreme_value))
                    {
                        decline_flag = true;
                    }
                    if (decline_flag == true && data[j + 1] > data[j])
                    {
                        refline_right = data[j];
                        break;
                    }
                }
                pix->peaks[peak_count].refline = MAX(refline_left, refline_right);

                index = pix->peaks[peak_count].extreme_index;
                while (data[index] > pix->peaks[peak_count].refline)//极大值往左找
                {
                    refline_diff = data[index] - pix->peaks[peak_count].refline;
                    translation += refline_diff;
                    weight_sum += refline_diff * index;
                    index--;
                }
                pix->peaks[peak_count].head_index = index;
                index = pix->peaks[peak_count].extreme_index + 1;
                while (data[index] > pix->peaks[peak_count].refline)//极大值往右找
                {
                    refline_diff = data[index] - pix->peaks[peak_count].refline;
                    translation += refline_diff;
                    weight_sum += refline_diff * index;
                    index++;
                }
                pix->peaks[peak_count].tail_index = index;
                pix->peaks[peak_count].width = pix->peaks[peak_count].tail_index - pix->peaks[peak_count].head_index;
                pix->peaks[peak_count].relative_altitude = pix->peaks[peak_count].extreme_value - pix->peaks[peak_count].refline;
                if (translation != 0 /*&& pix->peaks[peak_count].width < width_max && pix->peaks[peak_count].width < pix->peaks[peak_count].relative_altitude*/)
                {
                    pix->peaks[peak_count].centroid = weight_sum * 100 / translation;
                    centroid_round = ((pix->peaks[peak_count].centroid / 10) % 10 > 4) ? 1 : 0;
                    centroid_index = pix->peaks[peak_count].centroid / 100 + centroid_round;
                    pix->peaks[peak_count].centroid_value = pix->peaks[peak_count].extreme_value;//data[centroid_index];
                    extreme_index_left = pix->peaks[peak_count].extreme_index;
                    peak_count++;
                }
            }

            if (peak_count < PEAKS_NUM_MAX && pix->peaks[i].bulge_tail > divider_index + 1)
            {
                pix->peaks[peak_count].extreme_index = 0;
                pix->peaks[peak_count].extreme_value = 0;
                for (uint8_t j = divider_index; j <= pix->peaks[i].bulge_tail; j++)
                {
                    if (data[j] >= pix->peaks[peak_count].extreme_value)
                    {
                        extreme_index_right = j;
                        pix->peaks[peak_count].extreme_index = j;
                        pix->peaks[peak_count].extreme_value = data[j];
                    }
                }

                if(pix->peaks[peak_count].extreme_index == divider_index)
                {
                    pix->peaks[peak_count].extreme_value = pix->peaks[i].extreme_value;
                }

                translation = 0;
                weight_sum = 0;
                decline_flag = false;
                refline_left = data[divider_index];
                refline_right = data[pix->peaks[i].bulge_tail];

                for (int j = pix->peaks[peak_count].extreme_index; j > divider_index; j--)
                {
                    if ((data[j + 1] >= data[j] + refline_thres && data[j + 1] < pix->peaks[peak_count].extreme_value)\
                        || (data[j] + 5 * refline_thres < pix->peaks[peak_count].extreme_value))
                    {
                        decline_flag = true;
                    }
                    if (decline_flag == true && data[j - 1] > data[j])
                    {
                        refline_left = data[j];
                        break;
                    }
                }
                decline_flag = false;
                for (int j = pix->peaks[peak_count].extreme_index; j < pix->peaks[i].bulge_tail; j++)
                {
                    if ((data[j] + refline_thres <= data[j - 1] && data[j - 1] < pix->peaks[peak_count].extreme_value)\
                        || (data[j] + 5 * refline_thres <= data[j - 1]))
                    {
                        decline_flag = true;
                    }
                    if (decline_flag == true && data[j + 1] > data[j])
                    {
                        refline_right = data[j];
                        break;
                    }
                }
                pix->peaks[peak_count].refline = MAX(refline_left, refline_right);

                index = pix->peaks[peak_count].extreme_index;

                while (data[index] > pix->peaks[peak_count].refline)//极大值往左找
                {
                    refline_diff = data[index] - pix->peaks[peak_count].refline;
                    translation += refline_diff;
                    weight_sum += refline_diff * index;
                    index--;
                }
                pix->peaks[peak_count].head_index = index;
                index = pix->peaks[peak_count].extreme_index + 1;
                while (data[index] > pix->peaks[peak_count].refline)//极大值往右找
                {
                    refline_diff = data[index] - pix->peaks[peak_count].refline;
                    translation += refline_diff;
                    weight_sum += refline_diff * index;
                    index++;
                }
                pix->peaks[peak_count].tail_index = index;
                pix->peaks[peak_count].width = pix->peaks[peak_count].tail_index - pix->peaks[peak_count].head_index;
                pix->peaks[peak_count].relative_altitude = pix->peaks[peak_count].extreme_value - pix->peaks[peak_count].refline;

                if (translation != 0 /*&& pix->peaks[peak_count].width < width_max && pix->peaks[peak_count].width < pix->peaks[peak_count].relative_altitude*/)
                {
                    pix->peaks[peak_count].centroid = weight_sum * 100 / translation;
                    centroid_round = ((pix->peaks[peak_count].centroid / 10) % 10 > 4) ? 1 : 0;
                    centroid_index = pix->peaks[peak_count].centroid / 100 + centroid_round;
                    pix->peaks[peak_count].centroid_value = pix->peaks[peak_count].extreme_value;// data[centroid_index];
                    extreme_index_right = pix->peaks[peak_count].extreme_index;
                    peak_count++;
                }
            }
        }
/*************************************************************************************************************************/
        translation = 0;
        weight_sum = 0;
        decline_flag = false;
        refline_left = data[pix->peaks[i].bulge_head];
        refline_right = data[pix->peaks[i].bulge_tail];
        for (int j = extreme_index_left; j > pix->peaks[i].bulge_head; j--)
        {
            if ((data[j + 1] >= data[j] + refline_thres && data[j + 1] < data[extreme_index_left]) \
                || (data[j] + 5 * refline_thres < data[extreme_index_left]))
            {
                decline_flag = true;
            }
            if (decline_flag == true && data[j - 1] > data[j])
            {
                refline_left = data[j];
                break;
            }
        }
        decline_flag = false;
        for (int j = extreme_index_right; j < pix->peaks[i].bulge_tail; j++)
        {
            if ((data[j] + refline_thres <= data[j - 1] && data[j - 1] < data[extreme_index_right])\
                || (data[j] + 5 * refline_thres < data[extreme_index_right]))
            {
                decline_flag = true;
            }
            if (decline_flag == true && data[j + 1] > data[j])
            {
                refline_right = data[j];
                break;
            }
        }
        pix->peaks[i].refline = MAX(refline_left, refline_right);

        index = pix->peaks[i].extreme_index;

//        if(30<frame_line && frame_line <35)
//        {
//            // 打印当前循环的 i 和 peak_count
//            qDebug() << "frame_line:" << frame_line << "extreme_index_right:" << extreme_index_right;
//            qDebug() << "extreme_index_left:" << extreme_index_left
//                     << "frame_line:" << frame_line
//                     << "divider_index"<<divider_index
//                     << "pix->peaks[i].bulge_head"<< pix->peaks[i].bulge_head
//                     << "pix->peaks[i].bulge_tail"<< pix->peaks[i].bulge_tail
//                     << "pix->peaks[i].refline"<< pix->peaks[i].refline;
//        }
        uint8_t  min_value_temp = 255;
        //for (uint8_t j = pix->peaks[i].bulge_head; j <= pix->peaks[i].bulge_tail; j++)
        for (uint8_t j = extreme_index_left; j <= extreme_index_right; j++)
        {
            if (data[j]< min_value_temp)
            {
                min_value_temp = data[j];
            }
        }
//        qDebug()<< "min_value_temp2"<< min_value_temp<< "pix->peaks[i].refline2"<< pix->peaks[i].refline;
        if(divider_index != 0 && min_value_temp != 0  && min_value_temp <= pix->peaks[i].refline )
        {
            pix->peaks[i].refline = min_value_temp - 1;
        }
//        qDebug()<< "pix->peaks[i].refline3"<< pix->peaks[i].refline;
        while (data[index] > pix->peaks[i].refline)//极大值往左找
        {
            refline_diff = data[index] - pix->peaks[i].refline;
            translation += refline_diff;
            weight_sum += refline_diff * index;
            index--;
        }
        pix->peaks[i].head_index = index;
        index = pix->peaks[i].extreme_index + 1;
        while (data[index] > pix->peaks[i].refline)//极大值往右找
        {
            refline_diff = data[index] - pix->peaks[i].refline;
            translation += refline_diff;
            weight_sum += refline_diff * index;
            index++;
        }
        pix->peaks[i].tail_index = index;
        pix->peaks[i].width = pix->peaks[i].tail_index - pix->peaks[i].head_index;
        pix->peaks[i].relative_altitude = pix->peaks[i].extreme_value - pix->peaks[i].refline;


        //银色踢脚线中间合并峰宽有时高达68
        if(frame_line > 70 && frame_line < 90 )
        {
            width_max = 80;
        }else
        {
            width_max = 60;
        }


        if (translation != 0 && pix->peaks[i].width < width_max /*&& pix->peaks[i].width < pix->peaks[i].relative_altitude*/)
        {
            pix->peaks[i].centroid = weight_sum * 100 / translation;
            centroid_round = ((pix->peaks[i].centroid / 10) % 10 > 4) ? 1 : 0;//四舍五入
            centroid_index = pix->peaks[i].centroid / 100 + centroid_round;
            pix->peaks[i].centroid_value = pix->peaks[i].extreme_value;//data[centroid_index];
        }

    //        if(20<frame_line && frame_line <30)
    //        {
    //            // 打印当前循环的 i 和 peak_count
    //            qDebug() << "i:" << i << "peak_count:" << peak_count;
    //            qDebug() << "sunshine_flag:" << sunshine_flag
    //                     << "frame_line:" << frame_line
    //                     << "pix->centriod[frame_line]:" << pix->centriod[frame_line]
    //                     << "pix->confidence[frame_line]:" << pix->confidence[frame_line]
    //                     << "pix->peaks[i].centroid_value:" << pix->peaks[i].centroid_value
    //                     << "last_centroid:" << last_centroid
    //                     << "pix->peaks[i].bulge_head"<< pix->peaks[i].bulge_head
    //                     << "pix->peaks[i].bulge_tail"<< pix->peaks[i].bulge_tail
    //                     << "pix->peaks[i].head_index"<< pix->peaks[i].head_index
    //                     << "pix->peaks[i].tail_index"<< pix->peaks[i].tail_index
    //                     << "divider_index"<<divider_index
    //                     << "pix->peaks[i].refline"<< pix->peaks[i].refline;
    //        }

        //求解地线部分的middle peak
        if(frame_line < GROUND_MAX_INDEX && divider_index == 0 &&  peak_count == 1 && peak_count < PEAKS_NUM_MAX /*&& pix->peaks[i].extreme_value > 250*/)
        {
            translation = 0;
            weight_sum = 0;
            pix->peaks[peak_count].refline = (pix->peaks[i].refline + pix->peaks[i].extreme_value)/2;
            pix->peaks[peak_count].extreme_value = pix->peaks[i].extreme_value;
            pix->peaks[peak_count].extreme_index = pix->peaks[i].extreme_index;

            index = pix->peaks[peak_count].extreme_index;
            while (data[index] > pix->peaks[peak_count].refline)//极大值往左找
            {
                refline_diff = data[index] - pix->peaks[peak_count].refline;
                translation += refline_diff;
                weight_sum += refline_diff * index;
                index--;
            }
            pix->peaks[peak_count].head_index = index;
            index = pix->peaks[peak_count].extreme_index + 1;
            while (data[index] > pix->peaks[peak_count].refline)//极大值往右找
            {
                refline_diff = data[index] - pix->peaks[peak_count].refline;
                translation += refline_diff;
                weight_sum += refline_diff * index;
                index++;
            }
            pix->peaks[peak_count].tail_index = index;

            uint8_t index_diff = ABS(ABS( pix->peaks[peak_count].tail_index, pix->peaks[i].tail_index),ABS( pix->peaks[peak_count].head_index, pix->peaks[i].head_index));

            qDebug()<< "frame_line:" << frame_line
                     << "  index_diff:" << index_diff;
            if(ABS(ABS( pix->peaks[peak_count].tail_index, pix->peaks[i].tail_index),ABS( pix->peaks[peak_count].head_index, pix->peaks[i].head_index)) > 1)
            {
                pix->peaks[i].width = pix->peaks[peak_count].tail_index - pix->peaks[peak_count].head_index;
                pix->peaks[i].relative_altitude = pix->peaks[peak_count].extreme_value - pix->peaks[peak_count].refline;

                if (translation != 0 && pix->peaks[i].width < width_max /*&& pix->peaks[i].width < pix->peaks[i].relative_altitude*/)
                {
                    uint16_t  centroid = pix->peaks[i].centroid;
                    pix->peaks[i].centroid = weight_sum * 100 / translation;
                    centroid_round = ((pix->peaks[i].centroid / 10) % 10 > 4) ? 1 : 0;//四舍五入
                    centroid_index = pix->peaks[i].centroid / 100 + centroid_round;
                    pix->peaks[i].centroid_value = pix->peaks[i].extreme_value;//data[centroid_index]
                    pix->peaks[i].refline = pix->peaks[peak_count].refline;
                    qDebug()<< "ok frame_line:" << frame_line
                             << " pix->peaks[i].centroid:" <<centroid
                             << " pix->peaks[peak_count].centroid:" << pix->peaks[i].centroid;
                }
            }
        }

        //合并临近凸起
        #if 0
        if(!sunshine_flag && (i + 1)<= peak_count_copy && pix->peaks[i+1].bulge_head < pix->peaks[i].bulge_tail + 10
           && peak_count < PEAKS_NUM_MAX /*&& data[pix->peaks[i].bulge_head] > 150 && data[pix->peaks[i-1].bulge_tail] > 150*/)
        {
            translation = 0;
            weight_sum = 0;
            decline_flag = false;
            pix->peaks[i].bulge_tail = pix->peaks[i+1].bulge_tail;
            extreme_index_left = pix->peaks[i].extreme_index;
            extreme_index_right = pix->peaks[i+1].extreme_index;
            refline_left = data[pix->peaks[i].bulge_head];
            refline_right = data[pix->peaks[i+1].bulge_tail];

            if(data[extreme_index_left] > data[extreme_index_right])
            {
                pix->peaks[peak_count].extreme_index = extreme_index_left;
                pix->peaks[peak_count].extreme_value = data[extreme_index_left];
            }
            else
            {
                pix->peaks[peak_count].extreme_index = extreme_index_right;
                pix->peaks[peak_count].extreme_value = data[extreme_index_right];
            }

            for (int j = extreme_index_left; j > pix->peaks[i].bulge_head; j--)
            {
                if ((data[j + 1] >= data[j] + refline_thres && data[j + 1] < data[extreme_index_left]) \
                    || (data[j] + 5 * refline_thres < data[extreme_index_left]))
                {
                    decline_flag = true;
                }
                if (decline_flag == true && data[j - 1] > data[j])
                {
                    refline_left = data[j];
                    break;
                }
            }
            decline_flag = false;
            for (int j = extreme_index_right; j < pix->peaks[i].bulge_tail; j++)
            {
                if ((data[j] + refline_thres <= data[j - 1] && data[j - 1] < data[extreme_index_right])\
                    || (data[j] + 5 * refline_thres < data[extreme_index_right]))
                {
                    decline_flag = true;
                }
                if (decline_flag == true && data[j + 1] > data[j])
                {
                    refline_right = data[j];
                    break;
                }
            }
            pix->peaks[peak_count].refline = MAX(refline_left, refline_right);

            uint8_t  min_value_temp = 255;
            for (uint8_t j = extreme_index_left; j <= extreme_index_right; j++)
            {
                if (data[j]< min_value_temp)
                {
                    min_value_temp = data[j];
                }
            }

            if(min_value_temp != 0 && min_value_temp <= pix->peaks[peak_count].refline)
            {
                pix->peaks[peak_count].refline = min_value_temp - 1;
            }

            index = pix->peaks[peak_count].extreme_index;
            while (data[index] > pix->peaks[peak_count].refline)//极大值往左找
            {
                refline_diff = data[index] - pix->peaks[peak_count].refline;
                translation += refline_diff;
                weight_sum += refline_diff * index;
                index--;
            }
            pix->peaks[peak_count].head_index = index;
            index = pix->peaks[peak_count].extreme_index + 1;
            while (data[index] > pix->peaks[i].refline)//极大值往右找
            {
                refline_diff = data[index] - pix->peaks[peak_count].refline;
                translation += refline_diff;
                weight_sum += refline_diff * index;
                index++;
            }
            pix->peaks[peak_count].tail_index = index;
            pix->peaks[peak_count].width = pix->peaks[peak_count].tail_index - pix->peaks[peak_count].head_index;
            pix->peaks[peak_count].relative_altitude = pix->peaks[peak_count].extreme_value - pix->peaks[peak_count].refline;
            if (translation != 0 /*&& pix->peaks[peak_count].width < width_max && pix->peaks[peak_count].width < pix->peaks[peak_count].relative_altitude*/)
            {
                pix->peaks[peak_count].centroid = weight_sum * 100 / translation;
                centroid_round = ((pix->peaks[peak_count].centroid / 10) % 10 > 4) ? 1 : 0;//四舍五入
                centroid_index = pix->peaks[peak_count].centroid / 100 + centroid_round;
                pix->peaks[peak_count].centroid_value = pix->peaks[peak_count].extreme_value;//data[centroid_index];
                peak_count++;
            }
        }
        #endif
    }

    //防止近距离盲区过曝，切换到量程范围内时，调节曝光时间过长
    if(0 == max_value || sunshine_flag)
    {
        max_value = max_value_temp;
    }

    if(LiDARData->background_real_frame == 0 || sunshine_flag)
    {
        if(frame_line < 20 )
        {
            LiDARData->all_pix_sum += max_value ;
        }
        else if(frame_line < 60)
        {
            LiDARData->all_pix_sum += 2*max_value;
        }
        else if(frame_line < 80)
        {
            LiDARData->all_pix_sum += max_value;
        }
    }
/*************************************************************************************************************/
    //if (isFilterEnable == false)//改为选用最大值，加标定模式
    {
        uint16_t centroid_diff = 65535;
        uint16_t max_temp = 0;
        uint8_t max_centroid_index = 0;
        uint16_t max_centroid = 0;
        pix->centriod[frame_line] = 0;
        uint8_t centroid_index = 0;
        bool confidence_filter = false;
        //后面改用switch case形式
        //银色反光面反射强于地线光斑时
        //置信度相差过大时选择置信度大的峰
        if(last_centroid == 0 || frame_line <= GROUND_START_INDEX)
        {
            last_centroid = 0;
            centroid_diff = 0;
        }

        if(last_centroid != 0 && divider_index != 0 && frame_line <= LASER_IMAGE_START_INDEX && ABS(last_five_centroid[3], last_five_centroid[4]) < 200
           && ABS(last_five_centroid[2], last_five_centroid[3]) < 200)
        {
            last_centroid = last_five_centroid[3];
        }

        for (int i = 0; i < peak_count; i++)
        {
//            if(40<frame_line && frame_line <60)
//            {
//                // 打印当前循环的 i 和 peak_count
//                qDebug() << "i:" << i << "peak_count:" << peak_count;
//                qDebug() << "sunshine_flag:" << sunshine_flag
//                         << "frame_line:" << frame_line
//                         << "pix->centriod[frame_line]:" << pix->centriod[frame_line]
//                         << "pix->confidence[frame_line]:" << pix->confidence[frame_line]
//                         << "pix->peaks[i].centroid_value:" << pix->peaks[i].centroid_value
//                         << "last_centroid:" << last_centroid
//                         << "pix->peaks[i].bulge_head"<< pix->peaks[i].bulge_head
//                         << "pix->peaks[i].bulge_tail"<< pix->peaks[i].bulge_tail
//                         << "pix->peaks[i].head_index"<< pix->peaks[i].head_index
//                         << "pix->peaks[i].tail_index"<< pix->peaks[i].tail_index
//                         << "divider_index"<<divider_index
//                         << "pix->peaks[i].refline"<< pix->peaks[i].refline
//                         << "pix->peaks[i].centriod"<< pix->peaks[i].centroid;
//            }

//            //找出最大质心索引值
//            if(pix->peaks[i].centroid > max_centroid)
//            {
//                max_centroid = pix->peaks[i].centroid;
//                max_centroid_index = i;
//            }

           if(!sunshine_flag && (frame_line <= GROUND_START_INDEX || frame_line > LASER_IMAGE_END_INDEX) &&  pix->centriod[frame_line] != 0 && ABS(pix->confidence[frame_line], pix->peaks[i].centroid_value) > 50)
           {
                confidence_filter = true;
           }

            //通常情况下，选择与上一质心接近的点
            if (!confidence_filter && pix->peaks[i].centroid != 0 && ABS(pix->peaks[i].centroid, last_centroid) < centroid_diff && last_centroid != 0/*&&(!multi_peak_flag ||  peak_count == 1)*/)
            {
                centroid_index = i;
                centroid_diff = ABS(pix->peaks[i].centroid, last_centroid);
                max_temp = pix->peaks[i].centroid_value;
                pix->centriod[frame_line] = pix->peaks[i].centroid;
                pix->confidence[frame_line] = max_temp;
                pix->peak_width[frame_line] =  pix->peaks[i].width;
                LiDARData->refline[frame_line] =  pix->peaks[i].refline;
            }

            //上一质心为0的情况下，选择远距离的质心
            if (!confidence_filter && pix->peaks[i].centroid != 0 && ABS(pix->peaks[i].centroid, last_centroid) > centroid_diff  && last_centroid == 0/*&&(!multi_peak_flag ||  peak_count == 1)*/)
            {
                centroid_index = i;
                centroid_diff = ABS(pix->peaks[i].centroid, last_centroid);
                max_temp = pix->peaks[i].centroid_value;
                pix->centriod[frame_line] = pix->peaks[i].centroid;
                pix->confidence[frame_line] = max_temp;
                pix->peak_width[frame_line] =  pix->peaks[i].width;
                LiDARData->refline[frame_line] =  pix->peaks[i].refline;
            }


            //阳光模式下，选择置信度大的质心
            if ((sunshine_flag||confidence_filter) && pix->peaks[i].centroid != 0 && pix->peaks[i].centroid_value > max_temp  /*&&(!multi_peak_flag ||  peak_count == 1)*/)
            {
                centroid_index = i;
                centroid_diff = ABS(pix->peaks[i].centroid, last_centroid);
                max_temp = pix->peaks[i].centroid_value;
                pix->centriod[frame_line] = pix->peaks[i].centroid;
                pix->confidence[frame_line] = max_temp;
                pix->peak_width[frame_line] =  pix->peaks[i].width;
                LiDARData->refline[frame_line] =  pix->peaks[i].refline;
            }

        }

        //记录上一个质心值
        if((pix->centriod[frame_line] < 5000) || (ABS(pix->centriod[frame_line],last_five_centroid[4]) < 200 && pix->centriod[frame_line] > 5000))
        {
           last_centroid = pix->centriod[frame_line];
        }
        //与回溯第五个质心值作比较
        if(/*frame_line > 70 &&*/ frame_line < LASER_IMAGE_END_INDEX /*&& pix->centriod[frame_line] != 0*/)
        {
            last_five_centroid[0] = last_five_centroid[1];
            last_five_centroid[1] = last_five_centroid[2];
            last_five_centroid[2] = last_five_centroid[3];
            last_five_centroid[3] = last_five_centroid[4];
            last_five_centroid[4] = pix->centriod[frame_line];

            if(/*frame_line >= 75 && */last_five_centroid[0] > 0
                && (/*last_centroid - last_five_centroid[0] < 180 ||*/ frame_line > LASER_IMAGE_START_INDEX)
            )
            {
                last_centroid =  last_five_centroid[0];
                if(ABS(last_five_centroid[0], last_five_centroid[1]) < 200)
                {
                    last_centroid =  last_five_centroid[0];
                }else  if(ABS(last_five_centroid[2], last_five_centroid[1]) < 200)
                {
                    last_centroid =  last_five_centroid[1];
                }
            }

        }

        //阳光场景滤波,待进一步验证
        uint8_t centriod_temp =  pix->centriod[frame_line]/100;
        if(sunshine_flag && centriod_temp < pix->background_sunshine_max_index[frame_line] + 30 && centriod_temp > 0)
        {
            if(frame_line > 35 && (pix->confidence[frame_line] < pix->background_sunshine_max_value[frame_line] + 80))
            {
               pix->centriod[frame_line] = 0;
            }

            if(frame_line <= 35 && pix->background_sunshine_max_index[frame_line] > centriod_temp && pix->peak_width[frame_line] > pix->background_sunshine_max_index[frame_line] - centriod_temp)
            {
               pix->centriod[frame_line] = 0;
            }

            if(centriod_temp >= pix->background_sunshine_max_index[frame_line] && centriod_temp -  pix->background_sunshine_max_index[frame_line] < pix->peak_width[frame_line] + 10)
            {
                pix->centriod[frame_line] = 0;
            }
        }

        if(sunshine_flag && (pix->peak_width[frame_line] > 20 && frame_line > GROUND_MAX_INDEX))
        {
              pix->centriod[frame_line] = 0;
        }

        //镜面特殊角度位置中间部分过曝,平均值过高，有近距离噪点
        if(!sunshine_flag && peak_count_copy != 0 && ((average_value > 70 && max_value_temp < 200) || average_value > 100))
        {
            pix->centriod[frame_line] = 0;
        }

        uint16_t wall_space_index = 8279;
         uint16_t Skirting_Line_index = 7834;

         if(Skirting_Line_index)
         {
             if(peak_count > 1)
             {
                 centroid_diff = 65535;
                 uint8_t start_index = 0;
                 if(peak_count > 2 && divider_index != 0)
                 {
                     start_index = 1;
                 }
                 for (int i = start_index; i < peak_count; i++)
                 {
         //                if(pix->peaks[i].centroid > max_centroid && i != centroid_index)
         //                {
         //                    max_centroid = pix->peaks[i].centroid;
         //                    PixCurveInfo.last_confidence[frame_line] = pix->peaks[i].centroid_value;
         //                    PixCurveInfo.last_centriod[frame_line] = pix->peaks[i].centroid;
         //                    PixCurveInfo.last_peak_width[frame_line] =  pix->peaks[i].width;
         //              //      LiDARData->last_refline[frame_line] =  pix->peaks[i].refline;
         //                }

                     //墙面加踢脚线场景，第二峰选离墙面近的
                     if( i != centroid_index && pix->peaks[i].centroid < wall_space_index && ABS(pix->peaks[i].centroid,wall_space_index ) < centroid_diff)
                     {
                         centroid_diff = ABS(pix->peaks[i].centroid,wall_space_index );
                         PixCurveInfo.last_confidence[frame_line] = pix->peaks[i].centroid_value;
                         PixCurveInfo.last_centriod[frame_line] = pix->peaks[i].centroid;
                         PixCurveInfo.last_peak_width[frame_line] =  pix->peaks[i].width;
                     }
                 }
             }
         }


        //踢脚线场景二次选峰
        if(0)
        {
          if(frame_line == APP_LIDAR_WIDTH_TOTAL-10)
          {
              for(int j=APP_LIDAR_WIDTH_TOTAL-1; j >=0;j--)
              {
                  if(PixCurveInfo.last_centriod[j] != 0 && PixCurveInfo.last_centriod[j] <wall_space_index )
                       {
                           if(/*count < 10 && */ABS(PixCurveInfo.last_centriod[j], Skirting_Line_index) < 200
                              || (ABS(PixCurveInfo.last_centriod[j], last_centroid) < 200)
                            /*&& ABS(PixCurveInfo.centriod[j], LastFindWallStatue.Skirting_Line_index) > 200*/ )
                           {
                               PixCurveInfo.centriod[j] = PixCurveInfo.last_centriod[j];
                               PixCurveInfo.confidence[j] = PixCurveInfo.last_confidence[j];
                               PixCurveInfo.peak_width[j] = PixCurveInfo.last_peak_width[j];
                               LiDARData->change_flag[j] = true;
                               lidardata.final_index[j] = PixCurveInfo.last_centriod[j];
                           }

   //                        if(count >= 10)
   //                        {
   //                            if(ABS(PixCurveInfo.centriod[j], last_centroid) > ABS(PixCurveInfo.last_centriod[j], last_centroid))
   //                            {
   //                                PixCurveInfo.centriod[j] = PixCurveInfo.last_centriod[j];
   //                                PixCurveInfo.confidence[j] = PixCurveInfo.last_confidence[j];
   //                                PixCurveInfo.peak_width[j] = PixCurveInfo.last_peak_width[j];
   //                            }
   //                        }

                       }
                       if(PixCurveInfo.centriod[j] != 0)
                       {
                          last_centroid = PixCurveInfo.centriod[j];
                       }

//                  if(PixCurveInfo.last_centriod[j] != 0
//                      && PixCurveInfo.last_centriod[j] < wall_space_index  && ABS(PixCurveInfo.last_centriod[j],Skirting_Line_index) < 200
//                       && ABS(PixCurveInfo.centriod[j], Skirting_Line_index) > 200)
//                  {
//                      LiDARData->change_flag[j] = true;
//                      lidardata.final_index[j] = PixCurveInfo.last_centriod[j];
//                      PixCurveInfo.centriod[j] = PixCurveInfo.last_centriod[j];
//                  }
              }
          }
        }
//        int line_k = 0;
//        if(frame_line == APP_LIDAR_WIDTH_TOTAL-1)
//        {
//            last_centroid = 0;
//            for(int j=APP_LIDAR_WIDTH_TOTAL-1; j >=0;j--)
//            {
//                if(PixCurveInfo.last_centriod[j] != 0)
//                {
//                    if(last_centroid == 0)
//                    {
//                        if(PixCurveInfo.centriod[j] < PixCurveInfo.last_centriod[j])
//                        {
//                          //  centroid_temp = PixCurveInfo.last_centriod[j];
//                           // PixCurveInfo.centriod[j] = PixCurveInfo.last_centriod[j];
//                            LiDARData->change_flag[j] = true;
//                        }
//                    }
//                    else
//                    {
//                        if(ABS(PixCurveInfo.centriod[j],last_centroid) > ABS(PixCurveInfo.last_centriod[j],last_centroid))
//                        {
//                           // centroid_temp = PixCurveInfo.last_centriod[j];
//                           // PixCurveInfo.centriod[j] = PixCurveInfo.last_centriod[j];
//                         //  if(ABS((PixCurveInfo.last_centriod[j] + PixCurveInfo.centriod[j])/2,last_centroid) > 100)
//                           if((PixCurveInfo.last_centriod[j] < last_centroid && line_k == 1) || line_k == 0)
//                           {
//                               LiDARData->change_flag[j] = true;
//                           }
//                        }
//                    }
//                }

//                if(LiDARData->change_flag[j])
//                {
//                    lidardata.final_index[j] = PixCurveInfo.last_centriod[j];
//                }else {
//                    lidardata.final_index[j] = PixCurveInfo.centriod[j];
//                }

//                if( lidardata.final_index[j+6] > lidardata.final_index[j+1] + 200 &&  lidardata.final_index[j+1] != 0)
//                {
//                    line_k = 1;
//                }

//               if(/*ABS(lidardata.final_index[j],(lidardata.index[j-1])) < 200 && */ABS(lidardata.final_index[j],(lidardata.final_index[j+1])) < 200)
//                    last_centroid  =  lidardata.final_index[j];
//            }

//        }

        return;
    }
}


void LidarVideoView::imageInfoPrecess(TRData * pack)
{
    if(pack->pack_id == PACK_GET_VIDEO)
    {
        static uint16_t clos_num = 0;
        static uint8_t clos_flag[360] = {0};
        uint16_t clos = pack->chunk_offset;
        uint16_t data_len = pack->data.size();
        imageWidth = 320;
        //imageHeight = 120;
        imageHeight = 160;
    //        if(clos % 2==0)
    //        {
    //            PixValueType VideoBuffer_Soft_Subsample[120] = {0};
    //            for(uint32_t i = 0;i < data_len; i++)
    //            {
    //                VideoBuffer_Soft_Subsample[i] =  pack->data[i];
    //            }
    //            FilterCentroid(VideoBuffer_Soft_Subsample, clos, &PixCurveInfo, false);
    //        }

        if(clos < imageWidth && clos_flag[clos] == 0)
        {
            clos_num++;
            clos_flag[clos] = 1;
            //memcpy(originImageData + clos * imageWidth, pack->data.data(), data_len);
            for(uint32_t rows = 0;rows < data_len; rows++)
            {
                originImageData[rows * imageWidth + clos] =  pack->data[rows];
                //qDebug() << rows <<":" <<pack->data[rows];
            }


        }
            // qDebug()<<"clo:"<<clos;
        if(clos_num == 80)
        {
            clos_num = 0;
            memset(clos_flag, 0, 360);
            memcpy(tempImageData, originImageData, imageWidth * imageHeight);
            showImage();
            showCurve();
            showImageInfo();
        }
    }
    else if(pack->pack_id == PACK_VIDEO_SIZE)
    {
        imageWidth = *(uint16_t *)pack->data.data();
        imageHeight = *(uint16_t *)(pack->data.data() + 2);
    }
}

uint8_t LidarVideoView::getMiddleValue(uint8_t *start_addr, uint8_t *end_addr)
{
    uint16_t len = end_addr - start_addr;
    if(len <= 1)
    {
        return *start_addr;
    }

    std::vector<uint8_t> data;
    for(uint16_t i = 0; i < len; i++)
    {
        data.push_back(*(uint8_t *)(start_addr + i));
    }
    sort(data.begin(), data.end());
    if(len / 2 == 1)
    {
        return data[len / 2];
    }
    else
    {
        return data[len / 2] + data[len / 2 - 1];
    }
}

void LidarVideoView::showImage()
{
    if(filterMode == 0)
    {
        memcpy(imageData, tempImageData, imageWidth * imageHeight);
        for(int clos=0;clos<imageWidth;clos++)
        {
            if(clos % 2==0)
            {
                PixValueType VideoBuffer_Soft_Subsample[APP_LIDAR_HEIGTH_TOTAL] = {0};
                for(uint32_t i = 0;i < imageHeight; i++)
                {
                    VideoBuffer_Soft_Subsample[i] = imageData[i * imageWidth + clos];
                }
                FilterCentroid(VideoBuffer_Soft_Subsample, clos/2, &PixCurveInfo, false);
            }
        }
//        qDebug()<<"GetCentroid:";
//        for(int i=0;i<160;i++)
//        {
//             qDebug()<<2*i<<":"<<LiDARData->index[i];
//        }
    }
    else if(filterMode == 1)
    {
        for(int i = 0; i < imageHeight; i++)
        {
            for(int j = 0; j < imageWidth; j++)
            {
                uint32_t index = j + i * imageWidth;
                if(j == 0)
                {
                    imageData[index] = getMiddleValue(&tempImageData[index], &tempImageData[index + 3]);
                }
                else if(j == imageWidth - 1)
                {
                    imageData[index] = getMiddleValue(&tempImageData[index - 3], &tempImageData[index]);
                }
                else if(j == imageWidth - 2)
                {
                    imageData[index] = getMiddleValue(&tempImageData[index - 2], &tempImageData[index + 1]);
                }
                else
                {
                    imageData[index] = getMiddleValue(&tempImageData[index - 1], &tempImageData[index + 2]);
                }
            }
        }
    }
    else if(filterMode == 2)
    {
        for(int i = 0; i < imageHeight; i++)
        {
            for(int j = 0; j < imageWidth; j++)
            {
                uint32_t index = j + i * imageWidth;
                if(j == 0)
                {
                    imageData[index] = (tempImageData[index] + tempImageData[index + 1] + tempImageData[index + 2] + tempImageData[index + 3]) / 4;
                }
                else if(j == imageWidth - 1)
                {
                    imageData[index] = (tempImageData[index - 3] + tempImageData[index - 2] + tempImageData[index - 1] + tempImageData[index]) / 4;
                }
                else if(j == imageWidth - 2)
                {
                    imageData[index] = (tempImageData[index - 2] + tempImageData[index - 1] + tempImageData[index] + tempImageData[index + 1]) / 4;
                }
                else
                {
                    imageData[index] = (tempImageData[index - 1] + tempImageData[index] + tempImageData[index + 1] + tempImageData[index + 2]) / 4;
                }
            }
        }
    }

    refLine = algorithm->calRefline(imageData, imageWidth, imageHeight);  //计算参考线

    QImage image(imageData, imageWidth, imageHeight, QImage::Format_Grayscale8);
    image.scaled(ui->imageLabel->size(), Qt::KeepAspectRatio);
    QPixmap pixmap = QPixmap().fromImage(image);

    int x = curveCols;
    int y = imageHeight;
    QPainter painter(&pixmap);
    painter.setPen(QPen(Qt::green, 1));
    painter.drawLine(QPoint(x, 0), QPoint(x, y));

    //画质心点
//    painter.setPen(QPen(Qt::red, 1));
//    for(int i=0;i<APP_LIDAR_WIDTH_TOTAL;i++)
//    {
//        {
//            painter.drawPoint(2*i,LiDARData->index[i]/100);
//        }
//    }

    painter.setPen(QPen(Qt::blue, 1));
    for(int i=0;i<APP_LIDAR_WIDTH_TOTAL;i++)
    {
        {
            painter.drawPoint(2*i,LiDARData->last_index[i]/100);
        }
    }

//    for(int i=0;i<APP_LIDAR_WIDTH_TOTAL;i++)
//    {
//        painter.setPen(QPen(Qt::yellow, 1));
//        if(LiDARData->change_flag[i])
//        {
//            painter.drawPoint(2*i,LiDARData->final_index[i]/100);
//        }
//    }


    ui->imageLabel->setScaledContents(true);
    ui->imageLabel->setPixmap(pixmap);
}

void LidarVideoView::showCurve()
{
    QVector<double> keys, values;
    keys.resize(imageHeight);
    values.resize(imageHeight);

    //原始波形
    for(int i = 0; i < imageHeight; i++)
    {
        keys[i] = i;
        values[i] = tempImageData[curveCols + i * imageWidth];
    }
    ui->pixelCurveWidget->graph(3)->setData(keys, values);

    //实际显示的波形
    for(int i = 0; i < imageHeight; i++)
    {
        keys[i] = i;
        values[i] = imageData[curveCols + i * imageWidth];
    }
    ui->pixelCurveWidget->graph(0)->setData(keys, values);

    if(isShowRefline == true)
    {
        for(int j = 0; j < imageHeight; j++)
        {
            keys[j] = j;
            values[j] = LiDARData->refline[curveCols/2];//refLine[curveCols];
        }
        ui->pixelCurveWidget->graph(1)->setData(keys, values);
        ui->pixelCurveWidget->graph(1)->setVisible(true);
    }
    else
    {
        ui->pixelCurveWidget->graph(1)->setVisible(false);
    }

    if(isShowIndicate == true)
    {
        for(int j = 0; j < imageHeight; j++)
        {
            keys[j] = j;
            values[j] = indecateValue;
        }
        ui->pixelCurveWidget->graph(2)->setData(keys, values);
        ui->pixelCurveWidget->graph(2)->setVisible(true);
    }
    else
    {
        ui->pixelCurveWidget->graph(2)->setVisible(false);
    }
   // qDebug()<<curveCols<<":"<<LiDARData->index[curveCols/2];
    ui->imageInfoTable->setItem(2, 1, new QTableWidgetItem(QString::number(LiDARData->index[curveCols/2])));
    ui->imageInfoTable->setItem(3, 1, new QTableWidgetItem(QString::number(LiDARData->refline[curveCols/2])));
    ui->imageInfoTable->setItem(0, 1, new QTableWidgetItem(QString::number(LiDARData->cols_average[curveCols/2])));
      ui->imageInfoTable->setItem(1, 1, new QTableWidgetItem(QString::number(LiDARData->cols_max_value[curveCols/2])));
    ui->pixelCurveWidget->replot();
}

void LidarVideoView::showImageInfo()
{
    double sum = 0;
    uint8_t  max = 0;
    ImageInfo image_info;
    for(int i = 0; i < imageWidth * imageHeight; i++)
    {
        sum += imageData[i];
        if(imageData[i] > max)
        {
            max = imageData[i];
        }
    }
    image_info.avg_value = sum / imageWidth / imageHeight;
    image_info.max_value = max;
    //    ui->imageInfoTable->setItem(0, 1, new QTableWidgetItem(QString::number(image_info.avg_value)));
    //    ui->imageInfoTable->setItem(1, 1, new QTableWidgetItem(QString::number(image_info.max_value)));
}
void LidarVideoView::updateFilterMode(int mode)
{
    if(filterMode == mode)
    {
        return;
    }
    else
    {
        filterMode = mode;
        showImage();
        showCurve();
        showImageInfo();
    }
}

void LidarVideoView::switchReflineVisible()
{
    if(isShowRefline == false)
    {
        isShowRefline = true;
        ui->switchReflineButton->setText("隐藏参考线");
        ui->switchReflineButton->setStyleSheet("background-color: rgb(128, 50, 50); color: white;");
        showCurve();
    }
    else
    {
        isShowRefline = false;
        ui->switchReflineButton->setText("显示参考线");
        ui->switchReflineButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");
        showCurve();
    }
}

void LidarVideoView::switchIndicateVisible()
{
    if(isShowIndicate == false)
    {
        isShowIndicate = true;
        ui->switchIndicateButton->setText("隐藏指示线");
        ui->switchIndicateButton->setStyleSheet("background-color: rgb(128, 50, 50); color: white;");
        showCurve();
    }
    else
    {
        isShowIndicate = false;
        ui->switchIndicateButton->setText("显示指示线");
        ui->switchIndicateButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");
        showCurve();
    }
}

void LidarVideoView::updateCurveCols()
{
    curveCols = ui->imageColsBox->value();
    showImage();
    showCurve();
}

void LidarVideoView::updateIndicateValue()
{
    indecateValue = ui->indicateValueBox->value();
    showCurve();
}

void LidarVideoView::savaImage()
{
    lidarPkg->sendCmd(1, PACK_STOP, 1u);
    lidarPkg->waitWork(200);
    lidarPkg->sendCmd(2, PACK_STOP, 1u);
    lidarPkg->waitWork(200);
    lidarPkg->sendCmd(4, PACK_STOP, 1u);
    lidarPkg->waitWork(200);
    QString fileName =  QFileDialog::getSaveFileName(this, "保存图像", NULL, "csv(*.csv)");
    if(fileName.isEmpty())
    {
       return;
    }
    else
    {
        QFile file(fileName);
        QTextStream out(&file);

        if(!file.open(QFile::WriteOnly | QFile::Text))
        {
            QMessageBox::critical(this, "write error", "write error");
        }
        else
        {
            for(int i = 0; i < imageWidth * imageHeight; i++)
            {
                out << imageData[i];
                if((i + 1) % imageWidth)
                {
                    out<<",";
                }
                else
                {
                    out<<"\n";
                }
            }
        }
    }
}

void LidarVideoView::loadImage()
{
    loadImagePath =  QFileDialog::getOpenFileName(this, "读取图像", NULL, "(*.csv *.bin)");
    ui->imagePathEdit->setText(loadImagePath);
}

void LidarVideoView::showAndPlayImage()
{
    if(loadImagePath.isEmpty())
    {
        return;
    }

    QString imageFileType = loadImagePath.mid(loadImagePath.length() - 4, 4);

    if(imageFileType == ".csv")
    {
        QFile file(loadImagePath);
        if(!file.open(QFile::ReadOnly | QFile::Text))
        {
            QMessageBox::critical(this, "read error", "read error");
        }
        else
        {
            char *tmp = new char[4096];
            int counter = 0;
            while(!file.atEnd())
            {
                memset(tmp, 0, 4096);
                int len = file.readLine(tmp, 4096);

                if(len > 0)
                {
                    QStringList sl = QString(tmp).split(",");
                    int expectSize = imageWidth * imageHeight;
                    if(counter >= expectSize)
                    {
                        QMessageBox::warning(this, QString("error"), QString("file size too long\nexpect:%1\nactual:%2").arg(expectSize).arg(counter));
                        file.close();
                        return;
                    }

                    for(auto n : sl)
                    {
                        tempImageData[counter++] = n.toInt();
                    }
                }
            }
            delete []tmp;
        }
        file.close();

        showImage();
        showCurve();
        showImageInfo();
    }
    else if(imageFileType == ".bin")
    {
        playFlag = true;

        QByteArray data;
        QFile file(loadImagePath);
        if(!file.open(QFile::ReadOnly))
        {
            QMessageBox::critical(this, "read error", "read error");
            file.close();
        }
        else
        {
            data = file.readAll();
            file.close();
        }

        uint32_t block_size = 20000;
        uint32_t block_num = data.size() / block_size;

        QByteArray temp_data;

        for(uint32_t j = 0; j < block_num; j++)
        {
            temp_data.clear();
            temp_data.resize(20000);
            for(uint32_t k = 0; k < block_size; k++)
            {
                temp_data[k] = data[j * block_size + k];
            }
            if(playFlag)
            {
                lidarPkg->parsePkg(temp_data);
                lidarPkg->waitWork(500);
            }
        }
    }
}

void LidarVideoView::stopShowImage()
{
    playFlag = false;
}

void LidarVideoView::mouseMoveEvent(QMouseEvent *event)
{
    //qDebug("mouseEvent");
    bool isSelect=false;  //是否有数据选中
    QCPGraph *mGraph = ui->pixelCurveWidget->graph(0);
    //得到锚点的像素坐标
    QPointF temp = tracer->position->coords();
    //将像素点转换成qcustomplot中的坐标值，并通过setGraphKey将锚点值设为真实数据值。
    tracer->setGraphKey(ui->pixelCurveWidget->xAxis->pixelToCoord(event->pos().x()));
    //遍历曲线(当前一共4条，原始曲线，滤波后曲线，参考线，指示线)
    for (int i = 0; i < 4; ++i)
    {
        //判断哪一条曲线被选中
        if( ui->pixelCurveWidget->graph(i)->selected())
        {
            //显示锚点
            tracer->setVisible(true);
            mGraph = ui->pixelCurveWidget->graph(i);
            //显示tip框
            QToolTip::showText(event->globalPos(), tr(
                                   "<h4>%L1</h4>"
                                   "<table>"
                                   "<tr>"
                                   "<td><h5>X: %L2</h5></td>" "<td>  ,  </td>" "<td><h5>Y: %L3</h5></td>"
                                   "</tr>"
                                   "</table>").arg(mGraph->name()).arg( QString::number( temp.x(), 'g' ) ).arg( QString::number( temp.y(), 'g' ) ), this, this->rect());
            isSelect=true;
            break;
        }
        else
        {
            //没有曲线被选中，不显示锚点
            tracer->setVisible(false);
        }
    }
    if(isSelect==true)
    {
        //将锚点设置到被选中的曲线上
        tracer->setGraph(mGraph);
        //重绘
        ui->pixelCurveWidget->replot();
        //ui->pixelCurveWidget->rescaleAxes();  //复位坐标轴(测试用)
    }

}
