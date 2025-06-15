#include "lidarcontrol.h"
#include "ui_lidarcontrol.h"

#include <QButtonGroup>
#include <QFileDialog>

#pragma execution_character_set("utf-8")

LidarControl::LidarControl(QWidget *parent, QTRNet *lidarPkg, LidarMapView *lidarMapView) :
    QWidget(parent),
    ui(new Ui::LidarControl)
{
    ui->setupUi(this);

    this->lidarPkg = lidarPkg;
    this->lidarMap = lidarMapView;

    uiInit();
    connectInit();
}

LidarControl::~LidarControl()
{
    delete ui;
}

void LidarControl::connectInit()
{
    connect(modeButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(updateMode(int)));
    connect(numButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(updateNum(int)));

    connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(stopLidar()));
    connect(ui->restartButton, SIGNAL(clicked()), this, SLOT(restartLidar()));
    connect(ui->getLidarInfoButton, SIGNAL(clicked()), this, SLOT(getLidarInfo()));
    connect(ui->setSSLTypepushButton, SIGNAL(clicked()), this, SLOT(setLidarType()));


    connect(ui->switchLaserButton, SIGNAL(clicked()), this, SLOT(switchLaser()));
    connect(ui->setLaserCurrentButton, SIGNAL(clicked()), this, SLOT(setLaserCurrent()));
    connect(ui->setRegButton, SIGNAL(clicked()), this, SLOT(setRegister()));
    connect(ui->autoSetDutyThreshold, SIGNAL(clicked()), this, SLOT(setAutoDutyThreshold()));
    connect(ui->getGroundDisButton, SIGNAL(clicked()), this, SLOT(getGroundDis()));
    connect(ui->setGroundDisButton, SIGNAL(clicked()), this, SLOT(setGroundDis()));
    connect(ui->getDutyThreshold, SIGNAL(clicked()), this, SLOT(getDutyThreshold()));
    connect(ui->setDutyThreshold, SIGNAL(clicked()), this, SLOT(setDutyThreshold()));
    connect(ui->getImageSizeButton, SIGNAL(clicked()), this, SLOT(getImageSize()));
    connect(ui->setImageSizeButton, SIGNAL(clicked()), this, SLOT(setImageSize()));
    connect(ui->getFilterGroupSize, SIGNAL(clicked()), this, SLOT(getFilterGroupSize()));
    connect(ui->setFilterGroupSize, SIGNAL(clicked()), this, SLOT(setFilterGroupSize()));
    connect(ui->setWorkingButton, SIGNAL(clicked()), this, SLOT(setWorking()));
    connect(ui->setSleepButton, SIGNAL(clicked()), this, SLOT(setSleep()));
	connect(ui->calibration_tab, SIGNAL(clicked()), this, SLOT(getCalibrationTab()));
//    connect(ui->getGroundDisButton, SIGNAL(clicked()), this, SLOT(getGroundDistance()));
//    connect(ui->setGroundDisButton, SIGNAL(clicked()), this, SLOT(setGroundDistance()));
    connect(ui->setBrightnessButton, SIGNAL(clicked()), this, SLOT(setBrightness()));
    connect(ui->onlineCalibration, SIGNAL(clicked()), this, SLOT(onlineCalibration()));
    connect(ui->disableOnlineCalibration, SIGNAL(clicked()), this, SLOT(disableOnlineCalibration()));
    connect(ui->enableOnlineCalibration, SIGNAL(clicked()), this, SLOT(enableOnlineCalibration()));
}

void LidarControl::uiInit()
{
    modeButtonGroup = new QButtonGroup(this);
    modeButtonGroup->addButton(ui->modeDistanceButton, 0);
    modeButtonGroup->addButton(ui->modeCentroidButton, 1);
    modeButtonGroup->addButton(ui->modeVideoButton, 2);
    modeButtonGroup->addButton(ui->modeSelftestButton, 3);
    modeButtonGroup->addButton(ui->modeTriggerButton, 4);

    numButtonGroup = new QButtonGroup(this);
    numButtonGroup->addButton(ui->lidarNumOneButton, 1);
    numButtonGroup->addButton(ui->lidarNumTwoButton, 2);
    numButtonGroup->addButton(ui->lidarNumThreeButton, 3);
    ui->lidarNumOneButton->setChecked(true);

    ui->lidarInfoTable->setEditTriggers(QAbstractItemView::NoEditTriggers);         //禁止编辑
    ui->lidarInfoTable->horizontalHeader()->setDefaultSectionSize(70);  //设置默认宽度
    ui->lidarInfoTable->verticalHeader()->setDefaultSectionSize(30);   //设置一行默认高度
    ui->lidarInfoTable->setColumnWidth(0,70);
    ui->lidarInfoTable->setColumnWidth(1,235);

    ui->switchLaserButton->setStyleSheet("background-color: rgb(128, 50, 50); color: white;");
}

/**
 * @brief 清空雷达版本信息的显示数据内容
 */
void LidarControl::clearInfoTable(void)
{
    ui->lidarInfoTable->setItem(0, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(1, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(2, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(3, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(4, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(5, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(6, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(7, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(8, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(9, 1, new QTableWidgetItem(""));
    ui->lidarInfoTable->setItem(10, 1, new QTableWidgetItem(""));

}

void LidarControl::updateMode(int mode)
{
    lidarMode = mode;
    uint8_t send_parbuf[256] = {0};
    uint16_t height = 240;
    uint16_t width = 640;
    uint8_t subsample = 2;

    switch (lidarMode)
    {
        case DISTANCE_MODE:
        {
            lidarPkg->sendCmd(0, PACK_CONFIG_ADDRESS, 1u);
            if(ui->lidarNumOneButton->isChecked()) {
                lidarPkg->sendCmd(1, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(1, PACK_GET_DISTANCE, 1u);
            } else if(ui->lidarNumTwoButton->isChecked()) {
                // 1号
//                lidarPkg->sendCmd(0, PACK_CONFIG_ADDRESS, 1u);
                lidarPkg->sendCmd(1, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(1, PACK_GET_DISTANCE, 1u);

                // 2号
                lidarPkg->sendCmd(2, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(2, PACK_GET_DISTANCE, 1u);
            } else if(ui->lidarNumThreeButton->isChecked()) {
                // 1号
//                lidarPkg->sendCmd(0, PACK_CONFIG_ADDRESS, 1u);
                lidarPkg->sendCmd(1, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(1, PACK_GET_DISTANCE, 1u);

                // 2号
                lidarPkg->sendCmd(2, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(2, PACK_GET_DISTANCE, 1u);

                // 3号
                lidarPkg->sendCmd(4, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(4, PACK_GET_DISTANCE, 1u);
            }

            break;
        }
        case CENTRIOD_MODE:
        {
            lidarPkg->sendCmd(0, PACK_CONFIG_ADDRESS, 1u);
            lidarPkg->sendCmd(1, PACK_GET_CENTRIOD, 1u);
            lidarPkg->sendCmd(1, PACK_AUTO_ADJUST_BRIGHTNESS, 1u);

            break;
        }
        case VIDEO_MODE:
        {
            height = 360;
            width = 320;
//            subsample = 4;

//            memset(send_parbuf, 0, sizeof(send_parbuf));
//            memcpy(send_parbuf, (uint8_t *)&height, 2);
//            memcpy(&send_parbuf[2], (uint8_t *)&width, 2);
//            send_parbuf[4] = subsample;
//            lidarPkg->sendWords(1, PACK_SET_VIDEO, (uint32_t *)send_parbuf, 5, 0);
//            lidarPkg->waitWork(200);

            lidarPkg->sendCmd(1, PACK_VIDEO_SIZE, 1u);  //获取视频大小信息
            lidarPkg->waitWork(200);

            lidarPkg->sendCmd(1, PACK_GET_VIDEO, (int16_t)1);
            lidarPkg->waitWork(200);

//            uint8_t temp_data[4];
//            temp_data[0] = (0x0d0d >> 0) & 0xFF;
//            temp_data[1] = (0x0d0d >> 8) & 0xFF;
//            temp_data[2] = (0x02 >> 0) & 0xFF;
//            temp_data[3] = (0x02 >> 8) & 0xFF;
//            uint32_t send_data = *(uint32_t *)temp_data;
//            lidarPkg->sendCmd(1, PACK_CONFIG_REGISTER, send_data);
//            lidarPkg->waitWork(100);
//            temp_data[0] = (0x0d0e >> 0) & 0xFF;
//            temp_data[1] = (0x0d0e >> 8) & 0xFF;
//            temp_data[2] = (0xd4 >> 0) & 0xFF;
//            temp_data[3] = (0xd4 >> 8) & 0xFF;
//            send_data = *(uint32_t *)temp_data;
//            lidarPkg->sendCmd(1, PACK_CONFIG_REGISTER, send_data);
//            lidarPkg->waitWork(100);
//            temp_data[0] = (0x0d0a >> 0) & 0xFF;
//            temp_data[1] = (0x0d0a >> 8) & 0xFF;
//            temp_data[2] = (0x02 >> 0) & 0xFF;
//            temp_data[3] = (0x02 >> 8) & 0xFF;
//            send_data = *(uint32_t *)temp_data;
//            lidarPkg->sendCmd(1, PACK_CONFIG_REGISTER, send_data);
            break;
        }
        case SELF_TEST_MODE:

            break;
        case TRIG_DISTANCE_MODE:

            if(ui->lidarNumOneButton->isChecked()) {
                lidarPkg->sendCmd(0, PACK_CONFIG_ADDRESS, 1u);
                lidarPkg->sendCmd(1, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(1, PACK_GET_DISTANCE_TRIG_MODE, 1u);
            } else if(ui->lidarNumOneButton->isChecked()) {
                // 1号
//                lidarPkg->sendCmd(0, PACK_CONFIG_ADDRESS, 1u);
                lidarPkg->sendCmd(1, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(1, PACK_GET_DISTANCE_TRIG_MODE, 1u);

                // 2号
                lidarPkg->sendCmd(2, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(2, PACK_GET_DISTANCE_TRIG_MODE, 1u);
            } else if(ui->lidarNumThreeButton->isChecked()) {
                // 1号
                lidarPkg->sendCmd(1, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(1, PACK_GET_DISTANCE_TRIG_MODE, 1u);

                // 2号
                lidarPkg->sendCmd(2, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(2, PACK_GET_DISTANCE_TRIG_MODE, 1u);

                // 3号
                lidarPkg->sendCmd(4, PACK_GET_COE, 1u);

                lidarPkg->waitWork(200);
                if(isCoeReady == false)
                {
                    return;
                }

                lidarPkg->sendCmd(4, PACK_GET_DISTANCE_TRIG_MODE, 1u);
            }
            break;

        default:
            break;
    }
}

void LidarControl::updateNum(int num)
{
    if(lidarNum == num)
    {
        return;
    }
    else
    {
        lidarNum = num;
        emit lidarNumChanged(lidarNum);
    }
}

void LidarControl::stopLidar()
{
    lidarMode = -1;
    lidarPkg->sendCmd(1, PACK_STOP, 1u);
    lidarPkg->waitWork(200);
    lidarPkg->sendCmd(2, PACK_STOP, 1u);
    lidarPkg->waitWork(200);
    lidarPkg->sendCmd(4, PACK_STOP, 1u);
    lidarPkg->waitWork(200);
}

void LidarControl::restartLidar()
{
    lidarMode = -1;
    lidarPkg->sendCmd(4, PACK_RESET_SYSTEM, 1u);
    //lidarPkg->waitWork(400);
    lidarPkg->sendCmd(2, PACK_RESET_SYSTEM, 1u);
    //lidarPkg->waitWork(400);
    lidarPkg->sendCmd(1, PACK_RESET_SYSTEM, 1u);
    //lidarPkg->waitWork(400);
}

void LidarControl::getLidarInfo()
{
    for(int i = 0; i <= 10; i++)
    {
        ui->lidarInfoTable->setItem(i, 1, new QTableWidgetItem(NULL));
    }

    if(ui->lidarNumOneButton->isChecked())
        lidarPkg->sendCmd(1, PACK_VERSION, 1u);
    else if(ui->lidarNumTwoButton->isChecked())
        lidarPkg->sendCmd(2, PACK_VERSION, 2u);
    else
        lidarPkg->sendCmd(3, PACK_VERSION, 3u);
    lidarPkg->waitWork(200);
}

void LidarControl::setLidarType()
{
    uint8_t send_buf[128];
    uint8_t ssl_type = ui->SLLTypelineEdit->text().toUInt();

    if(ssl_type == TYPE_NULL || ssl_type > TYPE_SSL_20L3)
    {
        ui->SLLTypelineEdit->setText("set para err");
        return;
    }

    memset(send_buf, 0, sizeof(send_buf));
    strcpy((char *)send_buf, "set ssl_type");
    memcpy(&send_buf[PACK_STRING_LEN], &ssl_type, 1);
    lidarPkg->sendBytes(1, PACK_SET_SSL_TYPE, (uint8_t *)send_buf, PACK_STRING_LEN + 1, 0);

    if(!lidarPkg->waitAck(1000, PACK_SET_SSL_TYPE))
    {
        qDebug()<<"set PACK_SET_SSL_TYPE err";
    }
    else
    {
         qDebug()<<"set PACK_SET_SSL_TYPE success";
    }
}

void LidarControl::lidarInfoPrecess(TRData *pack)
{
    if(pack->pack_id == PACK_VERSION)
    {
    //        if(pack->data.data()[2] == 0 && pack->data.data()[0] > 0)//适配新旧软件版本规则
    //        {
    //            lidarInfo.firmware_version = QString("v%1.%2.%3").arg(pack->data.data()[3]).arg(pack->data.data()[2]).arg(pack->data.data()[0]);
    //        }
    //        else
        {
             lidarInfo.firmware_version = QString("v%1.%2.%3.%4").arg(pack->data.data()[3]).arg(pack->data.data()[2]).arg(pack->data.data()[1]).arg(pack->data.data()[0]);
        }
        lidarInfo.hardware_version = QString("v%1.%2").arg(pack->data.data()[5]).arg(pack->data.data()[4]);
        if(*(uint16_t *)&pack->data.data()[6] < LIDARPRODUCT.size())
        {
            lidarInfo.type = LIDARPRODUCT[*(uint16_t *)&pack->data.data()[6]];
        }
        else
        {
            lidarInfo.type = "ERROR";
        }
        lidarInfo.product_date = QString("%1-%2-%3").arg(*(uint16_t *)&pack->data.data()[10]).arg(pack->data.data()[9]).arg(pack->data.data()[8]);
        lidarInfo.product_time = QString("%1:%2:%3").arg(pack->data.data()[14]).arg(pack->data.data()[13]).arg(pack->data.data()[12]);
        lidarInfo.sn.clear();
        for(int i = 0; i < 16; i++)
        {
            lidarInfo.sn += pack->data.data()[i + 16];
        }
        lidarInfo.pitch = QString("%1").arg(*(int32_t *)&pack->data.data()[32]);
        lidarInfo.roll = QString("%1").arg(*(int32_t *)&pack->data.data()[36]);
        lidarInfo.yaw = QString("%1").arg(*(int32_t *)&pack->data.data()[40]);
        lidarInfo.mcu_id = QString("%1%2%3").arg(*(uint32_t *)&pack->data.data()[44],8,16,QLatin1Char('0'))
                                                .arg(*(uint32_t *)&pack->data.data()[48],8,16,QLatin1Char('0'))
                                                .arg(*(uint32_t *)&pack->data.data()[52],8,16,QLatin1Char('0'));

        ui->lidarInfoTable->setItem(0, 1, new QTableWidgetItem(lidarInfo.firmware_version));
        ui->lidarInfoTable->setItem(1, 1, new QTableWidgetItem(lidarInfo.hardware_version));
        ui->lidarInfoTable->setItem(2, 1, new QTableWidgetItem(lidarInfo.type));
        ui->lidarInfoTable->setItem(3, 1, new QTableWidgetItem(lidarInfo.product_date));
        ui->lidarInfoTable->setItem(4, 1, new QTableWidgetItem(lidarInfo.product_time));
        ui->lidarInfoTable->setItem(5, 1, new QTableWidgetItem(lidarInfo.sn));
        ui->lidarInfoTable->setItem(6, 1, new QTableWidgetItem(lidarInfo.pitch));
        ui->lidarInfoTable->setItem(7, 1, new QTableWidgetItem(lidarInfo.roll));
        ui->lidarInfoTable->setItem(8, 1, new QTableWidgetItem(lidarInfo.mcu_id));
        lidarInfo.sn32.clear();
        if(pack->data.size() == 88)
        {
            for(int i = 0; i < 32; i++)
            {
                lidarInfo.sn32 += pack->data.data()[i + 56];
            }
        }
        ui->lidarInfoTable->setItem(9, 1, new QTableWidgetItem(lidarInfo.sn32));

        uint8_t ssl_type = pack->data.data()[88];
        if(ssl_type == TYPE_SSL_20L3)
        {
            lidarInfo.ssl_type = "SSL-20L3(1)" ;
        }
        else if(ssl_type == TYPE_SSL_20N3)
        {
            lidarInfo.ssl_type = "SSL-20N3(2)";
        }

        ui->lidarInfoTable->setItem(10, 1, new QTableWidgetItem(lidarInfo.ssl_type));
    }
    else if(pack->pack_id == PACK_GET_LASER_DUTY)
    {
        uint32_t duty = 0;
        memcpy(&duty, pack->data.data(), 4);
        ui->duty_min->setText(QString("%1").arg(duty));
        memcpy(&duty, pack->data.data()+4, 4);
        ui->duty_mid->setText(QString("%1").arg(duty));
        memcpy(&duty, pack->data.data()+8, 4);
        ui->duty_max->setText(QString("%1").arg(duty));
    }
    else if(pack->pack_id == PACK_GET_FILTER_GROUP_SIZE)
    {
        uint32_t filter_group_size = 0;
        memcpy(&filter_group_size, pack->data.data(), 4);
        ui->filter_group_size->setText(QString("%1").arg(filter_group_size));
    }
    else if(pack->pack_id == PACK_VIDEO_SIZE)
    {
        uint16_t image_width = *(uint16_t *)pack->data.data();
        uint16_t image_height = *(uint16_t *)(pack->data.data() + 2);
        ui->imageColsEdit->setText(QString("%1").arg(image_width));
        ui->imageRowsEdit->setText(QString("%1").arg(image_height));
    }
    else if(pack->pack_id == PACK_GET_GROUND_DISTANCE)
    {
        uint32_t ground_distance = *(uint32_t *)pack->data.data();
        ui->groundDisEdit->setText(QString("%1").arg(ground_distance));
    }
}

void LidarControl::updateCoeFlag()
{
    isCoeReady = true;
}

void LidarControl::switchLaser()
{
    if(isLaserOpen == true)
    {
        isLaserOpen = false;
        ui->switchLaserButton->setText("开启激光器");
        ui->switchLaserButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");

        lidarPkg->sendCmd(1, PACK_LASER_TOGGLE, 0u);
        lidarPkg->waitWork(200);
    }
    else
    {
        isLaserOpen = true;
        ui->switchLaserButton->setText("关闭激光器");
        ui->switchLaserButton->setStyleSheet("background-color: rgb(128, 50, 50); color: white;");

        lidarPkg->sendCmd(1, PACK_LASER_TOGGLE, 1u);
        lidarPkg->waitWork(200);
    }
}

void LidarControl::setLaserCurrent()
{
    uint32_t duty = ui->laserCurrentBox->value();
    lidarPkg->sendCmd(1, PACK_LASER_CURRENT, duty);
}

void LidarControl::setRegister()
{
    uint8_t temp_data[4];
    temp_data[0] = (ui->regBox->value() >> 0) & 0xFF;
    temp_data[1] = (ui->regBox->value() >> 8) & 0xFF;
    temp_data[2] = (ui->regValueBox->value() >> 0) & 0xFF;
    temp_data[3] = (ui->regValueBox->value() >> 8) & 0xFF;
    uint32_t send_data = *(uint32_t *)temp_data;
    lidarPkg->sendCmd(1, PACK_CONFIG_REGISTER, send_data);
}

void LidarControl::setAutoDutyThreshold()
{
    char send_parbuf[150] = {0};
    memset(send_parbuf, 0, sizeof(send_parbuf));
    strcpy_s(send_parbuf, "auto laser duty");

    lidarPkg->sendWords(1, PACK_AUTO_LASER_DUTY, (uint32_t *)send_parbuf, PACK_STRING_LEN, 0);
}

void LidarControl::getGroundDis()
{
    lidarPkg->sendCmd(1, PACK_GET_GROUND_DISTANCE, 1);
}

void LidarControl::setGroundDis()
{
    char send_parbuf[150] = {0};

    uint32_t ground_distance = 0;

    ground_distance = ui->groundDisEdit->text().toUInt();

    memset(send_parbuf, 0, sizeof(send_parbuf));
    strcpy_s(send_parbuf, "ground distance");
    memcpy(&send_parbuf[PACK_STRING_LEN], (uint8_t *)&ground_distance, 4);

    lidarPkg->sendWords(1, PACK_SET_GROUND_DISTANCE, (uint32_t *)send_parbuf, PACK_STRING_LEN+4, 0);
}

void LidarControl::getDutyThreshold()
{
    lidarPkg->sendCmd(1, PACK_GET_LASER_DUTY, 1);
}

void LidarControl::setDutyThreshold()
{
    char send_parbuf[150] = {0};

    uint32_t duty[3] = {0};

    duty[0] = ui->duty_min->text().toInt();
    duty[1] = ui->duty_mid->text().toInt();
    duty[2] = ui->duty_max->text().toInt();

    memset(send_parbuf, 0, sizeof(send_parbuf));
    strcpy_s(send_parbuf, "set laser duty");
    memcpy(&send_parbuf[PACK_STRING_LEN], (uint8_t *)duty, 12);

    lidarPkg->sendWords(1, PACK_SET_LASER_DUTY, (uint32_t *)send_parbuf, PACK_STRING_LEN+12, 0);
}

void LidarControl::getImageSize()
{
    lidarPkg->sendCmd(1, PACK_VIDEO_SIZE, 1u);
    lidarPkg->waitWork(200);
}

void LidarControl::setImageSize()
{
    uint8_t send_parbuf[5];
    uint16_t height = 320;
    uint16_t width = 320;
    uint8_t subsample = 2;

    memset(send_parbuf, 0, sizeof(send_parbuf));
    memcpy(send_parbuf, (uint8_t *)&height, 2);
    memcpy(&send_parbuf[2], (uint8_t *)&width, 2);
    send_parbuf[4] = subsample;
    lidarPkg->sendWords(1, PACK_SET_VIDEO, (uint32_t *)send_parbuf, 5, 0);
    lidarPkg->waitWork(200);
}

void LidarControl::getFilterGroupSize()
{
    lidarPkg->sendCmd(1, PACK_GET_FILTER_GROUP_SIZE, 1u);
}

void LidarControl::setFilterGroupSize()
{
    char send_parbuf[150] = {0};

    uint32_t filter_group_size = 0;

    filter_group_size = ui->filter_group_size->text().toInt();

    memset(send_parbuf, 0, sizeof(send_parbuf));
    strcpy_s(send_parbuf, "set filter size");
    memcpy(&send_parbuf[PACK_STRING_LEN], (uint8_t *)&filter_group_size, 4);

    lidarPkg->sendWords(1, PACK_SET_FILTER_GROUP_SIZE, (uint32_t *)send_parbuf, PACK_STRING_LEN+4, 0);
}

void LidarControl::setWorking()
{
    uint8_t send_parbuf[256] = {0};
    uint32_t trans_mode = 1;

    strcpy((char *)send_parbuf, "trans mode");
    memcpy(&send_parbuf[16], (uint8_t *)&trans_mode, 4);

    lidarPkg->sendWords(1, PACK_TRANSMISSION_MODE, (uint32_t *)send_parbuf, 4 + 16, 0);
}

void LidarControl::setSleep()
{
    uint8_t send_parbuf[256] = {0};
    uint32_t trans_mode = 0;

    strcpy((char *)send_parbuf, "trans mode");
    memcpy(&send_parbuf[16], (uint8_t *)&trans_mode, 4);

    lidarPkg->sendWords(1, PACK_TRANSMISSION_MODE, (uint32_t *)send_parbuf, 4 + 16, 0);
}

void LidarControl::getGroundDistance()
{
    lidarPkg->sendCmd(1, PACK_GET_GROUND_DISTANCE, 1u);
    lidarPkg->waitWork(200);
}

void LidarControl::setGroundDistance()
{
    uint8_t send_parbuf[256] = {0};
    uint32_t ground_distance = ui->groundDisEdit->text().toUInt();

    strcpy((char *)send_parbuf, "ground distance");
    memcpy(&send_parbuf[16], (uint8_t *)&ground_distance, 4);
    lidarPkg->sendWords(1, PACK_SET_GROUND_DISTANCE, (uint32_t *)send_parbuf, 4 + 16, 0);

    ui->groundDisEdit->setText("0");
}

void LidarControl::setBrightness()
{

}

void LidarControl::getCalibrationTab()
{
    QString fileName =  "SSL-20P_dis_cal_table.csv";
    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {

    }

    QTextStream out(&file);
    uint16_t datata[35][160] = {0};

    lidarPkg->sendCmd(1, PACK_GET_TAB, 1);

    for(uint16_t col = 0; col < 160; col++)
    {
        if(!lidarPkg->waitAck(3000,PACK_GET_TAB))
        {
            file.close();
            return;
            qDebug() << "***" << col;
        }
        else
        {
            qDebug() << "&&&" << col;
        }

        for(uint16_t row = 0; row < 35; row++)
        {
            datata[row][col] = lidarPkg->data[row];
        }
    }

    uint16_t distance_tab[35] = {
        60,80,100,120,140,160,180,200,220,240,
        260,280,300,320,350,400,450,500,550,600,
        650,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,1980
    };

    for(int j = 0; j < 35; j++)
    {
        out << distance_tab[j] << ",";
        for(int i = 0; i < 160; i++)
        {
            out << datata[j][i] << ",";
        }
        out << "\n";
    }

    file.close();
}

//cv::Mat polyfit(QVector<QPoint>& in_point, int n)
//{
//    int size = in_point.size();
//    //所求未知数个数
//    int x_num = n + 1;
//    //构造矩阵U和Y
//    cv::Mat mat_u(size, x_num, CV_64F);
//    cv::Mat mat_y(size, 1, CV_64F);

//    for (int i = 0; i < mat_u.rows; ++i)
//    {
//        for (int j = 0; j < mat_u.cols; ++j)
//        {
//            mat_u.at<double>(i, j) = pow(in_point[i].x(), j);
//        }
//    }

//    for (int i = 0; i < mat_y.rows; ++i)
//    {
//        mat_y.at<double>(i, 0) = in_point[i].y();
//    }

//    //矩阵运算，获得系数矩阵K
//    cv::Mat mat_k(x_num, 1, CV_64F);
//    mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
//    return mat_k;
//}

//在线标定函数
void LidarControl::onlineCalibration()
{
    if(lidarMap == nullptr)
    {
        return;
    }
    if(lidarMap->map.size() == 0)
    {
        return;
    }

    //获取当前真实距离，对应客户端的地面距离
    int real_distance = ui->realDistanceEdit->text().toUInt();
    //存储当前距离下每个点相对于真实距离的误差
    QVector<QPoint> offset;
    bool use_last = false;

    //假定从30mm-real_distance，误差增长为m次曲线，且30mm处误差为0
    int m = 2;
    QVector<double> offset_coe;
    for (int i = 0; i < 160; i++)
    {
        if(lidarMap->map[i].distance != 0)
        {
            use_last = true;
        }
        else if(use_last)
        {
            lidarMap->map[i].distance = lidarMap->map[i-1].distance;
        }
        else
        {
            for(int j = 0; j < 160; j++)
            {
                if(lidarMap->map[j].distance != 0)
                {
                    lidarMap->map[i].distance = lidarMap->map[j].distance;
                    use_last = true;
                }
            }
        }
        double dis_source = lidarMap->map[i].distance*cos(lidarMap->map[i].angle* 3.1415926 / 180.0);
        double coe_k = (real_distance - dis_source)/pow((real_distance - 30),m); //计算出系数k
        //qDebug() << "i: " << i << "  distance: " << dis_source << "  k: " << coe_k;
        offset_coe.append(coe_k);
    }
    lidarMap->isUseMapOffset(true);
    lidarMap->setMapOffset(offset_coe, real_distance, m);
}

//关闭在线标定
void LidarControl::disableOnlineCalibration()
{
    lidarMap->isUseMapOffset(false);
}
//开启在线标定
void LidarControl::enableOnlineCalibration()
{
    lidarMap->isUseMapOffset(true);
}

void LidarControl::on_sync_Button_clicked()
{
    lidarPkg->sendCmd(1, PACK_SYNC, 0u);
    lidarPkg->waitWork(200);
}


void LidarControl::on_addr_Button_clicked()
{
    lidarPkg->sendCmd(0, PACK_CONFIG_ADDRESS, 1u);
}



