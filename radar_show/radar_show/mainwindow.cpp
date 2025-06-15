#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QMessageBox>
#include <QSerialPortInfo>
#include <QHBoxLayout>
#include <QSettings>
#include <QDesktopWidget>
#include <QShortcut>
#include "lidarmapview.h"

#pragma execution_character_set("utf-8")

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPort(new QSerialPort(this))
    , status(new QLabel(this))
    , lidarPkg(new QTRNet(serialPort))
    , lidarmapview1(new LidarMapView(this))
{
    ui->setupUi(this);

    lidarmapview1->setPkg(lidarPkg);
    /* 刷新端口用定时器 */
    refreshTimer = new QTimer();
    refreshTimer->start(100);

    layoutInit();
    statusBarInit();
    toolBarInit();
//    setShortcut();
    connectInit();
//    recoverCustom();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::layoutInit()
{
    //菜单栏背景色设置
    ui->menubar->setStyleSheet("QMenuBar:item{background-color:rgb(200, 200, 200);}"
                             "QMenuBar{background-color:rgb(200, 200, 200);}");
    // 在界面初始化时
   // RadarWidget *radarWidget = new RadarWidget(this);
    vLayout = new QVBoxLayout(this->centralWidget());
//    vLayout->layout()->addWidget(radarWidget);
    vLayout->layout()->addWidget(lidarmapview1);
//    hLayout = new QHBoxLayout();
//    lidarMapLayout = new QVBoxLayout();
//    vLayout->addLayout(hLayout, 12);
//    hLayout->addLayout(lidarMapLayout, 1);
}

void MainWindow::statusBarInit()
{
    ui->statusbar->addWidget(status);
    ui->statusbar->addPermanentWidget(new QLabel("毫米波雷达"));
    ui->statusbar->addPermanentWidget(new QLabel(" "));
    ui->statusbar->setFixedHeight(25);
    ui->statusbar->setStyleSheet("QStatusBar:item{background-color:rgb(210, 210, 210);}"
                                 "QStatusBar{background-color:rgb(210, 210, 210);}");
    ui->statusbar->setSizeGripEnabled(false);
}

void MainWindow::toolBarInit()
{
    ui->toolBar->setMaximumHeight(30);
    ui->toolBar->setObjectName("ToolBar");

    ui->toolBar->addWidget(new QLabel(tr("Lidar: ")));
    lidarSelectBox = new QComboBox();
    foreach (QString lidar, LIDARPRODUCT)
    {
        lidarSelectBox->addItem(lidar);
    }
    ui->toolBar->addWidget(lidarSelectBox);

    ui->toolBar->addWidget(new QLabel(tr("  ")));
    ui->toolBar->addWidget(new QLabel(tr("COM: ")));
    comBox = new QComboBox();
    ui->toolBar->addWidget(comBox);

    // 新增：波特率设置
    ui->toolBar->addWidget(new QLabel(tr("  ")));
    ui->toolBar->addWidget(new QLabel(tr("波特率: "))); // 波特率标签
    baudRateComboBox = new QComboBox(); // 创建波特率下拉框
    baudRateComboBox->addItems({
        "9600",
        "19200",
        "38400",
        "57600",
        "115200",
        "380400",
        "921600",
    });
    ui->toolBar->addWidget(baudRateComboBox); // 添加到工具栏

    ui->toolBar->addWidget(new QLabel(tr("  ")));
    portSwitchButton = new QPushButton("打开端口");
    portSwitchButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");
    ui->toolBar->addWidget(portSwitchButton);

    ui->toolBar->addWidget(new QLabel(tr("  ")));
//    jigPageButton = new QPushButton("治具控制");
//    ui->toolBar->addWidget(jigPageButton);

//    // 方法1：使用setVisible
//    jigPageButton->setVisible(false);


    // 设置初始项为特定的字符串
    int index = lidarSelectBox->findText(lidarType);
    if (index != -1) { // 如果找到匹配的项
        lidarSelectBox->setCurrentIndex(index);
    }

    // 设置初始项为特定的字符串
    index = baudRateComboBox->findText(baudRate);
    if (index != -1) { // 如果找到匹配的项
        baudRateComboBox->setCurrentIndex(index);
    }

}

void MainWindow::connectInit()
{
    connect(refreshTimer,            SIGNAL(timeout()),                           this,    SLOT(refreshCom()));
    connect(portSwitchButton,        SIGNAL(clicked()),                           this,    SLOT(switchSeiralPort()));

    connect(lidarSelectBox,          SIGNAL(currentIndexChanged(QString)),        this,    SLOT(lidarTypeChanged(QString)));
    connect(serialPort,              SIGNAL(error(QSerialPort::SerialPortError)), this,    SLOT(handleError(QSerialPort::SerialPortError)));

//    connect(lidarPkg, SIGNAL(lidarDataReady(TRData*)), lidarvideoview, SLOT(imageInfoPrecess(TRData*)));
//    connect(lidarPkg, SIGNAL(lidarDataReady(TRData*)), lidarcontrol, SLOT(lidarInfoPrecess(TRData *)));
//    connect(lidarPkg, SIGNAL(coeReady()),              lidarcontrol, SLOT(updateCoeFlag()));
    connect(lidarPkg, SIGNAL(mapReady1(const QList<MapData>)), lidarmapview1, SLOT(updateMap(const QList<MapData>)));
//    connect(lidarPkg, SIGNAL(mapReady2(const QList<MapData>)), lidarmapview2, SLOT(updateMap(const QList<MapData>)));
//    connect(lidarPkg, SIGNAL(mapReady3(const QList<MapData>)), lidarmapview3, SLOT(updateMap(const QList<MapData>)));

//    connect(lidarcontrol, SIGNAL(lidarNumChanged(int)), this, SLOT(updateLidarMapUi(int)));

//    connect(jigPageButton,        SIGNAL(clicked()),                           this,    SLOT(openJigPage()));
    connect(this->lidarPkg ,SIGNAL(paraReady(int, int, int, int, double, double, double, double, int, int)), lidarmapview1, SLOT(updataPara(int, int, int, int, double, double, double, double, int, int)));
}


void MainWindow::refreshCom()
{
    static int port_last_num = 0;

    /* 串口未连接时刷新可用端口显示 */
    if(serialPort->isOpen() == false)
    {
        QStringList newPortStringList;
        for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts())
        {
            newPortStringList += info.portName();
        }

        /* 更新串口号 */
        if(newPortStringList.size() != port_last_num)
        {
            port_last_num = newPortStringList.size();
            comBox->clear();
            comBox->addItems(newPortStringList);
        }
    }
}

void MainWindow::switchSeiralPort(void)
{
    if(!serialPort->isOpen())
    {
        openSerialPort();
    }
    else
    {
        closeSerialPort();
    }
}

void MainWindow::openSerialPort(void)
{
    //readLidarData();
    if(serialPort->isOpen())
    {
        disconnect(serialPort, SIGNAL(readyRead()), this, SLOT(readLidarData()));
        serialPort->close();
    }

    serialPort->setPortName(comBox->currentText());
    //serialPort->setBaudRate(921600);
    serialPort->setBaudRate(baudRateComboBox->currentText().toInt());
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    QPalette pS;
    pS.setColor(QPalette::WindowText,Qt::darkBlue);
    status->setPalette(pS);

    if (serialPort->open(QIODevice::ReadWrite))
    {
        status->setText(tr("Connected to %1 ")
                        .arg(comBox->currentText()));

        serialPort->setDataTerminalReady(true);
        connect(serialPort, SIGNAL(readyRead()), this, SLOT(readLidarData()));

        portSwitchButton->setText("关闭端口");
        portSwitchButton->setStyleSheet("background-color: rgb(128, 50, 50); color: white;");
        comBox->setDisabled(true);
        lidarSelectBox->setDisabled(true);
    }
    else
    {
        QMessageBox::critical(this, tr("Error"), serialPort->errorString());
        status->setText(tr("Open error"));

        portSwitchButton->setText("打开端口");
        portSwitchButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");
        comBox->setDisabled(false);
        lidarSelectBox->setDisabled(false);
    }
}

void MainWindow::closeSerialPort()
{
    if(serialPort->isOpen())
        serialPort->close();
    disconnect(serialPort, SIGNAL(readyRead()), this, SLOT(readLidarData()));
    QPalette pS;
    pS.setColor(QPalette::WindowText,Qt::darkRed);
    status->setPalette(pS);
    status->setText(tr("Disconnected"));

    portSwitchButton->setText("打开端口");
    portSwitchButton->setStyleSheet("background-color: rgb(0, 128, 128); color: white;");
    comBox->setDisabled(false);
    lidarSelectBox->setDisabled(false);
}

void MainWindow::lidarTypeChanged(QString lidar)
{
    lidarType = lidar;
 //   readLidarData();
//    lidarPkg->lidarType = lidar;
//    sw->lidarType = lidar;
}

void MainWindow::readLidarData()
{
    QByteArray dataTemp;
    dataTemp = serialPort->readAll();
    // 打印出接收到的数据
    lidarPkg->parsePkg(dataTemp);

//    QByteArray dataTemp;
//    dataTemp.resize(32); // 数据长度
//    dataTemp[0] = 0xAA;
//    dataTemp[1] = 0xAA;
//    dataTemp[2] = 0xAA;
//    dataTemp[3] = 0xAA;
//    dataTemp[4] = 0x01;
//    dataTemp[5] = 0x02;
//    dataTemp[6] = 0x00;
//    dataTemp[7] = 0x00;
//    dataTemp[8] = 0x18;
//    dataTemp[9] = 0x00;
//    dataTemp[10] = 0xbc;
//    dataTemp[11] = 0x25;
//    dataTemp[12] = 0x00;
//    dataTemp[13] = 0x00;
//    dataTemp[14] = 0x00;
//    dataTemp[15] = 0x00;
//    dataTemp[16] = 0x58;
//    dataTemp[17] = 0x37;
//    dataTemp[18] = 0x00;
//    dataTemp[19] = 0x00;
//    dataTemp[20] = 0x00;
//    dataTemp[21] = 0x00;
//    dataTemp[22] = 0x70;
//    dataTemp[23] = 0x46;
//    dataTemp[24] = 0x00;
//    dataTemp[25] = 0x00;
//    dataTemp[26] = 0x00;
//    dataTemp[27] = 0x00;
//    dataTemp[28] = 0xa0;
//    dataTemp[29] = 0x64;
//    dataTemp[30] = 0x00;
//    dataTemp[31] = 0x00;
//    dataTemp[32] = 0x00;
//    dataTemp[33] = 0x00;
//    dataTemp[34] = 0x45;

//    // 打印 QByteArray 的内容
//    qDebug() << "Data in hex:" << dataTemp.toHex();
//    // 解析数据包
//   lidarPkg->parsePkg(dataTemp);

}

void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if(error == QSerialPort::ResourceError)
    {
        closeSerialPort();

        QPalette pS;
        pS.setColor(QPalette::WindowText,Qt::darkRed);
        status->setPalette(pS);

        status->setText(QStringLiteral("Disconnected"));
    }
}
