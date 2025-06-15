#include "lidarupgrade.h"
#include "ui_lidarupgrade.h"

#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include <QProgressDialog>

LidarUpgrade::LidarUpgrade(QWidget *parent, QTRNet *lidarPkg) :
    QWidget(parent),
    ui(new Ui::LidarUpgrade),
    encrypt(new Encrypt(this))
{
    ui->setupUi(this);
    this->lidarPkg = lidarPkg;

    connectInit();
}

LidarUpgrade::~LidarUpgrade()
{
    delete ui;
}

void LidarUpgrade::connectInit()
{
    connect(ui->unencryptedImportButton, SIGNAL(clicked()), this, SLOT(unencryptedImportFirmware()));
    connect(ui->encryptButton, SIGNAL(clicked()), this, SLOT(encrpytFirmware()));
    connect(ui->unencryptedDownloadButton, SIGNAL(clicked()), this, SLOT(unencryptedDownloadFirmware()));
    connect(ui->encryptedImportButton, SIGNAL(clicked()), this, SLOT(encryptedImportFirmware()));
    connect(ui->decyptButton, SIGNAL(clicked()), this, SLOT(decyptFirmware()));
    connect(ui->encryptedDownloadButton, SIGNAL(clicked()), this, SLOT(encryptedDownloadFirmware()));
    connect(ui->encryptedDownloadButton_1000, SIGNAL(clicked()), this, SLOT(encryptedDownloadFirmware_1000()));
}

//导入非加密固件
void LidarUpgrade::unencryptedImportFirmware()
{
    unencryptFileName = QFileDialog::getOpenFileName(this, "Please choose a bin file", "", "BIN Files(*.bin);;All(*.*)");    //选择路径
    if(unencryptFileName.length() == 0)
    {
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files."));
        return;
    }

    ui->unencryptedPathLineEdit->setText(unencryptFileName);
    unencryptInFile = new QFile(unencryptFileName);
    QFileInfo fileInfo = QFileInfo(unencryptFileName);

    if(fileInfo.suffix() == "bin")
    {
        unencryptFileName.remove(".bin");
        unencryptOutFile = new QFile(unencryptFileName + ".ifw");
    }
}

//导入加密固件
void LidarUpgrade::encryptedImportFirmware()
{
    encryptFileName = QFileDialog::getOpenFileName(this, "Please choose a ifw file", "", "IFW Files(*.ifw);;All(*.*)");    //选择路径
    if(encryptFileName.length() == 0)
    {
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files."));
        return;
    }

    ui->encryptedPathLineEdit->setText(encryptFileName);
    encryptInFile = new QFile(encryptFileName);
    QFileInfo fileInfo = QFileInfo(encryptFileName);

    if(fileInfo.suffix() == "ifw")
    {
        encryptFileName.remove(".ifw");
        encryptOutFile = new QFile(encryptFileName + ".bin");
    }
}

//固件加密
void LidarUpgrade::encrpytFirmware()
{
    if(!unencryptInFile->open(QFile::ReadOnly))
    {
        QMessageBox::warning(this, tr("Warning"), tr("Can't open the infile."));
        return;
    }

    unencryptFileData = unencryptInFile->read(unencryptInFile->bytesAvailable());
    unencryptInFile->close();

    if(!unencryptOutFile->open(QFile::WriteOnly | QFile::Truncate))   //不存在则创建文件
    {
        QMessageBox::warning(this, tr("Warning"), tr("Can't open the outfile."));
        return;
    }

    if(lidarPkg->lidarType == "SSL-20LB2")
    {
        encrypt->encryptFirmware(unencryptFileData, AES128_key1, 16, 64, 0x66, unencryptOutFile);//20LB2 I2C地址为0x66
        QMessageBox::warning(this, tr("Finish"), tr("加密完成"));
        return;
    }
    else if(lidarPkg->lidarType == "SSL-20NB2" )
    {
        encrypt->encryptFirmware(unencryptFileData, AES128_key1, 16, 64, 0x68, unencryptOutFile);//20NB2 I2C地址为0x68
        QMessageBox::warning(this, tr("Finish"), tr("加密完成"));
        return;
    }
    else if(lidarPkg->lidarType == "SSL-20L2")
    {
        encrypt->encryptFirmware(unencryptFileData, AES128_key1, 16, 64, 0x76, unencryptOutFile);//20L2 I2C地址为0x76
        QMessageBox::warning(this, tr("Finish"), tr("加密完成"));
        return;
    }
    else if(lidarPkg->lidarType == "SSL-20L3" || lidarPkg->lidarType == "SSL-20L3F7" || lidarPkg->lidarType == "SSL-20L3B2")
    {
        encrypt->encryptFirmware(unencryptFileData, AES128_key1, 16, 64, 0x82, unencryptOutFile);//20L2 I2C地址为0x76
        QMessageBox::warning(this, tr("Finish"), tr("加密完成"));
        return;
    }
    else if(lidarPkg->lidarType == "SSL-20N2")
    {
        encrypt->encryptFirmware(unencryptFileData, AES128_key1, 16, 64, 0x78, unencryptOutFile);//20NB2 I2C地址为0x78
        QMessageBox::warning(this, tr("Finish"), tr("加密完成"));
        return;
    }
    else if(lidarPkg->lidarType == "SSL-20PL" || lidarPkg->lidarType == "SSL-20P" || lidarPkg->lidarType == "SSL-20PT1")
    {
        encrypt->encryptFirmware(unencryptFileData, AES128_key2, 16, 0, 0,unencryptOutFile);
        QMessageBox::warning(this, tr("Finish"), tr("加密完成"));
        return;
    }
    else
    {
        encrypt->encryptFirmware(unencryptFileData, AES128_key1, 16, 0, 0,unencryptOutFile);
        QMessageBox::warning(this, tr("Finish"), tr("加密完成"));
        return;
    }
}

void LidarUpgrade::unencryptedDownloadFirmware()
{
    if(lidarPkg->lidarType != "SSL-20PL" && lidarPkg->lidarType != "SSL-20P" && lidarPkg->lidarType != "SSL-20PT1")
    {
        if(!downloadMass1(PACK_UPGRADE_START, "start download", PACK_UPGRADE_TRANSMIT_DATA, "downloading", PACK_UPGRADE_COMPLETE, "complete", 64, unencryptInFile))
        {
            QMessageBox::warning(this, tr("ERROR"), tr("upgrade failure"));
        }
    }
    else
    {
        if(!downloadMass2(PACK_UPGRADE_START, "start download", PACK_UPGRADE_TRANSMIT_DATA, "downloading", PACK_UPGRADE_COMPLETE, "complete", 256, unencryptInFile))
        {
            QMessageBox::warning(this, tr("ERROR"), tr("upgrade failure"));
        }
    }
}

//固件解密
void LidarUpgrade::decyptFirmware()
{
    if(!encryptInFile->open(QFile::ReadOnly))
    {
        QMessageBox::warning(this, tr("Warning"), tr("Can't open the infile."));
        return;
    }

    encryptFileData = encryptInFile->read(encryptInFile->bytesAvailable());
    encryptInFile->close();

    if(!encryptOutFile->open(QFile::WriteOnly | QFile::Truncate))   //不存在则创建文件
    {
        QMessageBox::warning(this, tr("Warning"), tr("Can't open the outfile."));
        return;
    }

    if(lidarPkg->lidarType != "SSL-20PL" && lidarPkg->lidarType != "SSL-20P" && lidarPkg->lidarType != "SSL-20PT1")
    {
        encrypt->decryptFirmware(encryptFileData, AES128_key1, 16, 0, encryptOutFile);
        QMessageBox::warning(this, tr("Finish"), tr("解密完成"));
        return;
    }
    else
    {
        encrypt->decryptFirmware(encryptFileData, AES128_key2, 16, 0, encryptOutFile);
        QMessageBox::warning(this, tr("Finish"), tr("解密完成"));
        return;
    }
}

void LidarUpgrade::encryptedDownloadFirmware()
{
    if(lidarPkg->lidarType != "SSL-20PL" && lidarPkg->lidarType != "SSL-20P" && lidarPkg->lidarType != "SSL-20PT1")
    {
        if(!downloadMass1(PACK_UPGRADE_START, "start download", PACK_UPGRADE_TRANSMIT_DATA, "encrypting", PACK_UPGRADE_COMPLETE, "complete", 64, encryptInFile))
        {
            QMessageBox::warning(this, tr("ERROR"), tr("upgrade failure"));
        }
    }
    else
    {
        for(int i = 0; i < 1000; i++)
        {
            if(!downloadMass2(PACK_UPGRADE_START, "start download", PACK_UPGRADE_TRANSMIT_DATA, "encrypting", PACK_UPGRADE_COMPLETE, "complete", 256, encryptInFile))
            {
                QMessageBox::warning(this, tr("ERROR"), tr("upgrade failure: ")+QString::number(i));
            }
            lidarPkg->waitWork(1000);
        }
    }
}

void LidarUpgrade::encryptedDownloadFirmware_1000()
{
    QProgressDialog progressDialog;
    progressDialog.setWindowTitle("download");//对话框标题
    progressDialog.setLabelText("Now downloading");//对话框文本
    progressDialog.setCancelButtonText("cancel");//设置取消按钮
    progressDialog.setRange(1,1000);//设置进度条范围
    progressDialog.setModal(true);//以模态方式弹出对话框
    progressDialog.showNormal();

    if(lidarPkg->lidarType != "SSL-20PL" && lidarPkg->lidarType != "SSL-20P" && lidarPkg->lidarType != "SSL-20PT1")
    {
        for(int i = 1; i <= 1000; i++)
        {
            if(!downloadMass1(PACK_UPGRADE_START, "start download", PACK_UPGRADE_TRANSMIT_DATA, "encrypting", PACK_UPGRADE_COMPLETE, "complete", 64, encryptInFile))
            {
                QMessageBox::warning(this, tr("ERROR"), tr("upgrade failure: ")+QString::number(i));
            }

            progressDialog.setValue(i);//设置进度条的值
            progressDialog.setLabelText("Now downloading: " + QString::number(i));//对话框文本
            if(progressDialog.wasCanceled())//取消为true
            {
                progressDialog.reset();
                break;
            }

            if(i == 1000)
            {
                QMessageBox::warning(this, tr("complete"), tr("upgrade complete: 1000"));
            }
            lidarPkg->waitWork(1000);
        }
    }
    else
    {
        for(int i = 1; i <= 1000; i++)
        {
            if(!downloadMass2(PACK_UPGRADE_START, "start download", PACK_UPGRADE_TRANSMIT_DATA, "encrypting", PACK_UPGRADE_COMPLETE, "complete", 256, encryptInFile))
            {
                QMessageBox::warning(this, tr("ERROR"), tr("upgrade failure: ")+QString::number(i));
            }

            progressDialog.setValue(i);//设置进度条的值
            progressDialog.setLabelText("Now downloading: " + QString::number(i));//对话框文本
            if(progressDialog.wasCanceled())//取消为true
            {
                progressDialog.reset();
                break;
            }

            if(i == 1000)
            {
                QMessageBox::warning(this, tr("complete"), tr("upgrade complete: 1000"));
            }
            lidarPkg->waitWork(1000);
        }
    }
}

bool LidarUpgrade::downloadMass1(uint8_t startCmd, const char *startStr, uint8_t sendCmd, const char *sendStr, uint8_t completeCmd, const char *completeStr, int block_size, QFile *firmware_file)
{
    static uint8_t addr = 1;

    addr = ui->addrSpinBox->value();

    QByteArray data;
    if(firmware_file->open(QFile::ReadOnly))
    {
        data = firmware_file->readAll();
        firmware_file->close();
    }
    else
    {
        firmware_file->close();
        return false;
    }

    std::vector<uint8_t> firmware;
    firmware.clear();
    firmware.resize(data.size());
    int k = 0;
    for(auto n : data)
    {
        firmware[k++] = n;
    }

    int mod = firmware.size() % 4;
    for(int i = 0 ; i < mod ;i++)
    {
        firmware.push_back(0xff);
    }

    uint8_t send_buf[128];

    if(sizeof(send_buf) < block_size + PACK_STRING_LEN)
    {
        return false;
    }

    memset(send_buf, 0, sizeof(send_buf));
    if(startStr != NULL && strlen(startStr) < PACK_STRING_LEN)
    {
        strcpy((char *)send_buf, startStr);
        lidarPkg->sendWords(addr, startCmd, (uint32_t *)send_buf, PACK_STRING_LEN, 0);
    }
    else
    {
        return false;
    }

    lidarPkg->waitWork(1500);    //等待flash擦除完毕

    if(!lidarPkg->waitAck(2000, startCmd))
    {
        return false;
    }

    uint32_t *p32 = (uint32_t *)firmware.data();
    int split = firmware.size() / block_size;    //分割成小块发送，MCU端的缓存区较小。
    int split_mod = firmware.size() % block_size;

    ui->downloadProgressBar->setRange(0, split + ((split_mod == 0) ? 0 : 1));

    int i;
    for(i = 0; i < split; i++)
    {
        memset(send_buf, 0, sizeof(send_buf));
        if(sendStr != NULL && strlen(sendStr) < PACK_STRING_LEN)
        {
            strcpy((char *)send_buf, sendStr);
            memcpy(&send_buf[PACK_STRING_LEN], (uint8_t *)p32, block_size);
            lidarPkg->sendWords(addr, sendCmd, (uint32_t *)send_buf, block_size + PACK_STRING_LEN, i * block_size);
        }
        else
        {
            return false;
        }

        p32 += block_size / 4;
        if(!lidarPkg->waitAck(1000, sendCmd))
        {
            return false;
        }
        ui->downloadProgressBar->setValue(i);
    }

    if(split_mod != 0)
    {
        memset(send_buf, 0, sizeof(send_buf));
        if(sendStr != NULL && strlen(sendStr) < PACK_STRING_LEN)
        {
            strcpy((char *)send_buf,sendStr);
            memcpy(&send_buf[PACK_STRING_LEN], (uint8_t *)p32, split_mod);
            lidarPkg->sendWords(addr, sendCmd, (uint32_t *)send_buf, split_mod + PACK_STRING_LEN, i * block_size);
        }
        else
        {
            return false;
        }

        if(!lidarPkg->waitAck(1000, sendCmd))
        {
            return false;
        }
    }

    //根据不同的文件类型，可写入不同的flag
    if(completeCmd == PACK_UPGRADE_COMPLETE)
    {
        memset(send_buf, 0, sizeof(send_buf));
        if(completeStr != NULL && strlen(completeStr) < PACK_STRING_LEN)
        {
            uint8_t *p8 = (uint8_t *)firmware.data();
            uint8_t all_data_sum = 0;
            for(uint32_t i = 0; i < firmware.size(); i++)
            {
                all_data_sum += *p8;
                p8++;
            }
            memset(send_buf, 0, sizeof(send_buf));
            strcpy((char *)send_buf, completeStr);
            memcpy(&send_buf[PACK_STRING_LEN], (uint8_t *)&all_data_sum, 1);
            lidarPkg->sendWords(addr, completeCmd, (uint32_t *)send_buf, 1 + PACK_STRING_LEN, 0);
        }
        else
        {
            return false;
        }
    }
    if(!lidarPkg->waitAck(2000, completeCmd))
    {
        return false;
    }

    ui->downloadProgressBar->setValue(i + 1);
    return true;
}

bool LidarUpgrade::downloadMass2(uint8_t startCmd, const char *startStr, uint8_t sendCmd, const char *sendStr, uint8_t completeCmd, const char *completeStr, int block_size, QFile *firmware_file)
{
    QByteArray data;
    if(firmware_file->open(QFile::ReadOnly))
    {
        data = firmware_file->readAll();
        firmware_file->close();
    }
    else
    {
        firmware_file->close();
        return false;
    }

    std::vector<uint8_t> firmware;
    firmware.clear();
    firmware.resize(data.size());
    int k = 0;
    for(auto n : data)
    {
        firmware[k++] = n;
    }

    int mod = firmware.size() % 4;
    for(int i = 0; i < mod; i++)
    {
        firmware.push_back(0xff);
    }

    uint8_t send_buf[1024];

    if(sizeof(send_buf) < block_size + PACK_STRING_LEN)
    {
        return false;
    }

    uint16_t firmware_head_len = 288;
    uint8_t *firmware_head = (uint8_t *)firmware.data();

    //qDebug() << "size:" << firmware_head_len;

    memset(send_buf, 0, sizeof(send_buf));
    if(startStr != NULL && strlen(startStr) < PACK_STRING_LEN)
    {
        strcpy((char *)send_buf, startStr);
        memcpy(&send_buf[PACK_STRING_LEN], (uint8_t *)firmware_head, firmware_head_len);
        lidarPkg->sendBytes(1, startCmd, (uint8_t *)send_buf, PACK_STRING_LEN + firmware_head_len, 0);
    }
    else
    {
        return false;
    }

    lidarPkg->waitWork(1500);    //等待flash擦除完毕

    if(!lidarPkg->waitAck(2000, startCmd))
    {
        return false;
    }

    uint8_t *p32 = firmware_head + firmware_head_len;
    int split = (firmware.size() - firmware_head_len) / block_size;    //分割成小块发送，MCU端的缓存区较小。
    int split_mod = (firmware.size() - firmware_head_len) % block_size;

    ui->downloadProgressBar->setRange(0, split + ((split_mod == 0) ? 0 : 1));

    int i;
    for(i = 0; i < split; i++)
    {
        memset(send_buf, 0, sizeof(send_buf));
        if(sendStr != NULL && strlen(sendStr) < PACK_STRING_LEN)
        {
            strcpy((char *)send_buf, sendStr);
            memcpy(&send_buf[PACK_STRING_LEN], (uint8_t *)p32, block_size);
            lidarPkg->sendBytes(1, sendCmd, (uint8_t *)send_buf, block_size + PACK_STRING_LEN, i * block_size);
        }
        else
        {
            return false;
        }

        p32 += block_size;
        if(!lidarPkg->waitAck(1000, sendCmd))
        {
            return false;
        }
        ui->downloadProgressBar->setValue(i);
    }

    if(split_mod != 0)
    {
        memset(send_buf, 0, sizeof(send_buf));
        if(sendStr != NULL && strlen(sendStr) < PACK_STRING_LEN)
        {
            strcpy((char *)send_buf,sendStr);
            memcpy(&send_buf[PACK_STRING_LEN], (uint8_t *)p32, split_mod);
            lidarPkg->sendBytes(1, sendCmd, (uint8_t *)send_buf, split_mod + PACK_STRING_LEN, i * block_size);
        }
        else
        {
            return false;
        }

        if(!lidarPkg->waitAck(1000, sendCmd))
        {
            return false;
        }
    }

    //根据不同的文件类型，可写入不同的flag
    if(completeCmd == PACK_UPGRADE_COMPLETE)
    {
        memset(send_buf, 0, sizeof(send_buf));
        if(completeStr != NULL && strlen(completeStr) < PACK_STRING_LEN)
        {
            memset(send_buf, 0, sizeof(send_buf));
            strcpy((char *)send_buf, completeStr);
            lidarPkg->sendBytes(1, completeCmd, (uint8_t *)send_buf, PACK_STRING_LEN, 0);
        }
        else
        {
            return false;
        }

        ui->downloadProgressBar->setValue(i + 1);
    }
    return true;
}
