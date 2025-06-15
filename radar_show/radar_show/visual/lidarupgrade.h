#ifndef LIDARUPGRADE_H
#define LIDARUPGRADE_H

#include <QWidget>
#include <QFile>
#include "qtrnet.h"
#include "encrypt.h"

#define PACK_STRING_LEN 16

namespace Ui {
class LidarUpgrade;
}

class LidarUpgrade : public QWidget
{
    Q_OBJECT

public:
    explicit LidarUpgrade(QWidget *parent = nullptr, QTRNet *lidarPkg = nullptr);
    ~LidarUpgrade();

private:
    Ui::LidarUpgrade *ui;
    Encrypt *encrypt;
    QTRNet *lidarPkg;

    QByteArray unencryptFileData;
    QString unencryptFileName;
    QFile *unencryptInFile;     //读入文件
    QFile *unencryptOutFile;    //输出文件

    QByteArray encryptFileData;
    QString encryptFileName;
    QFile *encryptInFile;     //读入文件
    QFile *encryptOutFile;    //输出文件

    const unsigned char AES128_key1[16] = {'L', 'D', 'R', 'O', 'B', 'O', 'T', 'S', 'E', 'N', 'S', 'O', 'R', 'N', 'O', '1'}; //密钥
    const unsigned char AES128_key2[16] = {'1', '2', '1', '3', '1', '9', '1', '0', '2', '8', '0', '9', NULL, NULL, NULL, NULL}; //密钥
    bool downloadMass1(uint8_t startCmd, const char *startStr, uint8_t sendCmd, const char *sendStr, uint8_t completeCmd, const char *completeStr, int block_size, QFile *firmware_file);
    bool downloadMass2(uint8_t startCmd, const char *startStr, uint8_t sendCmd, const char *sendStr, uint8_t completeCmd, const char *completeStr, int block_size, QFile *firmware_file);

    void connectInit(void);

private slots:
    void unencryptedImportFirmware(void);
    void encrpytFirmware(void);
    void unencryptedDownloadFirmware(void);
    void encryptedImportFirmware(void);
    void decyptFirmware(void);
    void encryptedDownloadFirmware(void);
    void encryptedDownloadFirmware_1000(void);
};

#endif // LIDARUPGRADE_H
