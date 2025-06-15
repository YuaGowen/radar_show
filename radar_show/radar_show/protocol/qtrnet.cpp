#include "qtrnet.h"
#include "QTime"
#include "QCoreApplication"
#include "QDebug"
#include <QtAlgorithms>
#include <math.h>
#include <QVector>
#include <QtMath>
#include "sslbf.h"
#include "qdebug.h"

QTRNet::QTRNet(QSerialPort *serialPort) : QObject(serialPort)
{
    if (serialPort == NULL)
    {
        throw "SerialPort is NULL";
    }

    this->serialPort = serialPort;
}

bool QTRNet::sendCmd(uint8_t address, uint8_t id, uint32_t config)
{
    if(isWaiting == true)
    {
        return false;
    }

    if (!serialPort->isOpen())
    {
        return false;
    }

    TRNet pkg;
    std::vector<uint8_t> out;
    TRData outData;
    outData.device_address = address;
    outData.pack_id = id;
    outData.chunk_offset = 0;
    outData.data.resize(4);
    memcpy(outData.data.data(), &config, 4);
    pkg.Pack(outData, out);

    serialPort->write((char *)out.data(), out.size());
    waitWork(50);
    return true;
}

bool QTRNet::sendWords(uint8_t address, uint8_t id, uint32_t *data, uint32_t len, uint16_t offset)
{
    if(isWaiting == true)
    {
        return false;
    }

    if(!serialPort->isOpen())
    {
        return false;
    }

    TRNet pkg;
    std::vector<uint8_t> out;
    TRData outData;
    outData.device_address = address;
    outData.pack_id = id;
    outData.chunk_offset = offset;
    outData.data.resize(len);
    memcpy(outData.data.data(), data, len);
    pkg.Pack(outData, out);

    serialPort->write((char *)out.data(), out.size());
    return true;
}

bool QTRNet::sendBytes(uint8_t address, uint8_t id, uint8_t *data, uint32_t len, uint16_t offset)
{
    if(isWaiting == true)
    {
        return false;
    }

    if(!serialPort->isOpen())
    {
        return false;
    }

    TRNet pkg;
    std::vector<uint8_t> out;
    TRData outData;
    outData.device_address = address;
    outData.pack_id = id;
    outData.chunk_offset = offset;
    outData.data.resize(len);
    memcpy(outData.data.data(), data, len);
    pkg.Pack(outData, out);

    serialPort->write((char *)out.data(), out.size());
    return true;
}

void QTRNet::parsePkg(const QByteArray &data)
{
    TRNet t;
    foreach(char d, data)
    {
        lidarDataQueue.push_back((uint8_t)d);
    }

    int pos = 0;

    for(int i = 0; i < (int)lidarDataQueue.size() - 4;)
    {
        bool is_find = t.FindLeadingCode(lidarDataQueue.data() + i);

        if(is_find)
        {
            const TRData *tr_data = nullptr;
            tr_data = t.Unpack(lidarDataQueue.data() + i, lidarDataQueue.size() - i);

            if(tr_data != nullptr)
            {
                mRecPackage.data = tr_data->data;
                mRecPackage.pack_id = tr_data->pack_id;
                mRecPackage.device_address = tr_data->device_address;
                mRecPackage.chunk_offset = tr_data->chunk_offset;

                if(mRecPackage.pack_id == PACK_GET_DISTANCE || mRecPackage.pack_id == PACK_GET_CENTRIOD || mRecPackage.pack_id == PACK_GET_COE || mRecPackage.pack_id == PACK_GET_TAB)
                {
                    parseMapData(&mRecPackage);
                }
                else if(mRecPackage.pack_id == PACK_ACK)
                {
                    ackType = mRecPackage.data.data()[0];
                    ackResult = mRecPackage.data.data()[1];
                }
                else
                {
                    emit lidarDataReady(&mRecPackage);
                }

                i += t.GetParseDataLen();
                pos = i;
            }
            else
            {
                i++;
            }
        }
        else
        {
            i++;
        }
    }

    if(pos > 0)
    {
        lidarDataQueue.erase(lidarDataQueue.begin(), lidarDataQueue.begin() + pos);
    }
}

QList<MapData> QTRNet::getMap() const
{
    return mRecMap;
}


void QTRNet::parseMapData(TRData *pack)
{
    if(pack->pack_id == PACK_GET_COE)
    {
        if(lidarType != "LD07")
        {
            memcpy(coe_20[pack->device_address], pack->data.data(), sizeof(coe_20[0]));
            qDebug() << "coe" << pack->device_address;
            for(int i=0; i<42; i++)
            {
                qDebug() << "coe" << i << ":" << coe_20[1][i];
            }
            emit coeReady();
        }
        else if(lidarType=="LD07")
        {
            memcpy(coe_07[pack->device_address], pack->data.data(), sizeof(coe_07[0]));
            emit coeReady();
        }
    }
    else if(pack->pack_id == PACK_GET_DISTANCE)
    {
        int data_len = pack->data.size();
        QVector<MapData> tempLidarData;
        int real_exposure= 0;
    //        #define MODE 0
    //        if(MODE == 1)
    //        {
    //            qDebug() << "exposure: " << (pack->data.data()[2]) << " value : " << (pack->data.data()[3]);
    //            qDebug() << "debug value/run time: " << *(uint16_t*)pack->data.data();
    //            real_exposure = (pack->data.data()[2]);
    //        }

    //        if(MODE == 0)
    //        {
    //            qDebug() << "exposure: " << (pack->chunk_offset & 0xff) << "offset time: " << (pack->chunk_offset>>8);
    //            qDebug() << "debug value/run time: " << *(uint32_t*)pack->data.data();
    //            real_exposure =  (pack->chunk_offset & 0xff);
    //        }

        int maxDis_in_70 = 0;       //输出到map
        int maxDis_out_70= 0;       //输出到map
        int minDis_in_70 = 10000;   //输出到map
        int minDis_out_70 = 10000;  //输出到map
        double left_angle = 0;      //输出到map
        double right_angle = 0;     //输出到map
        double angleResolution = 0; //输出到map
        double average_sum = 0;
        int average_cnt = 0;
        double average = 0;
        double max_angel = 55.0;
        double lastAngle;

        int max_dis = 0;
        int min_dis = 9999;
        int dis_source = 0;
        double AR = 0;
        double last_angle;

        if(lidarType=="SSL-20L2" || lidarType=="SSL-20N2"/* || lidarType=="SSL-20L3"*/)//设定有效视场角最大值
        {
            max_angel = 60;
        }
        else if(lidarType == "SSL-20N" || lidarType == "SSL-20NB2" || lidarType=="SSL-20L3" || lidarType == "SSL-20L3F7")
        {
            max_angel = 55;
        }
        else if(lidarType == "SSL-20L" || lidarType == "SSL-20LB2" || lidarType == "SSL-20LB5" || lidarType == "SSL-20P")
        {
            max_angel = 50;
        }
        else
        {
            max_angel = 50;
        }

        last_angle = -max_angel;//设定初始角度用于计算角分辨率
        lastAngle = -max_angel;//作用同上

        if(pack->device_address == 0)
            pack->device_address = 1;
        for(int i = 0, n = 0; i < data_len; i += 6, n++)
        {
            MapData data;
            data.index = n;
            data.intensity = 255;
            data.real_index = n;

            //点数据按型号处理
            if(lidarType=="SSL-20L" || lidarType=="SSL-20N" || lidarType=="LD07N" || lidarType=="SSL-20LB2" ||  lidarType=="SSL-20NB2")
            {
                //20N/20L 160个数据
                uint16_t origin_data_distacne = *(uint16_t *)(pack->data.data() + i);

                data.distance  = origin_data_distacne/100.0;                 //距离   [9:0]
                int16_t origin_data_speed = *(int16_t *)(pack->data.data() + i + 2);
                data.centriod = origin_data_speed/100.0;
                int16_t origin_data_angle = *(int16_t *)(pack->data.data() + i + 4);
                data.angle = origin_data_angle/100.0;

                qDebug() << "n=" << n << ","<<data.distance <<","<<data.centriod <<","<<data.angle;
                //做角度计算
               // angleTransform(data.distance, data.centriod, n * 2, &data.angle, &data.distance, pack->device_address);
            }
            else if(lidarType=="SSL-20P" || lidarType=="SSL-20PL")
            {
                uint16_t origin_data = *(uint16_t *)(pack->data.data() + i + 4);

                data.distance  = (origin_data & 0x07ff);                //距离   [10:0]
                data.dis_source = data.distance;
                data.intensity = ((origin_data >> 11) & 0x003) << 6;    //置信度 [12:11] * 64
                data.centriod  = ((origin_data >> 13) & 0x007) ;        //kb范围 [15:13]

                angleTransform(data.distance, data.centriod, n * 2, &data.angle, &data.distance, pack->device_address);
            }
            else if(lidarType=="LD07")
            {
                uint16_t origin_data = *(uint16_t *)(pack->data.data() + i + 4);

                data.distance  = (origin_data & 0x01ff);
                data.intensity = (origin_data >> 9);

                angleTransform(data.distance, data.centriod, n, &data.angle, &data.distance, pack->device_address);
            }
            else if(lidarType=="SSL-20L3" || lidarType == "SSL-20L3F7")
            {
                //20L3 160个数据
                uint16_t origin_data = *(uint16_t *)(pack->data.data() + i + 4);

                data.distance  = (origin_data & 0x1ff);                 //距离   [8:0]
                data.dis_source = data.distance;
                data.intensity = ((origin_data >> 9) & 0x007) << 5;    //置信度 [11:9] * 32
                data.centriod  = ((origin_data >> 12) & 0x00f);         //kb范围 [15:12]


                 data.real_index = 2*n;
                 data.real_index = n;
                 //qDebug()<<"cn:"<<2*n;
                angleTransform(data.distance, data.centriod,n/* data.index*/, &data.angle, &data.distance, pack->device_address);
            }
            else if(lidarType=="SSL-20L2" || lidarType=="SSL-20N2")
            {
                //20N2/20L2 240个数据
                uint16_t origin_data = *(uint16_t *)(pack->data.data() + i + 4);

                data.distance  = (origin_data & 0x1ff);                 //距离   [8:0]
                data.dis_source = data.distance;
                data.intensity = ((origin_data >> 9) & 0x007) << 5;    //置信度 [11:9] * 32
                data.centriod  = ((origin_data >> 12) & 0x00f);         //kb范围 [15:12]

                if(n < 40)
                {
                    data.real_index = 2*n;
                }
                else if(n <200)
                {
                    data.real_index = n + 40;
                }
                else
                {
                    data.real_index = 2*n - 160;
                }

                angleTransform(data.distance, data.centriod, data.index, &data.angle, &data.distance, pack->device_address);
            }
            else
            {
                break;//未定义型号不做处理
            }

//            // 原有的全角度角度分辨率计算、全角度最大最小值计算，仅debug使用，未输出到点云图
//            if((-max_angel) <= data.angle && data.angle <= max_angel)
//            {
//                if(data.angle != -90)
//                {
//                    if(data.angle - last_angle > AR)
//                    {
//                        //qDebug() << "n=" << n << ","<<AR<<data.angle<<","<<last_angle;
//                        AR = data.angle - last_angle;
//                       // angleResolution = AR;
//                    }
//                }
//                if(dis_source > max_dis)
//                {
//                    max_dis = dis_source;
//                }
//                if(dis_source < min_dis && dis_source != 0)
//                {
//                    min_dis = dis_source;
//                }
//                last_angle = data.angle;
//            }

            // 后增加的，获得精度测试输出数据，共用，输出到点原图
//            uint16_t temp = data.dis_source;
//            if(lidarType=="SSL-20L2")
//            {
//                //按距离消除距离为零和无穷大的点
//                if(temp >= 511)
//                    continue;
//            }

//            if(temp != 0 && data.angle > -35 && data.angle < 35)//±35°以内, 70度视场角以内
//            {
//                if(temp > maxDis_in_70 && temp < 1900)//获得最大值
//                {
//                    maxDis_in_70 = temp;
//                }

//                if(temp < minDis_in_70 && temp > 10)//获得最小值
//                {
//                    minDis_in_70 = temp;
//                }
//            }
//            else if(temp != 0 && data.angle >= (-max_angel) && data.angle <= max_angel)//±35°以外，额定视场角以内
//            {
//                if(temp > maxDis_out_70 && temp < 1900)//获得最大值
//                {
//                    maxDis_out_70 = temp;
//                }

//                if(temp < minDis_out_70 && temp > 10)//获得最小值
//                {
//                    minDis_out_70 = temp;
//                }

//                    //此处计算数据有问题
//    //                if(lastAngle != (-max_angel))//获得角分辨率
//    //                {
//    //                    if(data.angle - lastAngle > angleResolution)
//    //                    {
//    //                        angleResolution = data.angle - lastAngle;
//    //                    }
//    //                }
//    //                lastAngle = data.angle;
//            }

//            if(temp != 0 && data.angle >= (-max_angel) && data.angle <= max_angel)//获得距离的和计算平均值
//            {
//                average_sum += temp;
//                average_cnt++;
//            }

//            if(temp != 0)
//            {
//                if(data.angle < 0) {//获得左右角度
//                    if(data.angle < left_angle)
//                        left_angle = data.angle;
//                } else {
//                    if(data.angle > right_angle)
//                        right_angle = data.angle;
//                }
//            }

            tempLidarData.push_back(data);//统一添加点
        }

        //qDebug() << angleResolution;
        //qDebug() << "max: " << max_dis << " min: " << min_dis;

        QList<MapData> tempMapData;
        for(auto tmp : tempLidarData)
        {
            tempMapData << tmp;
        }
        mRecMap = tempMapData;

    //        std::vector<PointData> pending, normal;
    //        pending.resize(mapTmp.size());
    //        for (int i = 0 ; i < mapTmp.size() ; i++)
    //        {
    //            pending[i] = PointData(mapTmp[i].angle, mapTmp[i].distance, mapTmp[i].confidence, mapTmp[i].index);
    //        }
    //        normal = t.nearFilter(pending);

    //        for (int i = 0 ; i < normal.size() ; i++)
    //        {
    //            MapData d;
    //            d.index = normal[i].index;
    //            d.angle = normal[i].angle;
    //            d.distance = normal[i].distance;
    //            underwaterCorrect(normal[i].distance, normal[i].angle, &d.distance, &d.angle);
    //            d.confidence = normal[i].confidence;
    //            filterMap << d;
    //        }
    //        map = filterMap;


        if(pack->device_address == 1 || pack->device_address == 0)
        {
        //    average = average_sum/average_cnt;
//            if(MODE == 1)
//            {
//                emit paraReady(maxDis_in_70, maxDis_out_70, minDis_in_70, minDis_out_70, angleResolution, average, left_angle, right_angle,real_exposure, *(uint16_t*)pack->data.data());
//            }

//            if(MODE == 0)
//            {
                emit paraReady(maxDis_in_70, maxDis_out_70, minDis_in_70, minDis_out_70, angleResolution, average, left_angle, right_angle,real_exposure, *(uint32_t*)pack->data.data());
//            }
              emit mapReady1(mRecMap);
        }
        else if(pack->device_address == 2)
        {
            emit mapReady2(mRecMap);
        }
        else if(pack->device_address == 4)
        {
            emit mapReady3(mRecMap);
        }
        tempMapData.clear();
    }
    else if(pack->pack_id == PACK_GET_CENTRIOD)
    {
        int data_len = pack->data.size();
        QVector<MapData> tempLidarData;
        for(int i = 0, n = 0; i < data_len; i += 2, n++)
        {
            MapData data;
            data.index = n;
            data.distance = *(uint16_t *)(pack->data.data() + i);
            if(lidarType=="SSL-20L2" || lidarType=="SSL-20N2")
            {
                if(i >= 640)
                    break;
                int pixelU = 0;
                if(n < 40)
                {
                    data.real_index = 2*n;
                }
                else if(n < 200)
                {
                    data.real_index = n +40;
                }
                else
                {
                    data.real_index = 2*n - 160;
                }
               data.angle = (double)(data.real_index - 160) * 0.4;
                data.intensity = 200;
            }
            else if(lidarType=="SSL-20L3" || lidarType=="SSL-20L3F7")
            {
                if(i >= 320)
                    break;
                int pixelU = 0;

                data.real_index = n;

                data.angle = (double)(data.real_index - 80) * 0.8;
                data.intensity = 200;
            }
            else
            {
                if(i >= 320)
                    break;
                data.real_index = n;
                data.angle = (double)(n - 80) * 0.5;
                data.intensity = 200;
            }

            tempLidarData.push_back(data);
        }

        QList<MapData> tempMapData;
        for(auto tmp : tempLidarData)
        {
            tempMapData << tmp;
        }
        mRecMap = tempMapData;

        if(pack->device_address == 1)
        {
            emit mapReady1(mRecMap);
        }
        else if(pack->device_address == 2)
        {
            emit mapReady2(mRecMap);
        }
        else if(pack->device_address == 4)
        {
            emit mapReady3(mRecMap);
        }
        tempMapData.clear();
    }
}

void QTRNet::angleTransform(uint16_t distance, uint8_t centriod, int n, double *cal_angle, double *cal_distance, uint8_t address)
{
    if(lidarType=="LD07")
    {
        const double k0 = (double)coe_07[address][0] / 10000;
        const double k1 = (double)coe_07[address][1] / 10000;
        const double b0 = (double)coe_07[address][2] / 10000;
        const double b1 = (double)coe_07[address][3] / 10000;
        double pixelU = n, calDist, theta, tempTheta, tempDist, tempX, tempY;

        if (pixelU > 80)
        {
            pixelU = pixelU - 80;
            pixelU = 80 - pixelU;
            if (b0 > 1) //217之前的版本计算的b值在20-30之间，217之后的版本计算的b值小于1;
            {
                tempTheta = k0 * pixelU - b0;
            }
            else
            {
                tempTheta = atan(k0 * pixelU - b0) * 180 / pi;
            }
            tempDist = (distance - 1.22) / cos((22.5 - (tempTheta)) * pi / 180);
            tempTheta = tempTheta * pi / 180;
            tempX = cos(22.5 * pi / 180) * tempDist * cos(tempTheta) + sin(22.5 * pi / 180) * (tempDist * sin(tempTheta));
            tempY = -sin(22.5 * pi / 180) * tempDist * cos(tempTheta) + cos(22.5 * pi / 180) * (tempDist * sin(tempTheta));
            tempX = tempX + 1.22;
            tempY = tempY - 5.315;
            calDist = sqrt(tempX * tempX + tempY * tempY);
            theta = atan(tempY / tempX) * 180 / pi;
        }
        else
        {
            pixelU = 80 - pixelU;
            if (b1 > 1)
            {
                tempTheta = k1 * pixelU - b1;
            }
            else
            {
                tempTheta = atan(k1 * pixelU - b1) * 180 / pi;
            }
            tempDist = (distance - 1.22) / cos((22.5 + (tempTheta)) * pi / 180);
            tempTheta = tempTheta * pi / 180;
            tempX = cos(-22.5 * pi / 180) * tempDist * cos(tempTheta) + sin(-22.5 * pi / 180) * (tempDist * sin(tempTheta));
            tempY = -sin(-22.5 * pi / 180) * tempDist * cos(tempTheta) + cos(-22.5 * pi / 180) * (tempDist * sin(tempTheta));
            tempX = tempX + 1.22;
            tempY = tempY + 5.315;
            calDist = sqrt(tempX * tempX + tempY * tempY);
            theta = atan(tempY / tempX) * 180 / pi;
        }

        if(distance != 0)
        {
            *cal_angle = theta;
            *cal_distance = calDist;
        }
        if(distance == 0)
        {
            *cal_angle = theta;
            *cal_distance = 0;
        }
    }
    else if(lidarType=="SSL-20L2" || lidarType=="SSL-20N2" || lidarType=="SSL-20L3" || lidarType=="SSL-20L3F7")
    {
//        double a0 = (double)coe_20[address][0] * 1E-17;
//        double a1 = (double)coe_20[address][1] * 1E-17;
//        double a2 = (double)coe_20[address][2] * 1E-17;
//        double a3 = (double)coe_20[address][3] * 1E-17;
//        double a4 = (double)coe_20[address][4] * 1E-17;

//        double b0 = (double)coe_20[address][5] * 1E-14;
//        double b1 = (double)coe_20[address][6] * 1E-14;
//        double b2 = (double)coe_20[address][7] * 1E-14;
//        double b3 = (double)coe_20[address][8] * 1E-14;
//        double b4 = (double)coe_20[address][9] * 1E-14;

//        double c0 = (double)coe_20[address][10] * 1E-11;
//        double c1 = (double)coe_20[address][11] * 1E-11;
//        double c2 = (double)coe_20[address][12] * 1E-11;
//        double c3 = (double)coe_20[address][13] * 1E-11;
//        double c4 = (double)coe_20[address][14] * 1E-11;

//        double d0 = (double)coe_20[address][15] * 1E-9;
//        double d1 = (double)coe_20[address][16] * 1E-9;
//        double d2 = (double)coe_20[address][17] * 1E-9;
//        double d3 = (double)coe_20[address][18] * 1E-9;
//        double d4 = (double)coe_20[address][19] * 1E-9;

        double a0 = (double)coe_20[address][0] * 1E-20;
        double a1 = (double)coe_20[address][1] * 1E-20;
        double a2 = (double)coe_20[address][2] * 1E-20;
        double a3 = (double)coe_20[address][3] * 1E-20;
        double a4 = (double)coe_20[address][4] * 1E-20;

        double b0 = (double)coe_20[address][5] * 1E-17;
        double b1 = (double)coe_20[address][6] * 1E-17;
        double b2 = (double)coe_20[address][7] * 1E-17;
        double b3 = (double)coe_20[address][8] * 1E-17;
        double b4 = (double)coe_20[address][9] * 1E-17;

        double c0 = (double)coe_20[address][10] * 1E-14;
        double c1 = (double)coe_20[address][11] * 1E-14;
        double c2 = (double)coe_20[address][12] * 1E-14;
        double c3 = (double)coe_20[address][13] * 1E-14;
        double c4 = (double)coe_20[address][14] * 1E-14;

        double d0 = (double)coe_20[address][15] * 1E-12;
        double d1 = (double)coe_20[address][16] * 1E-12;
        double d2 = (double)coe_20[address][17] * 1E-12;
        double d3 = (double)coe_20[address][18] * 1E-12;
        double d4 = (double)coe_20[address][19] * 1E-12;

        double e0 = (double)coe_20[address][20] * 1E-7;
        double e1 = (double)coe_20[address][21] * 1E-7;
        double e2 = (double)coe_20[address][22] * 1E-7;
        double e3 = (double)coe_20[address][23] * 1E-7;
        double e4 = (double)coe_20[address][24] * 1E-7;

        double f0 = (double)coe_20[address][25] * 1E-5;
        double f1 = (double)coe_20[address][26] * 1E-5;
        double f2 = (double)coe_20[address][27] * 1E-5;
        double f3 = (double)coe_20[address][28] * 1E-5;
        double f4 = (double)coe_20[address][29] * 1E-5;

        //分段计算处理
        double a_1, b_1, c_1, d_1, e_1, f_1;
        double a_2, b_2, c_2, d_2, e_2, f_2;
        bool use_two_coe_flag = false;

        switch (centriod)
        {
            case 1:
            {
                use_two_coe_flag = false;
                a_1 = a0;
                b_1 = b0;
                c_1 = c0;
                d_1 = d0;
                e_1 = e0;
                f_1 = f0;
                break;
            }
            case 2:
            {
                use_two_coe_flag = true;
                a_1 = a0;
                b_1 = b0;
                c_1 = c0;
                d_1 = d0;
                e_1 = e0;
                f_1 = f0;

                a_2 = a1;
                b_2 = b1;
                c_2 = c1;
                d_2 = d1;
                e_2 = e1;
                f_2 = f1;
                break;
            }
            case 3:
            {
                use_two_coe_flag = false;
                a_1 = a1;
                b_1 = b1;
                c_1 = c1;
                d_1 = d1;
                e_1 = e1;
                f_1 = f1;
                break;
            }
            case 4:
            {
                use_two_coe_flag = true;
                a_1 = a1;
                b_1 = b1;
                c_1 = c1;
                d_1 = d1;
                e_1 = e1;
                f_1 = f1;

                a_2 = a2;
                b_2 = b2;
                c_2 = c2;
                d_2 = d2;
                e_2 = e2;
                f_2 = f2;
                break;
            }
            case 5:
            {
                use_two_coe_flag = false;
                a_1 = a2;
                b_1 = b2;
                c_1 = c2;
                d_1 = d2;
                e_1 = e2;
                f_1 = f2;
                break;
            }
            case 6:
            {
                use_two_coe_flag = true;
                a_1 = a2;
                b_1 = b2;
                c_1 = c2;
                d_1 = d2;
                e_1 = e2;
                f_1 = f2;

                a_2 = a3;
                b_2 = b3;
                c_2 = c3;
                d_2 = d3;
                e_2 = e3;
                f_2 = f3;
                break;
            }
            case 7:
            {
                use_two_coe_flag = false;
                a_1 = a3;
                b_1 = b3;
                c_1 = c3;
                d_1 = d3;
                e_1 = e3;
                f_1 = f3;
                break;
            }
            case 8:
            {
                use_two_coe_flag = true;
                a_1 = a3;
                b_1 = b3;
                c_1 = c3;
                d_1 = d3;
                e_1 = e3;
                f_1 = f3;

                a_2 = a4;
                b_2 = b4;
                c_2 = c4;
                d_2 = d4;
                e_2 = e4;
                f_2 = f4;
                break;
            }
            case 9:
            {
                use_two_coe_flag = false;
                a_1 = a4;
                b_1 = b4;
                c_1 = c4;
                d_1 = d4;
                e_1 = e4;
                f_1 = f4;
                break;
            }

            default:
            {
                a_1 = a0;
                b_1 = b0;
                c_1 = a0;
                d_1 = b0;
                e_1 = e0;
                f_1 = f0;
                break;
            }
        }

        //角度信息计算，320个数据中，中心160个数据全用，左右两侧160对称降采样取80个，得到240个数据
        //角度计算时，需带入数据序号0-319，先用0-239的序号恢复成0-319的序号再带入计算
        double pixelU = n;

        if(lidarType=="SSL-20L3" || lidarType=="SSL-20L3F7")
        {
            pixelU = n*2;
        }
        else
        {
            if(n < 40)
            {
                pixelU = 2*n;
            }
            else if(n <200)
            {
                pixelU = n +40;
            }
            else
            {
                pixelU = 2*n - 160;
            }
        }

        double temp_distance, temp_angle;

        //计算角度距离
        double tanTheta = a_1 * pixelU * pixelU * pixelU *  pixelU * pixelU  -
                          b_1 * pixelU * pixelU * pixelU *  pixelU +
                          c_1 * pixelU * pixelU * pixelU -
                          d_1 * pixelU * pixelU  +
                          e_1 * pixelU -
                          f_1;
        if(use_two_coe_flag)
        {
            double tanTheta_2 = a_2 * pixelU * pixelU * pixelU *  pixelU * pixelU  -
                                b_2 * pixelU * pixelU * pixelU *  pixelU +
                                c_2 * pixelU * pixelU * pixelU -
                                d_2 * pixelU * pixelU  +
                                e_2 * pixelU -
                                f_2;
            tanTheta = (tanTheta + tanTheta_2)/2;
        }
        //qDebug()<<n<<":"<<centriod;
        temp_angle = atan(tanTheta) * 180 / pi;

        if(last_index > n)
        {
           last_index = 0;
           last_anger = -90;
           last_centriod = 0;
        }

        if(0 && temp_angle - last_anger > 1.1 && last_centriod != 0)
        {
            switch (last_centriod)
            {
                case 1:
                {
                    use_two_coe_flag = false;
                    a_1 = a0;
                    b_1 = b0;
                    c_1 = c0;
                    d_1 = d0;
                    e_1 = e0;
                    f_1 = f0;
                    break;
                }
                case 2:
                {
                    use_two_coe_flag = true;
                    a_1 = a0;
                    b_1 = b0;
                    c_1 = c0;
                    d_1 = d0;
                    e_1 = e0;
                    f_1 = f0;

                    a_2 = a1;
                    b_2 = b1;
                    c_2 = c1;
                    d_2 = d1;
                    e_2 = e1;
                    f_2 = f1;
                    break;
                }
                case 3:
                {
                    use_two_coe_flag = false;
                    a_1 = a1;
                    b_1 = b1;
                    c_1 = c1;
                    d_1 = d1;
                    e_1 = e1;
                    f_1 = f1;
                    break;
                }
                case 4:
                {
                    use_two_coe_flag = true;
                    a_1 = a1;
                    b_1 = b1;
                    c_1 = c1;
                    d_1 = d1;
                    e_1 = e1;
                    f_1 = f1;

                    a_2 = a2;
                    b_2 = b2;
                    c_2 = c2;
                    d_2 = d2;
                    e_2 = e2;
                    f_2 = f2;
                    break;
                }
                case 5:
                {
                    use_two_coe_flag = false;
                    a_1 = a2;
                    b_1 = b2;
                    c_1 = c2;
                    d_1 = d2;
                    e_1 = e2;
                    f_1 = f2;
                    break;
                }
                case 6:
                {
                    use_two_coe_flag = true;
                    a_1 = a2;
                    b_1 = b2;
                    c_1 = c2;
                    d_1 = d2;
                    e_1 = e2;
                    f_1 = f2;

                    a_2 = a3;
                    b_2 = b3;
                    c_2 = c3;
                    d_2 = d3;
                    e_2 = e3;
                    f_2 = f3;
                    break;
                }
                case 7:
                {
                    use_two_coe_flag = false;
                    a_1 = a3;
                    b_1 = b3;
                    c_1 = c3;
                    d_1 = d3;
                    e_1 = e3;
                    f_1 = f3;
                    break;
                }
                case 8:
                {
                    use_two_coe_flag = true;
                    a_1 = a3;
                    b_1 = b3;
                    c_1 = c3;
                    d_1 = d3;
                    e_1 = e3;
                    f_1 = f3;

                    a_2 = a4;
                    b_2 = b4;
                    c_2 = c4;
                    d_2 = d4;
                    e_2 = e4;
                    f_2 = f4;
                    break;
                }
                case 9:
                {
                    use_two_coe_flag = false;
                    a_1 = a4;
                    b_1 = b4;
                    c_1 = c4;
                    d_1 = d4;
                    e_1 = e4;
                    f_1 = f4;
                    break;
                }

                default:
                {
                    a_1 = a0;
                    b_1 = b0;
                    c_1 = a0;
                    d_1 = b0;
                    e_1 = e0;
                    f_1 = f0;
                    break;
                }
            }
           centriod = last_centriod;
            //计算角度距离
            double tanTheta = a_1 * pixelU * pixelU * pixelU *  pixelU * pixelU  -
                              b_1 * pixelU * pixelU * pixelU *  pixelU +
                              c_1 * pixelU * pixelU * pixelU -
                              d_1 * pixelU * pixelU  +
                              e_1 * pixelU -
                              f_1;
            if(use_two_coe_flag)
            {
                double tanTheta_2 = a_2 * pixelU * pixelU * pixelU *  pixelU * pixelU  -
                                    b_2 * pixelU * pixelU * pixelU *  pixelU +
                                    c_2 * pixelU * pixelU * pixelU -
                                    d_2 * pixelU * pixelU  +
                                    e_2 * pixelU -
                                    f_2;
                tanTheta = (tanTheta + tanTheta_2)/2;
            }

            temp_angle = atan(tanTheta) * 180 / pi;
        }

        temp_distance = distance / cos(temp_angle * pi / 180);

//        if(last_anger != -90 && temp_angle - last_anger > 1.1 && n - last_index == 1 && (last_centriod % 2 !=0 && centriod %2 != 0||(last_centriod % 2==0 && centriod %2 != 0)||(last_centriod % 2 !=0 && centriod %2 == 0)))
//        {
//            if(last_centriod % 2==0)
//            {
//                switch (last_centriod)
//                {
//                    case 2:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a0;
//                        b_1 = b0;
//                        c_1 = c0;
//                        d_1 = d0;
//                        e_1 = e0;
//                        f_1 = f0;

//                        a_2 = a1;
//                        b_2 = b1;
//                        c_2 = c1;
//                        d_2 = d1;
//                        e_2 = e1;
//                        f_2 = f1;
//                        break;
//                    }
//                    case 4:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a1;
//                        b_1 = b1;
//                        c_1 = c1;
//                        d_1 = d1;
//                        e_1 = e1;
//                        f_1 = f1;

//                        a_2 = a2;
//                        b_2 = b2;
//                        c_2 = c2;
//                        d_2 = d2;
//                        e_2 = e2;
//                        f_2 = f2;
//                        break;
//                    }
//                    case 6:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a2;
//                        b_1 = b2;
//                        c_1 = c2;
//                        d_1 = d2;
//                        e_1 = e2;
//                        f_1 = f2;

//                        a_2 = a3;
//                        b_2 = b3;
//                        c_2 = c3;
//                        d_2 = d3;
//                        e_2 = e3;
//                        f_2 = f3;
//                        break;
//                    }
//                    case 8:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a3;
//                        b_1 = b3;
//                        c_1 = c3;
//                        d_1 = d3;
//                        e_1 = e3;
//                        f_1 = f3;

//                        a_2 = a4;
//                        b_2 = b4;
//                        c_2 = c4;
//                        d_2 = d4;
//                        e_2 = e4;
//                        f_2 = f4;
//                        break;
//                    }
//                    default:
//                    {
//                        break;
//                    }
//                }
//            }else if(centriod % 2==0)
//            {
//                switch (centriod)
//                {
//                    case 2:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a0;
//                        b_1 = b0;
//                        c_1 = c0;
//                        d_1 = d0;
//                        e_1 = e0;
//                        f_1 = f0;

//                        a_2 = a1;
//                        b_2 = b1;
//                        c_2 = c1;
//                        d_2 = d1;
//                        e_2 = e1;
//                        f_2 = f1;
//                        break;
//                    }
//                    case 4:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a1;
//                        b_1 = b1;
//                        c_1 = c1;
//                        d_1 = d1;
//                        e_1 = e1;
//                        f_1 = f1;

//                        a_2 = a2;
//                        b_2 = b2;
//                        c_2 = c2;
//                        d_2 = d2;
//                        e_2 = e2;
//                        f_2 = f2;
//                        break;
//                    }
//                    case 6:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a2;
//                        b_1 = b2;
//                        c_1 = c2;
//                        d_1 = d2;
//                        e_1 = e2;
//                        f_1 = f2;

//                        a_2 = a3;
//                        b_2 = b3;
//                        c_2 = c3;
//                        d_2 = d3;
//                        e_2 = e3;
//                        f_2 = f3;
//                        break;
//                    }
//                    case 8:
//                    {
//                        use_two_coe_flag = true;
//                        a_1 = a3;
//                        b_1 = b3;
//                        c_1 = c3;
//                        d_1 = d3;
//                        e_1 = e3;
//                        f_1 = f3;

//                        a_2 = a4;
//                        b_2 = b4;
//                        c_2 = c4;
//                        d_2 = d4;
//                        e_2 = e4;
//                        f_2 = f4;
//                        break;
//                    }
//                    default:
//                    {
//                        break;
//                    }
//                }
//            }else
//            {
//                switch (centriod)
//                {
//                    case 1:
//                    {
//                        use_two_coe_flag = false;
//                        a_1 = a0;
//                        b_1 = b0;
//                        c_1 = c0;
//                        d_1 = d0;
//                        e_1 = e0;
//                        f_1 = f0;
//                        break;
//                    }
//                    case 3:
//                    {
//                        use_two_coe_flag = false;
//                        a_1 = a1;
//                        b_1 = b1;
//                        c_1 = c1;
//                        d_1 = d1;
//                        e_1 = e1;
//                        f_1 = f1;
//                        break;
//                    }
//                    case 5:
//                    {
//                        use_two_coe_flag = false;
//                        a_1 = a2;
//                        b_1 = b2;
//                        c_1 = c2;
//                        d_1 = d2;
//                        e_1 = e2;
//                        f_1 = f2;
//                        break;
//                    }

//                    case 7:
//                    {
//                        use_two_coe_flag = false;
//                        a_1 = a3;
//                        b_1 = b3;
//                        c_1 = c3;
//                        d_1 = d3;
//                        e_1 = e3;
//                        f_1 = f3;
//                        break;
//                    }
//                    case 9:
//                    {
//                        use_two_coe_flag = false;
//                        a_1 = a4;
//                        b_1 = b4;
//                        c_1 = c4;
//                        d_1 = d4;
//                        e_1 = e4;
//                        f_1 = f4;
//                        break;
//                    }

//                    default:
//                    {
//                        break;
//                    }
//                }

//                switch (last_centriod)
//                {
//                    case 1:
//                    {
//                        a_2 = a0;
//                        b_2 = b0;
//                        c_2 = c0;
//                        d_2 = d0;
//                        e_2 = e0;
//                        f_2 = f0;
//                        break;
//                    }
//                    case 3:
//                    {
//                        a_2 = a1;
//                        b_2 = b1;
//                        c_2 = c1;
//                        d_2 = d1;
//                        e_2 = e1;
//                        f_2 = f1;
//                        break;
//                    }
//                    case 5:
//                    {
//                        a_2 = a2;
//                        b_2 = b2;
//                        c_2 = c2;
//                        d_2 = d2;
//                        e_2 = e2;
//                        f_2 = f2;
//                        break;
//                    }
//                    case 7:
//                    {
//                        a_2 = a3;
//                        b_2 = b3;
//                        c_2 = c3;
//                        d_2 = d3;
//                        e_2 = e3;
//                        f_2 = f3;
//                        break;
//                    }
//                    case 9:
//                    {
//                        a_1 = a4;
//                        b_1 = b4;
//                        c_1 = c4;
//                        d_1 = d4;
//                        e_1 = e4;
//                        f_1 = f4;
//                        break;
//                    }

//                    default:
//                    {
//                        a_1 = a0;
//                        b_1 = b0;
//                        c_1 = a0;
//                        d_1 = b0;
//                        e_1 = e0;
//                        f_1 = f0;
//                        break;
//                    }
//                }

//            }
//            use_two_coe_flag = true;
//            //计算角度距离
//            double tanTheta1 = a_1 * pixelU * pixelU * pixelU *  pixelU * pixelU  -
//                              b_1 * pixelU * pixelU * pixelU *  pixelU +
//                              c_1 * pixelU * pixelU * pixelU -
//                              d_1 * pixelU * pixelU  +
//                              e_1 * pixelU -
//                              f_1;
////            if(use_two_coe_flag)
////            {
//                double tanTheta2 = a_2 * pixelU * pixelU * pixelU *  pixelU * pixelU  -
//                                    b_2 * pixelU * pixelU * pixelU *  pixelU +
//                                    c_2 * pixelU * pixelU * pixelU -
//                                    d_2 * pixelU * pixelU  +
//                                    e_2 * pixelU -
//                                    f_2;
//                tanTheta = (tanTheta1 + tanTheta2)/2;
// //           }
//            double temp_angle0 = atan(tanTheta) * 180 / pi;
//            double temp_angle1 = atan(tanTheta1) * 180 / pi;
//            double temp_angle2 = atan(tanTheta2) * 180 / pi;

//            double temp = qMin(temp_angle1,qMin(temp_angle0,temp_angle2));
//            qDebug()<<temp_angle<<","<<centriod<<","<<last_centriod<<":"<<temp_angle0<<","<<temp_angle1<<","<<temp_angle2;
//            if(temp - last_anger < 1.1)
//            {
//                temp_angle = temp;
//            }

////            double temp = qMin(temp_angle1,temp_angle2);

////            if(temp < temp_angle0)
////            {
////               if(temp - last_anger < 1.1)
////               {
////                  temp_angle = temp;
////               }
////               else
////               {
////                  temp = qMax(temp_angle1,temp_angle2);
////                  if(temp < temp_angle0 && temp - last_anger < 1.1)
////                  {
////                      temp_angle =temp;
////                  }
////               }
////            }
//        }

        if(distance != 0)
        {
            *cal_angle = temp_angle;
            *cal_distance = temp_distance;
        }
        if(distance == 0)
        {
            *cal_angle = temp_angle;
            *cal_distance = 0;
        }
        last_centriod = centriod;
        last_index = n;
        last_anger = temp_angle;
    }
    else
    {
        double a0 = (double)coe_20[address][0] * 1E-17;
        double a1 = (double)coe_20[address][1] * 1E-17;
        double a2 = (double)coe_20[address][2] * 1E-17;
        double a3 = (double)coe_20[address][3] * 1E-17;
        double a4 = (double)coe_20[address][4] * 1E-17;

        double b0 = (double)coe_20[address][5] * 1E-14;
        double b1 = (double)coe_20[address][6] * 1E-14;
        double b2 = (double)coe_20[address][7] * 1E-14;
        double b3 = (double)coe_20[address][8] * 1E-14;
        double b4 = (double)coe_20[address][9] * 1E-14;

        double c0 = (double)coe_20[address][10] * 1E-11;
        double c1 = (double)coe_20[address][11] * 1E-11;
        double c2 = (double)coe_20[address][12] * 1E-11;
        double c3 = (double)coe_20[address][13] * 1E-11;
        double c4 = (double)coe_20[address][14] * 1E-11;

        double d0 = (double)coe_20[address][15] * 1E-9;
        double d1 = (double)coe_20[address][16] * 1E-9;
        double d2 = (double)coe_20[address][17] * 1E-9;
        double d3 = (double)coe_20[address][18] * 1E-9;
        double d4 = (double)coe_20[address][19] * 1E-9;

        double e0 = (double)coe_20[address][20] * 1E-7;
        double e1 = (double)coe_20[address][21] * 1E-7;
        double e2 = (double)coe_20[address][22] * 1E-7;
        double e3 = (double)coe_20[address][23] * 1E-7;
        double e4 = (double)coe_20[address][24] * 1E-7;

        double f0 = (double)coe_20[address][25] * 1E-5;
        double f1 = (double)coe_20[address][26] * 1E-5;
        double f2 = (double)coe_20[address][27] * 1E-5;
        double f3 = (double)coe_20[address][28] * 1E-5;
        double f4 = (double)coe_20[address][29] * 1E-5;

        //分段计算处理
        double a_, b_, c_, d_, e_, f_;
        switch (centriod)
        {
            case 1:
            {
                a_ = a0;
                b_ = b0;
                c_ = c0;
                d_ = d0;
                e_ = e0;
                f_ = f0;
                break;
            }
            case 2:
            {
                a_ = a1;
                b_ = b1;
                c_ = c1;
                d_ = d1;
                e_ = e1;
                f_ = f1;
                break;
            }
            case 3:
            {
                a_ = a2;
                b_ = b2;
                c_ = c2;
                d_ = d2;
                e_ = e2;
                f_ = f2;
                break;
            }
            case 4:
            {
                a_ = a3;
                b_ = b3;
                c_ = c3;
                d_ = d3;
                e_ = e3;
                f_ = f3;
                break;
            }
            case 5:
            {
                a_ = a4;
                b_ = b4;
                c_ = c4;
                d_ = d4;
                e_ = e4;
                f_ = f4;
                break;
            }

            default:
            {
                a_ = a0;
                b_ = b0;
                c_ = a0;
                d_ = b0;
                e_ = e0;
                f_ = f0;
                break;
            }
        }

        //qDebug() << "coe" << a_ << b_ << c_ << d_ << e_ << f_;

        //角度信息计算
        double pixelU = n;
        double temp_distance, temp_angle;

        //计算角度距离
        double tanTheta = a_ * pixelU * pixelU * pixelU *  pixelU * pixelU  -
                          b_ * pixelU * pixelU * pixelU *  pixelU +
                          c_ * pixelU * pixelU * pixelU -
                          d_ * pixelU * pixelU  +
                          e_ * pixelU -
                          f_;
        temp_angle = atan(tanTheta) * 180 / pi;
        temp_distance = distance / cos(temp_angle * pi / 180);

        qDebug() << "coea" << tanTheta;

        if(distance != 0)
        {
            *cal_angle = temp_angle;
            *cal_distance = temp_distance;
        }
        if(distance == 0)
        {
            *cal_angle = temp_angle;
            *cal_distance = 0;
        }
    }
}

void QTRNet::waitWork(int ms)
{
    if(isWaiting == true)
        return;

    QTime t;
    t.start();
    isWaiting = true;
    while (t.elapsed() < ms)
        QCoreApplication::processEvents();
    isWaiting = false;
}

void QTRNet::underwaterCorrect(uint16_t measureDis, double measureAng, uint16_t *realDis, double *realAng)
{
    int index1 = 0;
    int index2 = 0;
    int index3 = 0;
    int index4 = 0;

    for(int i = 0; i < 20; i++)
    {
        for(int j = 0; j < 100; j++)
        {
            if(i*100+j > 0)
            {
                if(measureAngle[i*100+j] < measureAng && measureAngle[i*100+j-1] >= measureAng)
                {
                    if(measureDistance[i*100+j] > measureDis && measureDistance[i*100+j-1])
                    {
                        index1 = i*100+j;
                        index2 = i*100+j-1;
                        break;
                    }
                }
            }
        }
        if(index1 != 0)
        {
            break;
        }
    }

    if(index1 / 100 > 0)
    {
        int i = index1 / 100 - 1;
        for(int j = 0; j < 100; j++)
        {
            if(measureAngle[i*100 + j] < measureAng)
            {
                index3 = i*100 + j;
                index4 = i*100 + j -1;
                break;
            }
        }
    }
    else
    {
        *realDis = 0;
        *realAng = 0;
        return;
    }

    double mAng1 = (measureAngle[index1] + measureAngle[index3])/2;
    double mAng2 = (measureAngle[index2] + measureAngle[index4])/2;
    double rAng1 = (realAngle[index1] + realAngle[index3])/2;
    double rAng2 = (realAngle[index2] + realAngle[index4])/2;

    double mDis1 = (measureDistance[index3] + measureDistance[index4])/2;
    double mDis2 = (measureDistance[index1] + measureDistance[index2])/2;
    double rDis1 = (realDistance[index3] + realDistance[index4])/2;
    double rDis2 = (realDistance[index1] + realDistance[index2])/2;

    double finalAngle = rAng1 + (rAng2 - rAng1) * (measureAng - mAng1) / (mAng2 - mAng1);
    double finalDistance = rDis1 + (rDis2 - rDis1) * (measureDis - mDis1) / (mDis2 - mDis1);

    *realAng = finalAngle;
    *realDis = int(finalDistance);
}

bool QTRNet::waitAck(int timeout , int type)
{
    bool result = false;
    QTime t;
    t.start();
    while(t.elapsed() < timeout)
    {
        if(ackType == type)
        {
            result = true;
            ackType = -1;
            break;
        }
        QCoreApplication::processEvents();
    }
    return result ;
}

