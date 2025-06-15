#include "pro_v2.h"

static uint8_t ParseBuffer[PARSE_LEN];  // 存放解包后的数据

/**
 * @brief  解包函数
 * @param  待解包数据指针
 * @retval true 表示成功 ， false 表示失败
 */
bool Unpacking(PackageDataStruct *package)
{
    uint16_t i = 0;

    if ((package->data_in_buff == NULL) && (package->data_in_len < MIN_PRO_NUM))
    {
        return false;
    }

    if (package->data_in_len >= MIN_PRO_NUM)
    {
        if ((*(package->data_in_buff + package->data_in_len - 1) == P_TAIL) && (*(package->data_in_buff + package->data_in_len - 2) == P_TAIL))
        {
            i = MIN_PRO_NUM - 2;
            while (i++)
            {
                if (*(package->data_in_buff + package->data_in_len - i) == P_HEADER)
                {
                    if (*(package->data_in_buff + package->data_in_len - (i + 1)) == P_HEADER)
                    {
                        uint8_t *pbuff = package->data_in_buff + package->data_in_len - (i - 1); // pbuff指向DataInBuff有效数据起始位置(即AA AA 后面的一位)
                        uint16_t len = i - 3;                                                    //i 的长度为数据包(AA AA ... 55 55) 长度减一，len为有效数据长度(除去AA AA 55 55部分，包含checksum)
                        uint16_t j = 0;
                        uint8_t checksum = 0;
                        uint16_t data_out_count = 0;

                        if (len > sizeof(ParseBuffer)) //ParseBuffer的size要足够大
                        {
                            return false;
                        }

                        for (j = 0; j < len; j++)
                        {
                            if (*(pbuff + j) == P_CTRL)
                            {
                                j++;
                            }
                            ParseBuffer[data_out_count++] = *(pbuff + j);
                            if (data_out_count == PARSE_LEN)
                            {
                                return false;
                            }
                        }

                        for (j = 0; j < data_out_count - 1; j++)
                        {
                            checksum += ParseBuffer[j];
                        }

                        if (checksum == ParseBuffer[data_out_count - 1])
                        {
                            SdkProtocolHeaderTypeDef *sdk = (SdkProtocolHeaderTypeDef *)ParseBuffer;
                            *package->data_out_len = data_out_count - 1 - sizeof(SdkProtocolHeaderTypeDef);

                            package->data_out_buff = ParseBuffer + sizeof(SdkProtocolHeaderTypeDef);


                            if (sdk->device_address == LIDAR_ADDRESS)
                            {
                                package->data_id = sdk->function_code;
                                return true;
                            }
                            else
                            {
                                return false;
                            }
                        }
                        else
                        {
                            return false;
                        }
                    }
                }
                if (i == package->data_in_len)
                {
                    return false;
                }
            }
        }
    }

    return false;
}


/**
 * @brief  数据打包函数
 * @param  package :待打包数据信息
 * @note   需要确保输入的数据信息正确，特别是out buffer最好要是待打包数据长度的两倍
 * @retval true 表示成功 ， false 表示失败
 */
bool Package(PackageDataStruct package)
{
    uint32_t j = 0;
    uint32_t i = 0;
    SdkProtocolHeaderTypeDef sdk_header;
    uint8_t *psdk = (uint8_t *)&sdk_header;
    uint8_t checksum = 0;

    if ((package.data_in_buff == NULL) || (package.data_out_buff == NULL) || (package.data_out_len == NULL))
    {
        return false;
    }

    sdk_header.device_address = LIDAR_ADDRESS;
    sdk_header.function_code = package.data_id;
    sdk_header.start_address = package.offset;
    sdk_header.len = package.data_in_len;

    *(package.data_out_buff + i++) = P_HEADER;
    *(package.data_out_buff + i++) = P_HEADER;

    for (j = 0; j < sizeof(SdkProtocolHeaderTypeDef); j++)
    {
        if ((*(psdk + j) == P_CTRL) || (*(psdk + j) == P_HEADER) || (*(psdk + j) == P_TAIL))
        {
            *(package.data_out_buff + i++) = P_CTRL;
        }
        *(package.data_out_buff + i++) = *(psdk + j);
        checksum += *(psdk + j);
    }

    for (j = 0; j < package.data_in_len; j++)
    {
        if ((*(package.data_in_buff + j) == P_CTRL) || (*(package.data_in_buff + j) == P_HEADER) || (*(package.data_in_buff + j) == P_TAIL))
        {
            *(package.data_out_buff + i++) = P_CTRL;
        }
        checksum += *(package.data_in_buff + j);
        *(package.data_out_buff + i++) = *(package.data_in_buff + j);
    }

    if ((checksum == P_CTRL) || (checksum == P_HEADER) || (checksum == P_TAIL))
    {
        *(package.data_out_buff + i++) = P_CTRL;
    }
    *(package.data_out_buff + i++) = checksum;

    *(package.data_out_buff + i++) = P_TAIL;
    *(package.data_out_buff + i++) = P_TAIL;

    *package.data_out_len = i;

    return true;
}


/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
