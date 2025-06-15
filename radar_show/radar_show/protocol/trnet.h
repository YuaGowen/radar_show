#ifndef _TR_NET_H
#define _TR_NET_H

#include <stdint.h>
#include <vector>

struct TRData
{
    uint8_t device_address;
    uint8_t pack_id;
    uint16_t chunk_offset;
    std::vector<uint8_t> data;
};

class TRNet
{
public:
    TRNet();
    const TRData *Unpack(const uint8_t *data, uint32_t len);    //�������
    bool Pack(const TRData &in, std::vector<uint8_t> &out);     //�������
    bool FindLeadingCode(const uint8_t *buff);                  //�ж�֡ͷ�Ƿ���ȷ
    uint32_t GetParseDataLen() {return parseDataLen;}           //���ش˴ν��������ݳ���

protected:
    TRData trData;
    
private:
    const uint32_t LEADING_CODE = 0xAAAAAAAA;
    const uint32_t headerLen = 6;
    const uint32_t minDataLen = 11;
    uint32_t parseDataLen;
    uint8_t CalCheckSum(const uint8_t *data, uint16_t len);
};

#endif // _TR_NET_H
