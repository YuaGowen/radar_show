#include "trnet.h"
#include <cstring>
#include <QDebug>

TRNet::TRNet():
    parseDataLen(0)
{
}

uint8_t TRNet::CalCheckSum(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        checksum += *data++;
    }

    return checksum;
}

const TRData* TRNet::Unpack(const uint8_t *data, uint32_t len)
{
    if(data == nullptr || len < minDataLen)
    {
        return nullptr;
    }

    const uint8_t *p = data;
    uint32_t code = *(uint32_t *)data;
    if(code != LEADING_CODE)
    {
        return nullptr;
    }

    p += 8;
    uint16_t data_len = *(uint16_t *)p;
    if(data_len > (len - minDataLen))
    {
        return nullptr;
    }

    p += 2;
    uint8_t checksum = CalCheckSum(data + 4, headerLen + data_len);
    p += data_len;
    if(checksum == *p)
    {
        p = data + 4;
        trData.device_address = *p++;
        trData.pack_id = *p++;
        trData.chunk_offset = *(uint16_t *)p;
        if(trData.data.size() < data_len)
        {
            trData.data.resize(data_len);
        }
        p += 4;
        std::memcpy(trData.data.data(), p, data_len);

        parseDataLen = data_len + minDataLen;

        return &trData;
    }
    return nullptr;
}

bool TRNet::Pack(const TRData &in, std::vector<uint8_t> &out)
{
    out.resize(minDataLen + in.data.size());

    uint8_t *p = out.data();
    *(uint32_t *)p = LEADING_CODE;
    p += 4;
    *p++ = in.device_address;
    *p++ = in.pack_id;
    *(uint16_t *)p = in.chunk_offset;
    p += 2;
	*(uint16_t *)p = in.data.size();
	p += 2;
	std::memcpy(p, in.data.data(), in.data.size());
    uint8_t checksum = CalCheckSum(out.data() + 4, out.size() - 5);
	out.back() = checksum;
    return true;
}

bool TRNet::FindLeadingCode(const uint8_t *buff)
{
    uint32_t code = *(uint32_t *)buff;
    return (code == LEADING_CODE);
}
