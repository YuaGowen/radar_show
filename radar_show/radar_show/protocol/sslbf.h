#ifndef __Sslbf_H_
#define __Sslbf_H_

#include <stdint.h>
#include <vector>

struct PointData
{
    double      angle;
    uint16_t    distance;
    uint16_t    confidence;
    uint32_t    index;
    int16_t     offset;
    uint16_t    centriod;

    PointData(double angle, uint16_t distance, uint16_t confidence, uint32_t index)
    {
        this->angle      = angle;
        this->distance   = distance;
        this->confidence = confidence;
        this->index      = index;
    }
    PointData() {}
};

class Sslbf
{
private:
    const int TOTAL_POINT = 160;
    const double FOV = 110; // LD07 95°,LD20 110°
    Sslbf(const Sslbf &) = delete;
    Sslbf &operator=(const Sslbf &) = delete;
public:
    Sslbf();
    std::vector<PointData> nearFilter(std::vector<PointData> &tmp) const;
    ~Sslbf();
};

#endif
