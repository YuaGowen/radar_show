
#include "sslbf.h"
#include <math.h>
#include <iostream>
#include <algorithm>
#include <QDebug>

Sslbf::Sslbf()
{
}

Sslbf::~Sslbf()
{
}

/*!
    \brief 过滤掉不合理的数据点
    \param[in]
      \arg  tmp 组包完的一圈雷达数据
    \param[out] none
    \retval     正常的数据
*/
std::vector<PointData> Sslbf::nearFilter(std::vector<PointData> &tmp) const
{
    std::vector<PointData> normal, item;
    std::vector<std::vector<PointData>> group;

    double angle_delta_up_limit = (FOV / TOTAL_POINT) * 2;

    PointData last(-10, 0, 0, 0);
    //分组
    for (auto n : tmp)
    {
        if (abs(n.distance - last.distance) > last.distance * 0.1)
        {
            if (item.empty() == false)
            {
                group.push_back(item);
                item.clear();
            }
        }
        item.push_back(n);
        last = n;
    }

    // push back last item
    if (item.empty() == false)
        group.push_back(item);

    if (group.empty())
        return normal;

    //筛选
    for (auto n : group)
    {
        if (n.size() == 0)
            continue;

        //滤除1-2个孤立点
        if (n.size() <= 2)
        {
            for (auto& point : n)
            {
                point.distance = 0;
                point.confidence = 0;
            }
        }
        normal.insert(normal.end(), n.begin(), n.end());
    }

    return normal;
}

