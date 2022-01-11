#define PCL_NO_PRECOMPILE

#ifndef FPFH_RGB_ORI_
#define FPFH_RGB_ORI_


#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//经个人验证和参考GITHUB-ISSUE，自定义点类型无法使用kdtree，需要自己去实现。但是我们可以使用现有的特征类型来代替，只需要将不使用的值位置0或置为相同值。
//struct FPFH_RGB_ORI;
struct FPFH_RGB_ORI
{
    //float histogram[158] = { 0.0f };
    float histogram[158];
    inline FPFH_RGB_ORI() = default;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (FPFH_RGB_ORI,
    (float[158], histogram, fpfh_rgb_ori)
)

#endif //FPFH_RGB_ORI_