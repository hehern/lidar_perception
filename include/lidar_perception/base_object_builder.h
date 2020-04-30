#ifndef BASE_OBJECT_BUILDER_H_
#define BASE_OBJECT_BUILDER_H_

#include <memory>
#include <string>
#include <vector>
#include "common/geometry_util.h"
#include "common/pcl_types.h"
#include "object.h"

namespace apollo {
namespace perception {

struct ObjectBuilderOptions {
  Eigen::Vector3d ref_center;
};

class BaseObjectBuilder {
 public:
  BaseObjectBuilder() {}
  virtual ~BaseObjectBuilder() {}

  virtual bool Init() = 0;

  // @brief: calc object feature, and fill fields.
  // @param [in]: options.
  // @param [in/out]: object list.
  virtual bool Build(const ObjectBuilderOptions& options,
                     std::vector<std::shared_ptr<Object>>* objects) = 0;

  virtual std::string name() const = 0;

 protected:
  virtual void SetDefaultValue(pcl_util::PointCloudPtr cloud,
                               std::shared_ptr<Object> obj,
                               Eigen::Vector4f* min_pt,
                               Eigen::Vector4f* max_pt) {
    GetCloudMinMax3D<pcl_util::Point>(cloud, min_pt, max_pt);//获得目标点云的最大box值,即最小XYZ和最大XYZ,分别存在min_pt max_pt里面
    Eigen::Vector3f center(((*min_pt)[0] + (*max_pt)[0]) / 2,//初始化center点
                           ((*min_pt)[1] + (*max_pt)[1]) / 2,
                           ((*min_pt)[2] + (*max_pt)[2]) / 2);

    // handle degeneration case
    float epslin = 1e-3;
    for (int i = 0; i < 3; i++) {
      if ((*max_pt)[i] - (*min_pt)[i] < epslin) {
        (*max_pt)[i] = center[i] + epslin / 2;
        (*min_pt)[i] = center[i] - epslin / 2;
      }
    }

    // length
    obj->length = (*max_pt)[0] - (*min_pt)[0];//长-X
    // width
    obj->width = (*max_pt)[1] - (*min_pt)[1];//宽-Y
    if (obj->length - obj->width < 0) {//约定最长的边为长,所以长小于宽时交换
      float tmp = obj->length;
      obj->length = obj->width;
      obj->width = tmp;
      obj->direction = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else {
      obj->direction = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    // height
    obj->height = (*max_pt)[2] - (*min_pt)[2];//高
    // center
    obj->center = Eigen::Vector3d(((*max_pt)[0] + (*min_pt)[0]) / 2,//obj的center点填入
                                  ((*max_pt)[1] + (*min_pt)[1]) / 2,
                                  ((*max_pt)[2] + (*min_pt)[2]) / 2);
    // polygon
    if (cloud->size() < 4) {                             //点云个数小于4时候,就把minbox的边界点定义为minmax3d
      obj->polygon.points.resize(4);
      obj->polygon.points[0].x = static_cast<double>((*min_pt)[0]);
      obj->polygon.points[0].y = static_cast<double>((*min_pt)[1]);
      obj->polygon.points[0].z = static_cast<double>((*min_pt)[2]);

      obj->polygon.points[1].x = static_cast<double>((*max_pt)[0]);
      obj->polygon.points[1].y = static_cast<double>((*min_pt)[1]);
      obj->polygon.points[1].z = static_cast<double>((*min_pt)[2]);

      obj->polygon.points[2].x = static_cast<double>((*max_pt)[0]);
      obj->polygon.points[2].y = static_cast<double>((*max_pt)[1]);
      obj->polygon.points[2].z = static_cast<double>((*min_pt)[2]);

      obj->polygon.points[3].x = static_cast<double>((*min_pt)[0]);
      obj->polygon.points[3].y = static_cast<double>((*max_pt)[1]);
      obj->polygon.points[3].z = static_cast<double>((*min_pt)[2]);
    }
  }
};


}  // namespace perception
}  // namespace magride

#endif
