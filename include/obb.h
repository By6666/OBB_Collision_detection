#ifndef OBB_H
#define OBB_H

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <limits>

typedef cv::Point2f Point_type;
typedef std::vector<Point_type> PointSet_type;

class OBB {
 public:
  OBB(const PointSet_type& object_l, const PointSet_type& object_r)
      : obj_1_(object_l), obj_2_(object_r) {
    projection_axis_.reserve(obj_1_.size() + obj_2_.size());
  }

  /* 检测是否碰撞 */
  bool IsCollision();

 private:
  struct ResultRange {
    float min;
    float max;
  };
  PointSet_type obj_1_;
  PointSet_type obj_2_;
  PointSet_type projection_axis_;
  /* 获得所有投影轴 */
  void get_all_projection_axis();

  /* 获得单个投影轴 */
  Point_type get_single_projection_axis(const Point_type& check_edge);

  /* 计算一个obj在投影轴上的范围 */
  ResultRange CalculateRange(const Point_type& project_axis,
                             const PointSet_type& obj);

  /* 向量 vertex_2 指向 vertex_1 */
  inline Point_type CalculateVector(const Point_type& vertex_1,
                                    const Point_type& vertex_2) {
    return Point_type(vertex_1.x - vertex_2.x, vertex_1.y - vertex_2.y);
  }

  /* 两个向量的点乘 */
  inline float ScalarProduct(const Point_type& vector_1,
                             const Point_type& vector_2) {
    return vector_1.x * vector_2.x + vector_1.y * vector_2.y;
  }

  /* 判断两个范围有没有交集
   * true 相交，false 不相交*/
  inline bool JudgeIntersecting(const ResultRange& rang_1,
                                const ResultRange& rang_2) {
    return rang_1.max >= rang_2.min && rang_2.max >= rang_1.min;
  }
};

#endif
