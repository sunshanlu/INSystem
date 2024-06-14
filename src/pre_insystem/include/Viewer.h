#pragma once

#include <atomic>
#include <mutex>
#include <vector>

#include <pangolin/gl/glvbo.h>

#include "Common.h"

namespace insystem {

/// @brief 绘制轨迹
class Trajectory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Trajectory(int maxsize, Vec3f color);

    /// 添加轨迹点
    void AddPt(Vec3f pos);

    /// 渲染
    void Render();

    /// 请求停止
    void RequestStop();

private:
    std::vector<Vec3f> Pos_;
    std::size_t MaxSize_;
    pangolin::GlBuffer Vbo_;
    Vec3f Color_;
};

class Coordinate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Coordinate(Vec3f ColorX, Vec3f ColorY, Vec3f ColorZ, float x, float y, float z)
        : ColorX_(ColorX)
        , ColorY_(ColorY)
        , ColorZ_(ColorZ)
        , XAxis_(Vec3f(x, 0, 0))
        , YAxis_(Vec3f(0, y, 0))
        , ZAxis_(Vec3f(0, 0, z)) {}

    /// 更新坐标系位置
    void UpdateSE3(SE3f Twb) { SE3_ = Twb; }

    /// 渲染坐标系
    void Render();

private:
    SE3f SE3_;     ///< 坐标系位置Twc
    Vec3f ColorX_; ///< X轴颜色
    Vec3f ColorY_; ///< Y轴颜色
    Vec3f ColorZ_; ///< Z轴颜色
    Vec3f XAxis_;  ///< X轴向量
    Vec3f YAxis_;  ///< Y轴向量
    Vec3f ZAxis_;  ///< Z轴向量
};

class Viewer {

    using atomic = std::atomic<bool>;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Viewer(std::string winname, int winWidth, int winHeight, int maxsize, Vec3f TColor, Vec3f XColor, Vec3f YColor,
           Vec3f ZColor, float x, float y, float z);

    /// 更新导航状态
    void UpdateNavState(NavStateSharedPtr NavState);

    /// 渲染
    void Render();

    /// 外部线程请求停止函数
    void RequestStop() { Request_.store(true); }

    /// 返回是否停止
    bool IsStop() { return IsStop_.load(); }

private:
    std::string WinName_;        ///< 渲染窗口名称
    int WinWid_, WinHei_;        ///< 窗口大小
    mutable std::mutex Mtx_;     ///< 维护导航状态的互斥锁
    NavStateSharedPtr NavState_; ///< 当前导航状态
    Trajectory Traj_;            ///< 轨迹信息
    Coordinate Coord_;           ///< 坐标系
    atomic Request_;             ///< 外部线程请求停止
    atomic IsStop_;              ///< 是否停止
    bool NewState_;              ///< 是否是更新的导航状态
};

} // namespace insystem