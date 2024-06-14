#include <pangolin/pangolin.h>

#include "NavState.h"
#include "Viewer.h"

namespace insystem {

using namespace std::chrono_literals;

Trajectory::Trajectory(int maxsize, Vec3f color)
    : MaxSize_(maxsize)
    , Color_(std::move(color)) {
    Pos_.reserve(maxsize);
}

/**
 * @brief 添加位置信息
 *
 * @param pos 输入的位置
 */
void Trajectory::AddPt(Vec3f pos) {
    Pos_.emplace_back(pos);
    if (Pos_.size() > MaxSize_)
        Pos_.erase(Pos_.begin(), Pos_.begin() + MaxSize_ / 2);
    Vbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, Pos_);
}

/**
 * @brief 轨迹渲染
 * @details
 *      1. 使用顶点数组缓冲区的方式加速渲染GlBuffer
 */
void Trajectory::Render() {
    if (!Vbo_.IsValid())
        return;
    glColor3f(Color_[0], Color_[1], Color_[2]);
    glLineWidth(3.0);
    pangolin::RenderVbo(Vbo_, GL_LINE_STRIP);
    glLineWidth(1.0);

    glPointSize(5.0);
    pangolin::RenderVbo(Vbo_, GL_POINTS);
    glPointSize(1.0);
}

/**
 * @brief 坐标系渲染
 *
 */
void Coordinate::Render() {
    Vec3f px = SE3_ * XAxis_;
    Vec3f py = SE3_ * YAxis_;
    Vec3f pz = SE3_ * ZAxis_;
    Vec3f p = SE3_.translation();
    glLineWidth(5.0);
    glBegin(GL_LINES);
    glColor3f(ColorX_[0], ColorX_[1], ColorX_[2]);
    glVertex3f(p[0], p[1], p[2]);
    glVertex3f(px[0], px[1], px[2]);

    glColor3f(ColorY_[0], ColorY_[1], ColorY_[2]);
    glVertex3f(p[0], p[1], p[2]);
    glVertex3f(py[0], py[1], py[2]);

    glColor3f(ColorZ_[0], ColorZ_[1], ColorZ_[2]);
    glVertex3f(p[0], p[1], p[2]);
    glVertex3f(pz[0], pz[1], pz[2]);
    glEnd();
    glLineWidth(1.0);
}

/**
 * @brief 可视化类的构造函数
 *
 * @param winname   输入的窗口名称
 * @param winWidth  输入的窗口宽度
 * @param winHeight 输入的窗口高度
 * @param maxsize   输入的轨迹容器的最大size
 * @param TColor    输入的轨迹颜色
 * @param XColor    输入的x轴颜色
 * @param YColor    输入的y轴颜色
 * @param ZColor    输入的z轴颜色
 * @param x         输入的x长度
 * @param y         输入的y长度
 * @param z         输入的z长度
 */
Viewer::Viewer(std::string winname, int winWidth, int winHeight, int maxsize, Vec3f TColor, Vec3f XColor, Vec3f YColor,
               Vec3f ZColor, float x, float y, float z)
    : WinName_(std::move(winname))
    , WinWid_(std::move(winWidth))
    , WinHei_(std::move(winHeight))
    , NavState_(nullptr)
    , Traj_(maxsize, TColor)
    , Coord_(XColor, YColor, ZColor, x, y, z)
    , Request_(false)
    , IsStop_(false)
    , NewState_(false) {

    pangolin::CreateWindowAndBind(WinName_, WinWid_, WinHei_);
    pangolin::GetBoundWindow()->RemoveCurrent();
}

/**
 * @brief 更新导航状态
 * @details
 *      1. 添加最新的导航状态到轨迹队列中
 *      2. 更新坐标系的位姿状态
 * @param NavState
 */
void Viewer::UpdateNavState(NavStateSharedPtr NavState) {
    std::lock_guard<std::mutex> lock(Mtx_);
    NavState_ = NavState;
    NewState_ = true;
}

/**
 * @brief 可视化线程函数
 * @details
 *      1. 渲染轨迹
 *      2. 渲染坐标系
 */
void Viewer::Render() {
    pangolin::BindToContext(WinName_);
    glEnable(GL_DEPTH_TEST);
    pangolin::OpenGlRenderState renderStatus(
        pangolin::ProjectionMatrix(WinWid_, WinHei_, 800, 800, WinWid_ / 2, WinHei_ * 0.6, 0.1, 1000),
        pangolin::ModelViewLookAt(0, 0, 700, 0, 0, 0, pangolin::AxisY));
    pangolin::Handler3D handler(renderStatus);
    pangolin::View &view =
        pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, WinWid_ / (double)WinHei_).SetHandler(&handler);
    while (!pangolin::ShouldQuit() && !Request_.load()) {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        view.Activate(renderStatus);
        {
            std::lock_guard<std::mutex> lock(Mtx_);

            if (NewState_) {
                Vec3f pos = NavState_->p_.translation().cast<float>();
                Traj_.AddPt(pos);
                Coord_.UpdateSE3(NavState_->p_.cast<float>());
                NewState_ = false;
            }

            if (NavState_) {
                Coord_.Render();
                Traj_.Render();
            }
        }
        pangolin::FinishFrame();
        std::this_thread::sleep_for(30ms);
    }
    pangolin::GetBoundWindow()->RemoveCurrent(); ///< 解绑当前窗口
    IsStop_.store(true);
}

} // namespace insystem
