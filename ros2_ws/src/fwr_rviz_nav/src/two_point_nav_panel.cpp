#include <cstdio>
#include <cmath>
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <QComboBox>
#include <QLabel>
#include <QWidget>
#include <atomic>
#include <thread>

#include <QApplication>

// ---------------------------------------------------------------------------
// 直接向 X 服务器查询"物理按键是否按下"（RViz 里长按只收到 ~0.3 s 的按键事件，
// 只看事件永远判不出"按住"）。这里只声明用到的几个 Xlib 符号，
// 不包含 X11 头文件 —— 它会把 None/Bool/KeyPress 等定义成宏，和 Qt 冲突。
// ---------------------------------------------------------------------------
extern "C" {
struct _XDisplay;
typedef struct _XDisplay Display;
Display * XOpenDisplay(const char * name);
int XCloseDisplay(Display * display);
int XQueryKeymap(Display * display, char keys[32]);
unsigned long XKeysymToKeycode(Display * display, unsigned long keysym);
}
#include <QCheckBox>
#include <QDateTime>
#include <QEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QShortcut>
#include <QTimer>
#include <QKeySequence>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <fwr_interfaces/srv/start_fixed_goal.hpp>
#include <fwr_interfaces/srv/start_multi_point_goal.hpp>
#include <fwr_interfaces/srv/start_two_point_goal.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fwr_rviz_nav/navigation_route.hpp"
#include "fwr_rviz_nav/point_selection.hpp"
#include "fwr_rviz_nav/real_nav_start_flow.hpp"

namespace fwr_rviz_nav
{
class TwoPointNavPanel : public rviz_common::Panel
{
public:
  explicit TwoPointNavPanel(QWidget * parent = nullptr)
  : TwoPointNavPanel(parent, false) {}

  ~TwoPointNavPanel() override
  {
    if (timer_ != nullptr) {timer_->stop();}
    if (!guarded_) {return;}
    if (stop_client_ && stop_client_->service_is_ready()) {
      stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    if (gate_client_ && gate_client_->service_is_ready()) {
      auto request = motion_gate_request(false);
      auto future = gate_client_->async_send_request(request);
      rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(700));
    }
  }

protected:
  TwoPointNavPanel(QWidget * parent, bool guarded)
  : rviz_common::Panel(parent), node_(std::make_shared<rclcpp::Node>(
      guarded ? "fwr_real_two_point_nav_panel" : "fwr_two_point_nav_panel")), guarded_(guarded)
  {
    auto * layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel(
      "选中工具栏的 Publish Point 后点击地图。\n"
      "点一次：从当前位置前往该点；点两次：第一点为起点、第二点为终点。", this));
    start_label_ = new QLabel("起点：未选择（单点自动用当前位置）", this);
    goal_label_ = new QLabel("终点：未选择", this);
    status_label_ = new QLabel("等待地图点击", this);
    preview_button_ = new QPushButton("重新预览", this);
    start_button_ = new QPushButton("开始导航", this);
    stop_button_ = new QPushButton("停止导航（空格）", this);
    resume_button_ = new QPushButton("继续导航（空格）", this);
    reset_button_ = new QPushButton("清除选择", this);
    recover_button_ = new QPushButton("清空状态 / 复位故障", this);
    // 2026-10-31：建图时现场需要一个「马上停」的按钮（沿墙建图是自动跑的）
    map_start_button_ = new QPushButton("开始建图(自动行走)", this);
    map_stop_button_ = new QPushButton("■ 停止建图(急停)", this);
    save_map_button_ = new QPushButton("保存地图(SLAM)", this);
    // 2026-11-05（用户要求）：
    //   · 导航模式只显示导航按钮；建图模式只显示建图按钮
    //   · 两种模式都显示「清空状态/复位故障」
    //   · 新增「播种定位」：把选中的点作为 AMCL 初始位姿（朝向取当前车头）
    seed_button_ = new QPushButton("重新定位（一键播种）", this);
    seed_button_->setToolTip(
      "把车搬到别处后按这个：系统会用激光和地图重新匹配位置并播种 AMCL，\n"
      "不需要再点 2D Pose Estimate。约需几秒。");
    start_button_->setEnabled(false);
    preview_button_->setEnabled(false);
    for (auto * widget : {start_label_, goal_label_, status_label_}) {
      widget->setWordWrap(true);
      layout->addWidget(widget);
    }
    speed_box_ = new QComboBox(this);
    speed_box_->addItems({"速度 ×0.5", "速度 ×0.75", "速度 ×1.0"});
    speed_box_->setCurrentIndex(2);
    // 模式选择：导航 / 建图（启动时自动判断）
    mode_box_ = new QComboBox(this);
    mode_box_->addItems({"导航模式", "建图模式", "手动模式"});
    layout->addWidget(mode_box_);

    nav_group_ = new QWidget(this);
    auto * nav_layout = new QVBoxLayout(nav_group_);
    nav_layout->setContentsMargins(0, 0, 0, 0);
    nav_layout->addWidget(speed_box_);
    nav_layout->addWidget(preview_button_);
    nav_layout->addWidget(start_button_);
    if (guarded_) {nav_layout->addWidget(stop_button_);}
    if (guarded_) {nav_layout->addWidget(resume_button_);}
    nav_layout->addWidget(seed_button_);
    nav_layout->addWidget(reset_button_);
    layout->addWidget(nav_group_);

    map_group_ = new QWidget(this);
    auto * map_layout = new QVBoxLayout(map_group_);
    map_layout->setContentsMargins(0, 0, 0, 0);
    map_layout->addWidget(map_start_button_);
    map_layout->addWidget(map_stop_button_);
    map_layout->addWidget(save_map_button_);
    layout->addWidget(map_group_);

    // ---- 手动模式（2026-11-12 用户要求）----
    // 键盘操控直接做在面板里：RViz 里没有终端，拉起 teleop_twist_keyboard 也收不到按键。
    manual_group_ = new QWidget(this);
    {
      auto * manual_layout = new QVBoxLayout(manual_group_);
      manual_layout->setContentsMargins(0, 0, 0, 0);
      auto * speed_row = new QHBoxLayout();
      speed_row->addWidget(new QLabel("速度:", this));
      manual_speed_box_ = new QComboBox(this);
      manual_speed_box_->addItems({"0.30 m/s", "0.25 m/s", "0.20 m/s", "0.18 m/s", "0.15 m/s", "0.12 m/s"});
      manual_speed_box_->setCurrentIndex(5);          // 默认最慢 0.12 m/s
      speed_row->addWidget(manual_speed_box_);
      manual_layout->addLayout(speed_row);
      // 方向按钮：**鼠标按住就走，松开就停**。
      // 2026-11-12 实测：RViz 里长按键盘只收到 ~0.15 s 的按键事件（系统键盘重复正常，
      // 但在 RViz 里被截断），所以提供鼠标按住方案 —— Qt 的 pressed/released 100% 可靠。
      auto * pad = new QGridLayout();
      auto make_btn = [this](const char * text) {
        auto * b = new QPushButton(QString::fromUtf8(text), this);
        b->setMinimumHeight(34);
        return b;
      };
      manual_fwd_button_ = make_btn("↑ 前进");
      manual_back_button_ = make_btn("↓ 后退");
      manual_left_button_ = make_btn("← 左转");
      manual_right_button_ = make_btn("右转 →");
      pad->addWidget(manual_fwd_button_, 0, 1);
      pad->addWidget(manual_left_button_, 1, 0);
      pad->addWidget(manual_right_button_, 1, 2);
      pad->addWidget(manual_back_button_, 2, 1);
      manual_layout->addLayout(pad);
      // 按住=走，松开=停；单击也给 1.5 s 的"起步"（万一 pressed 事件被吞也能动）
      // ⚠️ 必须用**指针**捕获成员标志：lambda 里按引用捕获一个引用参数会悬空
      //（捕获到的是栈上的引用变量，函数返回后就是野引用 —— 之前"点了没反应"就是它）。
      auto wire = [this](QPushButton * b, int idx, const char * name) {
        connect(b, &QPushButton::pressed, this, [this, idx, name]() {
          manual_btn_[idx] = true;
          panel_debug_log(std::string("btn ") + name + " down");
        });
        connect(b, &QPushButton::released, this, [this, idx, name]() {
          manual_btn_[idx] = false;
          panel_debug_log(std::string("btn ") + name + " up");
        });
        connect(b, &QPushButton::clicked, this, [this, idx, name]() {
          if (manual_latch_box_ != nullptr && manual_latch_box_->isChecked()) {
            const bool on = !manual_btn_[idx];     // 持续模式：点击切换保持
            for (int i = 0; i < 4; ++i) {manual_kb_[i] = false; manual_btn_[i] = false;}
            manual_btn_[idx] = on;
            panel_debug_log(std::string("btn ") + name + (on ? " latch ON" : " latch OFF"));
            return;
          }
          manual_btn_[idx] = true;            // 单击兜底：1.5 s 后自动停
          const int gen = ++manual_click_gen_;
          panel_debug_log(std::string("btn ") + name + " clicked -> 1.5s");
          QTimer::singleShot(1500, this, [this, idx, gen]() {
            if (gen == manual_click_gen_) {manual_btn_[idx] = false;}
          });
        });
      };
      wire(manual_fwd_button_, 0, "fwd");
      wire(manual_back_button_, 1, "back");
      wire(manual_left_button_, 2, "left");
      wire(manual_right_button_, 3, "right");

      manual_latch_box_ = new QCheckBox("持续模式（按一下保持，再按同一键停）", this);
      manual_layout->addWidget(manual_latch_box_);
      auto * hint = new QLabel(
        "手动模式：鼠标**按住**方向按钮即走、松开即停；键盘 W/S/A/D 也可。"
        "切回其它模式会自动关闭运动门。", this);
      hint->setWordWrap(true);
      manual_layout->addWidget(hint);
      manual_stop_button_ = new QPushButton("立即停车（清零速度）", this);
      manual_layout->addWidget(manual_stop_button_);
      layout->addWidget(manual_group_);
    }
    if (guarded_) {layout->addWidget(recover_button_);}   // 两种模式都要
    layout->addStretch();

    // 启动时自动判断：有 slam_toolbox 说明在建图
    {
      bool mapping = false;
      for (const auto & name : node_->get_node_names()) {
        if (name.find("slam_toolbox") != std::string::npos) {mapping = true; break;}
      }
      mode_box_->setCurrentIndex(mapping ? 1 : 0);
    }
    apply_mode();
    connect(mode_box_, QOverload<int>::of(&QComboBox::currentIndexChanged),
      this, [this](int) {apply_mode();});

    auto qos = rclcpp::QoS(1).transient_local();
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/two_point_preview_path", qos);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/two_point_markers", qos);
    // Display-only publisher used to wipe the last Nav2 global plan from RViz
    // when the operator clears the two points. Only the RViz Path display
    // subscribes to /plan; planner_server overwrites it on the next plan.
    plan_clear_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan", rclcpp::QoS(1));
    manual_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);
    // 手动模式：20 Hz 把按键状态发到 /cmd_vel_manual（松手就是 0）
    manual_gate_timer_ = new QTimer(this);
    manual_gate_timer_->setInterval(1000);
    connect(manual_gate_timer_, &QTimer::timeout, this, [this]() {
      if (manual_active_ && guarded_) {set_manual_gates(true);}   // 保活：栈重启后自动恢复
    });
    manual_timer_ = new QTimer(this);
    manual_timer_->setInterval(50);
    connect(manual_timer_, &QTimer::timeout, this, [this]() {manual_tick();});
    if (manual_stop_button_ != nullptr) {
      connect(manual_stop_button_, &QPushButton::clicked, this, [this]() {
        clear_manual_keys();
        if (manual_vel_pub_) {manual_vel_pub_->publish(geometry_msgs::msg::Twist());}
        if (status_label_ != nullptr) {status_label_->setText("已清零速度（手动模式保持）");}
      });
    }
    base_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, "/base_bridge");
    mapping_save_client_ = node_->create_client<std_srvs::srv::Trigger>("/mapping/save");
    // SLAM Toolbox 一键保存：同时写 posegraph 和栅格图。Cartographer 的
    // /mapping/save 不适用于当前手动建图链路。
    slam_save_client_ = node_->create_client<std_srvs::srv::Trigger>("/mapping/save_slam");
    // 2026-11-03：全局"建图运行中"开关。停止建图时必须让探索器/监督器一起停，
    // 否则它们会自己把运动门重新打开 → 车又动起来（曾因此扯断 USB 线）。
    // 用 transient_local，后启动的节点也能收到当前状态。
    mapping_run_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/mapping/run", rclcpp::QoS(1).transient_local());
    click_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (msg->header.frame_id != "map") {
          status_label_->setText("请把 RViz Fixed Frame 设为 map 后再选点");
          return;
        }
        if (guarded_ && flow_.pending_or_active()) {
          stop_real_navigation("已改变点位，原任务已请求停止");
        }
        points_.click(msg->point.x, msg->point.y);
        ++selection_revision_;
        preview_ready_ = false;
        start_button_->setEnabled(false);
        preview_button_->setEnabled(points_.has_goal());
        // 2026-11-05：选点后**自动预览**。原来选完点"开始导航"一直是灰的，
        // 必须先点【重新预览】才能开始，现场看起来像"选了目标却不走"。
        // 每次点击都会带新的 selection_revision，旧预览结果会被丢弃，所以直接调是安全的。
        if (points_.has_goal()) {request_preview();}
        publish_empty_path();
        publish_markers();
        refresh_labels();
        if (points_.has_goal()) {
          request_preview();
        } else {
          status_label_->setText("等待地图点击");
        }
      });
    plan_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      node_, "/compute_path_to_pose");
    // 2026-11-03：多点路线的预览要连所有途经点，用"穿过多个点"的规划动作。
    through_plan_client_ =
      rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(
      node_, "/compute_path_through_poses");
    if (guarded_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
      gate_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
        "/base_bridge/set_parameters");
      real_nav_client_ = node_->create_client<fwr_interfaces::srv::StartTwoPointGoal>(
        "/explore/start_two_point_goal");
      // 2026-11-03 多点导航（3 个点及以上才走这条路；1 点/2 点仍用原来的流程）
      multi_nav_client_ = node_->create_client<fwr_interfaces::srv::StartMultiPointGoal>(
        "/explore/start_multi_point_goal");
      real_single_goal_client_ = node_->create_client<fwr_interfaces::srv::StartFixedGoal>(
        "/explore/start_fixed_goal");
      arm_client_ = node_->create_client<std_srvs::srv::Trigger>("/base/arm_auto");
      stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/explore/stop");
      // 2026-11-05：继续任务优先用节点的进度（本该走的下一段），而不是按最近点重算。
      resume_client_ = node_->create_client<std_srvs::srv::Trigger>("/explore/resume");
      clear_fault_client_ = node_->create_client<std_srvs::srv::Trigger>("/base/clear_fault");
      speed_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
        "/auto_command_gate/set_parameters");
      ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/base/ready", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          base_ready_ = msg->data;
        });
      fault_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/base/fault", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          base_fault_ = msg->data;
        });
      connect(recover_button_, &QPushButton::clicked, this, [this]() {recover_state();});
    connect(speed_box_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int index) {
          if (!speed_client_->service_is_ready()) {
            status_label_->setText("速度服务未就绪");
            return;
          }
          // ×1.25（0.20 m/s）超出驱动在负载下能维持的能力，会触发堵转保护，上限收到 ×1.0。
          static const double scales[] = {0.5, 0.75, 1.0};
          auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
          rcl_interfaces::msg::Parameter parameter;
          parameter.name = "speed_scale";
          parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          parameter.value.double_value = scales[index];
          request->parameters.push_back(parameter);
          speed_client_->async_send_request(request);
          status_label_->setText(QString("速度倍率 ×%1").arg(scales[index]));
        });
      notice_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/explore/notice", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          const QString text = QString::fromStdString(msg->data);
          status_label_->setText(text);
          // 断流/授权丢失暂停：保持运动门打开，恢复后由主控自动继续
          if (text.contains("暂停")) {sensor_pause_ = true;}
          if (text.contains("恢复") || text.contains("继续导航")) {sensor_pause_ = false;}
        });
      state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/explore/state", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          if (sensor_pause_) {return;}   // 断流暂停中：主控会自己恢复，不关门
          if (real_task_active_ && flow_.explore_state(msg->data == "stopped")) {
            // 任务结束的原因要看得见：带上当前故障码，操作者不用去翻日志。
            if (base_fault_ != 0) {
              stop_real_navigation(QString(
                "任务已结束（fault=%1：堵转/超速保护），已关闭运动门；"
                "点「清空状态 / 复位故障」可恢复").arg(base_fault_));
            } else {
              stop_real_navigation("任务已结束，底盘运动门已关闭");
            }
          }
        });
    } else {
      nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_, "/navigate_to_pose");
    }

    connect(preview_button_, &QPushButton::clicked, this, [this]() {request_preview();});
    connect(seed_button_, &QPushButton::clicked, this, [this]() {seed_localization();});
    connect(start_button_, &QPushButton::clicked, this, [this]() {start_navigation();});
    connect(resume_button_, &QPushButton::clicked, this, [this]() {resume_navigation();});
    connect(map_stop_button_, &QPushButton::clicked, this, [this]() {
      // 先急停，再让 mapping_save 结束轨迹并存盘（pbstream + pgm）
      if (manual_vel_pub_) {
        manual_vel_pub_->publish(geometry_msgs::msg::Twist());
      }
      set_base_gates(false);
      publish_mapping_run(false);          // 通知探索器/监督器一起停
      if (!mapping_save_client_ || !mapping_save_client_->service_is_ready()) {
        status_label_->setText("已停止；但 /mapping/save 服务不可用，地图未保存");
        return;
      }
      status_label_->setText("已停止，正在保存地图…");
      // 只调用一次！保存会结束轨迹，重复调用没有意义
      auto future = mapping_save_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
          const auto response = f.get();
          if (response) {
            status_label_->setText(
              QString::fromStdString("地图已保存：" + response->message));
          }
        });
      (void)future;
    });
    connect(save_map_button_, &QPushButton::clicked, this, [this]() {
      // 先把手动/自动运动都停住，再保存；避免存到运动中的瞬时地图。
      if (manual_vel_pub_) {
        manual_vel_pub_->publish(geometry_msgs::msg::Twist());
      }
      set_base_gates(false);
      if (!slam_save_client_ || !slam_save_client_->service_is_ready()) {
        status_label_->setText(
          "保存失败：/mapping/save_slam 不可用；请确认当前是 real_mapping.launch.py");
        return;
      }
      status_label_->setText("已停车，正在保存 SLAM 地图（posegraph + 栅格）…");
      auto future = slam_save_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
          const auto response = f.get();
          if (response && response->success) {
            status_label_->setText(
              QString::fromStdString("地图已保存：" + response->message));
          } else {
            status_label_->setText(QString::fromStdString(
              "保存失败：" + (response ? response->message : "无响应")));
          }
        });
      (void)future;
    });
    connect(map_start_button_, &QPushButton::clicked, this, [this]() {
      publish_mapping_run(true);           // 允许探索
      set_base_gates(true);
      status_label_->setText("已打开运动门（motion/manual/mixed），可继续建图");
    });
    connect(stop_button_, &QPushButton::clicked, this, [this]() {
      if (guarded_) {stop_real_navigation("已请求停止导航，底盘运动门已关闭");}
    });
    // 空格 = 停止/继续（2026-11-05 用户要求：车里导航时用键盘空格直接停/继续）
    // ApplicationShortcut：焦点在 3D 视图里也能收到。
    {
      auto * space = new QShortcut(QKeySequence(Qt::Key_Space), this);
      space->setContext(Qt::ApplicationShortcut);
      connect(space, &QShortcut::activated, this,
        [this]() {toggle_navigation_by_space();});
    }
    connect(reset_button_, &QPushButton::clicked, this, [this]() {
      if (guarded_ && flow_.pending_or_active()) {
        stop_real_navigation("已请求停止导航，底盘运动门已关闭");
      }
      points_.clear();
      ++selection_revision_;
      // 2026-11-15 现场："清除选择 → 重新选点 → 开始"后仍跑旧路线。
      // 原因是 has_resume_goal_/resume_* 还留着，空格/继续会续跑旧任务。
      // 清空选择时必须把"可继续的任务"一起作废。
      has_resume_goal_ = false;
      resume_mode_ = false;
      resume_two_point_ = false;
      resume_multi_ = false;
      ready_retry_ = 0;
      preview_ready_ = false;
      preview_button_->setEnabled(false);
      start_button_->setEnabled(false);
      publish_empty_path();
      publish_markers();
      publish_empty_global_plan();
      refresh_labels();
      status_label_->setText("等待地图点击");
    });
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {rclcpp::spin_some(node_);});
    timer_->start(50);
  }

private:
  using Plan = nav2_msgs::action::ComputePathToPose;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using RealGoal = fwr_interfaces::srv::StartTwoPointGoal;
  using MultiRealGoal = fwr_interfaces::srv::StartMultiPointGoal;
  using RealSingleGoal = fwr_interfaces::srv::StartFixedGoal;
  using Gate = rcl_interfaces::srv::SetParameters;

  static std::shared_ptr<Gate::Request> motion_gate_request(bool enabled)
  {
    auto request = std::make_shared<Gate::Request>();
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "motion_enabled";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = enabled;
    request->parameters.push_back(parameter);
    return request;
  }

  void request_motion_gate(bool enabled, std::function<void(bool)> callback)
  {
    if (!gate_client_ || !gate_client_->service_is_ready()) {
      callback(false);
      return;
    }
    gate_client_->async_send_request(motion_gate_request(enabled),
      [callback](rclcpp::Client<Gate>::SharedFuture future) {
        bool success = false;
        try {
          const auto response = future.get();
          success = response && response->results.size() == 1 && response->results[0].successful;
        } catch (const std::exception &) {
          // The caller reports failure and keeps the goal disabled.
        }
        callback(success);
      });
  }

  void stop_real_navigation(const QString & message)
  {
    flow_.stop();
    real_task_active_ = false;
    base_ready_ = false;
    const auto revision = ++selection_revision_;
    start_button_->setEnabled(false);
    status_label_->setText("正在停机并关闭底盘运动门…");
    if (stop_client_ && stop_client_->service_is_ready()) {
      stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    request_motion_gate(false, [this, revision, message](bool success) {
      if (revision != selection_revision_) {return;}
      status_label_->setText(success ? message :
        "停止请求已发出，但运动门关闭未确认；请立即切断电机电源并检查底盘状态");
      start_button_->setEnabled(success && preview_ready_ && points_.has_goal());
    });
  }

  void fail_real_start(const QString & message, unsigned long revision)
  {
    if (revision != selection_revision_) {return;}
    stop_real_navigation(message);
  }

  void wait_for_base_ready(unsigned long revision, int attempts)
  {
    if (revision != selection_revision_ || !flow_.waiting_ready()) {return;}
    if (base_ready_) {
      if (flow_.ready()) {request_real_arm(revision);}
      return;
    }
    if (attempts >= 50) {
      fail_real_start(QString("底盘或传感器未就绪（fault=%1），已关闭运动门；"
                        "点「清空状态」可复位").arg(base_fault_), revision);
      return;
    }
    QTimer::singleShot(100, this, [this, revision, attempts]() {
      wait_for_base_ready(revision, attempts + 1);
    });
  }

  void request_real_arm(unsigned long revision)
  {
    status_label_->setText("底盘就绪，正在申请自动导航授权…");
    arm_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, revision](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (revision != selection_revision_) {return;}
        try {
          const auto response = future.get();
          if (!flow_.armed(response->success)) {
            fail_real_start(QString("底盘授权失败：%1").arg(
              QString::fromStdString(response->message)), revision);
            return;
          }
          // 授权话题由底盘桥广播，等待探索器收到后再发送目标。
          QTimer::singleShot(400, this, [this, revision]() {
            send_real_navigation_request(revision);
          });
        } catch (const std::exception & error) {
          fail_real_start(QString("底盘授权失败：%1").arg(error.what()), revision);
        }
      });
  }

  geometry_msgs::msg::PoseStamped make_pose(const Point & point) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node_->now();
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  struct MapPose
  {
    double x;
    double y;
    double yaw;
  };

  std::optional<MapPose> current_map_pose() const
  {
    if (!tf_buffer_) {return std::nullopt;}
    for (const auto * frame : {"base_link", "base_footprint"}) {
      try {
        const auto transform = tf_buffer_->lookupTransform("map", frame, tf2::TimePointZero);
        const auto & translation = transform.transform.translation;
        const auto & rotation = transform.transform.rotation;
        const double yaw = std::atan2(
          2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
          1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
        return MapPose{translation.x, translation.y, yaw};
      } catch (const tf2::TransformException &) {
        // Try the alternate robot frame before reporting failure.
      }
    }
    return std::nullopt;
  }

  // AMCL 只在收到扫描时更新 map→odom，实车启动瞬间偶尔有 100–200 ms 空档；
  // 单次 lookup 失败就关运动门太激进，这里最多重试 1 秒。
  std::optional<MapPose> wait_map_pose(int attempts = 10) const
  {
    for (int i = 0; i < attempts; ++i) {
      if (auto pose = current_map_pose()) {return pose;}
      if (i + 1 < attempts) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }
    return std::nullopt;
  }

  // 2026-10-31：建图急停/恢复用，直接改 base_bridge 的三道开关
  void publish_mapping_run(bool running)
  {
    if (!mapping_run_pub_) {return;}
    std_msgs::msg::Bool msg;
    msg.data = running;
    mapping_run_pub_->publish(msg);
  }

  void set_base_gates(bool enabled)
  {
    if (!base_param_client_ || !base_param_client_->service_is_ready()) {
      status_label_->setText("base_bridge 参数服务不可用");
      return;
    }
    // 2026-11-03 修"点开始建图没反应"：
    // 原来把 manual_enabled 也一起打开，而 base_bridge / auto_command_gate 的规则是
    // "manual_enabled=True 时自动链路被封锁" → Nav2 的指令全被清零，车不动。
    // 建图/导航只需要 motion_enabled；手动通道留给探索器自己的脱困用。
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("motion_enabled", enabled),
      rclcpp::Parameter("manual_enabled", false),
      rclcpp::Parameter("manual_allow_mixed", false)};
    base_param_client_->set_parameters(params);
  }

  void recover_state()
  {
    ++selection_revision_;
    real_task_active_ = false;
    status_label_->setText("正在复位：关闭运动门并清除故障…");
    // 固件只在“运动路径全部禁用”时才接受清故障，所以两道门都要关。
    if (gate_client_ && gate_client_->service_is_ready()) {
      auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      for (const auto & name : {"motion_enabled", "manual_enabled"}) {
        rcl_interfaces::msg::Parameter parameter;
        parameter.name = name;
        parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        parameter.value.bool_value = false;
        request->parameters.push_back(parameter);
      }
      gate_client_->async_send_request(request);
    }
    if (!clear_fault_client_ || !clear_fault_client_->service_is_ready()) {
      status_label_->setText("复位失败：/base/clear_fault 服务不可用");
      return;
    }
    clear_fault_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response->success) {
            status_label_->setText(QString("复位失败：%1").arg(
              QString::fromStdString(response->message)));
            return;
          }
        } catch (const std::exception & error) {
          status_label_->setText(QString("复位失败：%1").arg(error.what()));
          return;
        }
        points_.clear();
        has_resume_goal_ = false;
        resume_mode_ = false;
        resume_two_point_ = false;
        resume_multi_ = false;
        ready_retry_ = 0;
        preview_ready_ = false;
        start_button_->setEnabled(false);
        preview_button_->setEnabled(false);
        publish_empty_path();
        publish_markers();
        refresh_labels();
        status_label_->setText("已复位：请重新选点并预览");
      });
  }

  void selected_revision_guard() {}

  /// 手动模式/键盘调试日志（写到开发机 /tmp/fwr_panel_debug.log）
  static void panel_debug_log(const std::string & line)
  {
    static FILE * fp = std::fopen("/tmp/fwr_panel_debug.log", "a");
    if (fp == nullptr) {return;}
    std::fprintf(fp, "%.3f %s\n", std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count(), line.c_str());
    std::fflush(fp);
  }

  void apply_mode()
  {
    const int mode = (mode_box_ != nullptr) ? mode_box_->currentIndex() : 0;  // 0 导航 1 建图 2 手动
    const bool mapping = (mode == 1);
    const bool manual = (mode == 2);
    if (nav_group_ != nullptr) {nav_group_->setVisible(mode == 0);}
    if (map_group_ != nullptr) {map_group_->setVisible(mapping);}
    if (manual_group_ != nullptr) {manual_group_->setVisible(manual);}
    if (start_label_ != nullptr) {start_label_->setVisible(!mapping && !manual);}
    if (goal_label_ != nullptr) {goal_label_->setVisible(!mapping && !manual);}
    if (manual != manual_active_) {
      if (manual) {enter_manual_mode();} else {leave_manual_mode();}
    }
  }

  // ---------- 手动模式 ----------
  void enter_manual_mode()
  {
    // 先停掉可能正在跑的导航任务（手动通道和导航任务互斥）
    if (guarded_ && flow_.pending_or_active()) {
      stop_real_navigation("切换到手动模式，已停止导航任务");
    }
    clear_manual_keys();
    manual_active_ = true;
    if (guarded_) {set_manual_gates(true);}
    if (manual_timer_ != nullptr) {manual_timer_->start();}
    if (!manual_pub_thread_.joinable()) {
      manual_pub_run_ = true;
      manual_pub_thread_ = std::thread([this]() {
        Display * disp = XOpenDisplay(nullptr);
        const unsigned long syms[4][2] = {
          {0x0077UL, 0xff52UL}, {0x0073UL, 0xff54UL},
          {0x0061UL, 0xff51UL}, {0x0064UL, 0xff53UL}};
        while (manual_pub_run_.load()) {
          bool phys[4] = {false, false, false, false};
          if (disp != nullptr) {
            char keys[32] = {0};
            XQueryKeymap(disp, keys);
            for (int i = 0; i < 4; ++i) {
              for (int k = 0; k < 2; ++k) {
                const unsigned long kc = XKeysymToKeycode(disp, syms[i][k]);
                if (kc != 0 && (keys[kc / 8] & (1 << (kc % 8)))) {phys[i] = true;}
              }
            }
          }
          const double v = manual_pub_speed_.load();
          geometry_msgs::msg::Twist cmd;
          if (phys[0] || manual_btn_[0].load()) {cmd.linear.x += v;}
          if (phys[1] || manual_btn_[1].load()) {cmd.linear.x -= v;}
          if (phys[2] || manual_btn_[2].load()) {cmd.angular.z += 0.5;}
          if (phys[3] || manual_btn_[3].load()) {cmd.angular.z -= 0.5;}
          if (manual_vel_pub_) {manual_vel_pub_->publish(cmd);}
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (disp != nullptr) {XCloseDisplay(disp);}
      });
    }
    if (manual_gate_timer_ != nullptr) {manual_gate_timer_->start();}
    if (qApp != nullptr) {qApp->installEventFilter(this);}
    if (x_display_ == nullptr) {x_display_ = XOpenDisplay(nullptr);}
    panel_debug_log("enter_manual_mode");
    if (status_label_ != nullptr) {
      status_label_->setText("手动模式：W/S 前进后退，A/D 转向；松手即停");
    }
  }

  void leave_manual_mode()
  {
    manual_active_ = false;
    clear_manual_keys();
    if (manual_timer_ != nullptr) {manual_timer_->stop();}
    manual_pub_run_ = false;
    if (manual_pub_thread_.joinable()) {manual_pub_thread_.join();}
    manual_pub_speed_ = 0.12;
    if (manual_gate_timer_ != nullptr) {manual_gate_timer_->stop();}
    if (qApp != nullptr) {qApp->removeEventFilter(this);}
    if (manual_vel_pub_) {
      manual_vel_pub_->publish(geometry_msgs::msg::Twist());   // 先发一帧零速
    }
    if (guarded_) {set_manual_gates(false);}
    if (x_display_ != nullptr) {XCloseDisplay(x_display_); x_display_ = nullptr;}
    panel_debug_log("leave_manual_mode");
    if (status_label_ != nullptr) {status_label_->setText("已退出手动模式，运动门已关闭");}
  }

  /// 手动模式：开运动门 + 开手动通道（导航模式下这两个都是关的）
  void set_manual_gates(bool enabled)
  {
    if (!base_param_client_ || !base_param_client_->service_is_ready()) {
      if (status_label_ != nullptr) {status_label_->setText("base_bridge 参数服务不可用");}
      return;
    }
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("motion_enabled", enabled),
      rclcpp::Parameter("manual_enabled", enabled),
      rclcpp::Parameter("manual_allow_mixed", false)};
    base_param_client_->set_parameters(params);
  }

  static bool is_manual_key(int key)
  {
    return key == Qt::Key_W || key == Qt::Key_S || key == Qt::Key_A || key == Qt::Key_D ||
           key == Qt::Key_Up || key == Qt::Key_Down || key == Qt::Key_Left || key == Qt::Key_Right;
  }

  void set_manual_key(int key, bool pressed)
  {
    const char * what = nullptr;
    int idx = -1;
    switch (key) {
      case Qt::Key_W: case Qt::Key_Up: idx = 0; what = "前进"; break;
      case Qt::Key_S: case Qt::Key_Down: idx = 1; what = "后退"; break;
      case Qt::Key_A: case Qt::Key_Left: idx = 2; what = "左转"; break;
      case Qt::Key_D: case Qt::Key_Right: idx = 3; what = "右转"; break;
      default: break;
    }
    if (idx >= 0) {
      const bool latch = manual_latch_box_ != nullptr && manual_latch_box_->isChecked();
      if (latch) {
        // 持续模式：按一下切换保持。
        // ⚠️ 必须"松开后才允许再次切换"，否则键盘自动重复每 30 ms 触发一次切换，
        // 按住 W 会变成 开/关/开/关 → 现场就是"有前进但一卡一卡"。
        const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
        if (pressed) {
          // 只有"真正松开过 >=300 ms"之后的按下才算一次新点击。
          // 键盘自动重复是 30 ms 一对 down/up，间隔远小于 300 ms，不会触发切换。
          const bool fresh = now_ms - manual_latch_release_ms_[idx] > 300;
          if (fresh) {
            const bool on = !manual_kb_[idx];
            for (int i = 0; i < 4; ++i) {manual_kb_[i] = false; manual_btn_[i] = false;}
            manual_kb_[idx] = on;
            manual_latch_release_ms_[idx] = now_ms;   // 之后的自动重复不再触发切换
            panel_debug_log("latch toggle");
          }
        } else {
          manual_latch_release_ms_[idx] = now_ms;
        }
      } else {
        manual_kb_[idx] = pressed;
        if (pressed) {
          // RViz 里长按只能收到 ~0.3 s 的按键重复，所以给 120 ms 自动续期
          manual_kb_until_ms_[idx] = QDateTime::currentMSecsSinceEpoch() + 120;
        }
      }
    }
    panel_debug_log(std::string("key ") + (what ? what : "?") + (pressed ? " down" : " up"));
    // 可见反馈：确认键盘事件到底有没有被面板收到（调试手动模式"不动"时最关键）
    if (what != nullptr && status_label_ != nullptr) {
      status_label_->setText(QString("手动模式：%1%2（速度 %3 m/s）")
        .arg(QString::fromUtf8(what))
        .arg(pressed ? "" : " 松开")
        .arg(manual_linear_speed(), 0, 'f', 2));
    }
  }

  void clear_manual_keys()
  {
    for (int i = 0; i < 4; ++i) {manual_kb_[i] = manual_btn_[i] = false;}
  }

  /// 手动模式档位：0.30 / 0.25 / 0.20 / 0.18 / 0.15 / 0.12 m/s（下拉框顺序）
  double manual_linear_speed() const
  {
    static const double kSpeeds[6] = {0.30, 0.25, 0.20, 0.18, 0.15, 0.12};
    const int idx = (manual_speed_box_ != nullptr) ? manual_speed_box_->currentIndex() : 5;
    return kSpeeds[(idx >= 0 && idx < 6) ? idx : 5];
  }

  void manual_tick()
  {
    if (!manual_active_ || !manual_vel_pub_) {return;}
    const double v = manual_linear_speed();
    const double w = 0.5;                       // 转向角速度固定 0.5 rad/s
    geometry_msgs::msg::Twist cmd;
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    // 直接向 X 服务器查询"这个键此刻是否物理按下"。
    // RViz 里长按只收到 ~0.3 s 的按键事件（之后自动重复被截断），只看事件永远判不出"按住"。
    bool phys[4] = {false, false, false, false};
    if (x_display_ != nullptr) {
      // KeySym 常量值见 X11/keysymdef.h
      static const unsigned long syms[4][2] = {
        {0x0077UL, 0xff52UL},   // w, Up
        {0x0073UL, 0xff54UL},   // s, Down
        {0x0061UL, 0xff51UL},   // a, Left
        {0x0064UL, 0xff53UL}};  // d, Right
      char keys[32] = {0};
      XQueryKeymap(x_display_, keys);
      for (int i = 0; i < 4; ++i) {
        for (int k = 0; k < 2; ++k) {
          const unsigned long kc = XKeysymToKeycode(x_display_, syms[i][k]);
          if (kc != 0 && (keys[kc / 8] & (1 << (kc % 8)))) {phys[i] = true;}
        }
      }
    }
    auto on = [this, now_ms, &phys](int i) {
      return manual_kb_[i] || manual_btn_[i] || phys[i] || now_ms < manual_kb_until_ms_[i];
    };
    if (on(0)) {cmd.linear.x += v;}
    if (on(1)) {cmd.linear.x -= v;}
    if (on(2)) {cmd.angular.z += w;}
    if (on(3)) {cmd.angular.z -= w;}
    manual_pub_speed_ = v;                 // 发布交给独立线程
    panel_debug_log("pub " + std::to_string(cmd.linear.x) + " " + std::to_string(cmd.angular.z));
  }

  bool eventFilter(QObject * obj, QEvent * event) override
  {
    if (manual_active_ && event != nullptr) {
      const auto type = event->type();
      if (type == QEvent::KeyPress || type == QEvent::KeyRelease) {
        auto * key_event = static_cast<QKeyEvent *>(event);
        if (is_manual_key(key_event->key())) {
          const int key = key_event->key();
          if (type == QEvent::KeyPress) {
            manual_release_at_[key] = 0;            // 取消待决的松手
            set_manual_key(key, true);
          } else {
            // 2026-11-12 实测：按住键时 Qt 每 ~30 ms 发一对 down/up，
            // 而且 up 的 isAutoRepeat() 在这里是 false → 不能用它判断。
            // 改成"松手去抖"：收到 up 先记时间，90 ms 内没有新的 down 才真正清零。
            const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
            manual_release_at_[key] = now_ms;
            QTimer::singleShot(90, this, [this, key, now_ms]() {
              auto it = manual_release_at_.find(key);
              if (it != manual_release_at_.end() && it->second == now_ms) {
                set_manual_key(key, false);
              }
            });
          }
          return true;
        }
      } else if (type == QEvent::FocusOut || type == QEvent::WindowDeactivate) {
        clear_manual_keys();                 // 失焦立刻停，防止"按键卡住"
      }
    }
    return rviz_common::Panel::eventFilter(obj, event);
  }

  /// 一键重新定位：调 auto_initial_pose 的 /localization/reseed 服务，
  /// 由它用激光与地图重新匹配并把结果播种给 AMCL（车被搬动后使用）。
  void seed_localization()
  {
    if (!reseed_client_) {
      reseed_client_ = node_->create_client<std_srvs::srv::Trigger>("/localization/reseed");
    }
    if (!reseed_client_->service_is_ready()) {
      if (status_label_ != nullptr) {
        status_label_->setText("重新定位：服务未就绪（auto_initial_pose 没在跑）");
      }
      return;
    }
    if (status_label_ != nullptr) {status_label_->setText("重新定位：先关运动门…");}
    // 2026-11-12：/localization/reseed 要求"运动路径已关闭 + 串口正常"。
    // 面板原来直接调用，导航刚停下时运动门还开着 → 现场报
    // "复位失败: motion paths must be disabled and serial connected"。
    // 这里先主动关门，等 0.6 s 让底盘落地，再请求播种。
    if (guarded_) {set_manual_gates(false);}   // motion_enabled / manual_enabled 都置 false
    QTimer::singleShot(600, this, [this]() {
      if (status_label_ != nullptr) {status_label_->setText("重新定位：正在匹配地图（约 1 秒）…");}
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      reseed_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto response = future.get();
          if (status_label_ != nullptr) {
            // 服务会等匹配完成再返回，直接把结果（含坐标）显示出来
            status_label_->setText(QString::fromStdString(response->message));
          }
        });
    });
  }

  void refresh_labels()
  {
    if (points_.single()) {
      start_label_->setText("起点：当前位置（自动）");
    } else if (points_.start()) {
      start_label_->setText(QString("起点：(%1, %2)")
        .arg(points_.start()->x, 0, 'f', 2).arg(points_.start()->y, 0, 'f', 2));
    } else {
      start_label_->setText("起点：未选择（单点自动用当前位置）");
    }
    goal_label_->setText(points_.goal() ?
      QString("终点：(%1, %2)").arg(points_.goal()->x, 0, 'f', 2).arg(points_.goal()->y, 0, 'f', 2) :
      "终点：未选择");
    if (points_.size() >= 3) {
      start_label_->setText(QString("起点：(%1, %2)｜途经 %3 个点")
        .arg(points_.start()->x, 0, 'f', 2).arg(points_.start()->y, 0, 'f', 2)
        .arg(static_cast<int>(points_.size()) - 2));
    }
  }

  void publish_empty_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    path_pub_->publish(path);
  }

  void publish_empty_global_plan()
  {
    if (plan_clear_pub_ == nullptr) {return;}
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    plan_clear_pub_->publish(path);
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    auto add_marker = [&](const Point & point, int id, float red, float green, float blue) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = node_->now();
      marker.ns = "two_point_navigation";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = point.x;
      marker.pose.position.y = point.y;
      marker.pose.position.z = 0.12;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.22;
      marker.color.a = 1.0;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      markers.markers.push_back(marker);
    };
    // 2026-11-03：多点路线把所有点都画出来（起点绿 / 途经黄 / 终点红）
    const auto & pts = points_.points();
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const bool is_last = (i + 1 == pts.size());
      const bool is_first = (i == 0);
      float r = 0.9f, g = 0.85f, b = 0.1f;          // 途经点：黄
      if (is_first && pts.size() >= 2) {r = 0.1f; g = 0.9f; b = 0.2f;}   // 起点：绿
      if (is_last) {r = 0.9f; g = 0.15f; b = 0.1f;}                       // 终点：红
      add_marker(pts[i], static_cast<int>(i), r, g, b);
    }
    marker_pub_->publish(markers);
  }

  void request_single_preview(unsigned long revision)
  {
    const auto request = make_single_preview_goal(points_, node_->now());
    if (!request) {return;}
    status_label_->setText("正在计算当前位置到终点的路径…");
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.result_callback = [this, revision](const rclcpp_action::ClientGoalHandle<Plan>::WrappedResult & result) {
      if (revision != selection_revision_) {return;}
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
        result.result->path.poses.empty())
      {
        // 2026-11-05：把"为什么失败"说清楚。常见两种：
        //   · 终点贴在墙/家具旁，落在膨胀区里（代价 93~100）→ 换个点
        //   · 终点附近恰好有激光临时标记 → 过一两秒标记清除后就能规划
        status_label_->setText(
          "单点规划失败：终点可能落在墙/家具的膨胀区（离墙 <0.5 m），"
          "或附近有临时障碍标记。请换个离墙 0.6 m 以上的点，或稍等再点「重新预览」");
        return;
      }
      path_pub_->publish(result.result->path);
      preview_ready_ = true;
      start_button_->setEnabled(true);
      status_label_->setText(QString(
        "单点预览完成：%1 个路径点。开始后从当前位置前往终点；再点一次可改为起点/终点。")
        .arg(result.result->path.poses.size()));
    };
    plan_client_->async_send_goal(*request, options);
  }

  void request_preview()
  {
    if (!points_.has_goal()) {return;}
    if (!plan_client_->action_server_is_ready()) {
      status_label_->setText("规划服务尚未就绪；启动 navigation:=true 后点“重新预览”");
      return;
    }
    preview_ready_ = false;
    start_button_->setEnabled(false);
    const auto revision = selection_revision_;
    if (points_.single()) {
      request_single_preview(revision);
      return;
    }
    status_label_->setText("正在计算两点之间的路径…");
    if (guarded_) {
      request_real_preview(revision);
      return;
    }
    Plan::Goal request;
    request.start = make_pose(*points_.start());
    request.goal = make_pose(*points_.goal());
    request.use_start = true;
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.result_callback = [this, revision](const rclcpp_action::ClientGoalHandle<Plan>::WrappedResult & result) {
      if (revision != selection_revision_) {return;}
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
        result.result->path.poses.empty())
      {
        status_label_->setText("规划失败：请检查两点、全局代价地图及 map→odom 定位");
        return;
      }
      path_pub_->publish(result.result->path);
      preview_ready_ = true;
      start_button_->setEnabled(true);
      status_label_->setText(QString(
        "起点到终点预览完成：%1 个路径点。开始后先前往起点；再点一次开始新的单点。")
        .arg(result.result->path.poses.size()));
    };
    plan_client_->async_send_goal(request, options);
  }

  void request_real_preview(unsigned long revision)
  {
    // 2026-11-03（用户要求）：多点路线把中间途经点也连起来。
    // 用 compute_path_through_poses 一次规划"当前位置→起点→…→终点"，
    // 返回的就是一条完整折线，中途不停车。
    if (points_.size() >= 3) {
      if (!through_plan_client_ || !through_plan_client_->action_server_is_ready()) {
        status_label_->setText("多点预览不可用（规划服务未就绪）");
        return;
      }
      auto request = std::make_shared<nav2_msgs::action::ComputePathThroughPoses::Goal>();
      request->goals.reserve(points_.points().size());
      for (const auto & point : points_.points()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = node_->now();
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.w = 1.0;
        request->goals.push_back(pose);
      }
      status_label_->setText(QString("正在规划穿过 %1 个点的路线…")
        .arg(request->goals.size()));
      rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SendGoalOptions options;
      options.result_callback = [this, revision](
        const rclcpp_action::ClientGoalHandle<
          nav2_msgs::action::ComputePathThroughPoses>::WrappedResult & result) {
        if (revision != selection_revision_) {return;}
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
          result.result->path.poses.empty())
        {
          status_label_->setText("多点路线规划失败：请检查各点是否可通行");
          return;
        }
        path_pub_->publish(result.result->path);
        preview_ready_ = true;
        start_button_->setEnabled(true);
        status_label_->setText(QString(
          "多点预览完成：%1 个路径点，经过 %2 个选点；开始后途经点不停车")
          .arg(result.result->path.poses.size()).arg(points_.size()));
      };
      through_plan_client_->async_send_goal(*request, options);
      return;
    }
    const auto legs = make_real_preview_legs(points_, node_->now());
    if (!legs) {return;}
    status_label_->setText("正在规划当前位置到起点…");
    rclcpp_action::Client<Plan>::SendGoalOptions first_options;
    first_options.result_callback = [this, revision, legs](
      const rclcpp_action::ClientGoalHandle<Plan>::WrappedResult & first) {
      if (revision != selection_revision_) {return;}
      if (first.code != rclcpp_action::ResultCode::SUCCEEDED || !first.result ||
        first.result->path.poses.empty())
      {
        status_label_->setText("当前位置到起点规划失败");
        return;
      }
      status_label_->setText("正在规划起点到终点…");
      auto joined = first.result->path;
      const double first_length = path_length_m(joined);
      rclcpp_action::Client<Plan>::SendGoalOptions second_options;
      second_options.result_callback = [this, revision, joined, first_length](
        const rclcpp_action::ClientGoalHandle<Plan>::WrappedResult & second) mutable {
        if (revision != selection_revision_) {return;}
        if (second.code != rclcpp_action::ResultCode::SUCCEEDED || !second.result ||
          second.result->path.poses.empty())
        {
          status_label_->setText("起点到终点规划失败");
          return;
        }
        const double second_length = path_length_m(second.result->path);
        joined.poses.insert(joined.poses.end(), second.result->path.poses.begin(),
          second.result->path.poses.end());
        path_pub_->publish(joined);
        preview_ready_ = true;
        const bool planned = real_preview_legs_planned(first_length, second_length);
        start_button_->setEnabled(planned);
        status_label_->setText(planned ?
          QString("两段预览完成：%1 个路径点；开始后仍需通过实车安全检查；再点一次开始新的单点")
            .arg(joined.poses.size()) :
          QString("两段预览路径无效，暂不可启动"));
      };
      plan_client_->async_send_goal((*legs)[1], second_options);
    };
    plan_client_->async_send_goal((*legs)[0], first_options);
  }

  void start_navigation()
  {
    begin_task(false);
  }

  /// 空格：导航中→停止；已停止且有可继续的任务→继续。
  void toggle_navigation_by_space()
  {
    if (!guarded_) {return;}
    if (flow_.pending_or_active() || real_task_active_) {
      stop_real_navigation("空格：已请求停止导航，底盘运动门已关闭");
      return;
    }
    if (has_resume_goal_) {
      resume_navigation();
      return;
    }
    status_label_->setText("空格：当前没有可停止或可继续的任务");
  }

  void resume_navigation()
  {
    if (!has_resume_goal_) {
      status_label_->setText("还没有可继续的任务：请先选点并开始导航一次");
      return;
    }
    begin_task(true);
  }

  void begin_task(bool resume)
  {
    resume_mode_ = resume;
    if (!resume && !points_.has_goal()) {
      status_label_->setText("还没选点：请用工具栏 Publish Point 在地图上点终点");
      return;
    }
    if (!resume && !preview_ready_) {
      status_label_->setText("预览还没就绪：请先点「重新预览」");
      return;
    }
    if (guarded_) {
      const bool goal_service_ready = (resume || points_.single()) ?
        real_single_goal_client_->service_is_ready() : real_nav_client_->service_is_ready();
      if (!gate_client_->service_is_ready() || !arm_client_->service_is_ready() ||
        !goal_service_ready)
      {
        // 2026-11-15：DDS 发现偶发慢一拍（尤其 WiFi），点一次就报"尚未就绪"
        // 很烦人。这里自动重试 3 次（1.5 s 一次），期间不改变选点。
        if (ready_retry_ < 6) {
          ++ready_retry_;
          QStringList missing;
          if (!gate_client_->service_is_ready()) {missing << "运动门(/base_bridge)";}
          if (!arm_client_->service_is_ready()) {missing << "授权(/base/arm_auto)";}
          if (!goal_service_ready) {
            missing << (points_.single() ? "单点目标服务" : "多点/两点目标服务");
          }
          status_label_->setText(
            QString("正在等待服务就绪（缺少：%1）… 第 %2/6 次")
            .arg(missing.join("、")).arg(ready_retry_));
          const auto rev = selection_revision_;
          const bool resume_flag = resume;
          QTimer::singleShot(2000, this, [this, rev, resume_flag]() {
              if (rev == selection_revision_) {begin_task(resume_flag);}
            });
          return;
        }
        ready_retry_ = 0;
        status_label_->setText("底盘参数、授权或实车导航服务尚未就绪");
        return;
      }
      ready_retry_ = 0;
      if (!flow_.begin()) {return;}
      start_button_->setEnabled(false);
      base_ready_ = false;
      status_label_->setText(resume ?
        "继续导航：正在打开底盘运动门…" : "正在打开底盘运动门并检查传感器…");
      const auto revision = selection_revision_;
      request_motion_gate(true, [this, revision](bool success) {
          if (revision != selection_revision_) {
            // 停止按钮可能先于“开门”回复到达；重新关门消除竞态。
            request_motion_gate(false, [](bool) {});
            return;
          }
          if (!flow_.gate_opened(success)) {
            fail_real_start("无法打开底盘运动门，已取消导航", revision);
            return;
          }
          wait_for_base_ready(revision, 0);
        });
      return;
    }
    if (!nav_client_->action_server_is_ready()) {
      status_label_->setText("导航服务尚未就绪");
      return;
    }
    if (points_.single()) {
      const auto goal = make_single_navigation_goal(points_, node_->now());
      if (!goal) {return;}
      start_button_->setEnabled(false);
      status_label_->setText("正在从当前位置前往终点…");
      send_single_navigation_goal(*goal, selection_revision_);
      return;
    }
    const auto legs = make_navigation_legs(points_, node_->now());
    if (!legs) {return;}
    start_button_->setEnabled(false);
    status_label_->setText("正在从当前位置前往起点…");
    const auto revision = selection_revision_;
    send_navigation_leg(*legs, 0, revision);
  }

  void handle_real_goal_response(
    bool accepted, const std::string & reason, unsigned long revision, bool single)
  {
    if (revision != selection_revision_) {return;}
    if (!flow_.goal_accepted(accepted)) {
      fail_real_start(QString("启动被拒绝：%1").arg(QString::fromStdString(reason)), revision);
      return;
    }
    real_task_active_ = true;
    if (points_.goal()) {
      resume_goal_x_ = points_.goal()->x;
      resume_goal_y_ = points_.goal()->y;
      has_resume_goal_ = true;
      resume_two_point_ = !single;   // 两点任务被中断 → 继续时仍走两点流程
      // 2026-11-05：多点任务（≥3 点）继续时必须重发多点。原来掉进"单点"分支只发终点，
      // 终点往往就在 0.4 m 容差内 → Nav2 瞬间判到达 → 一按继续任务就结束。
      resume_multi_ = (!single && points_.size() >= 3);
    }
    status_label_->setText(single ?
      "已开始：从当前位置前往终点；停止可按下方按钮" :
      "已开始：先到起点，再到终点；停止可按下方按钮");
    // 2026-11-02：原来 2500 ms。换回 Nav2 官方 BT 后，启动阶段要先跑
      // ClearEntireCostmap 等动作，2.5 s 内还没到 RUNNING，面板就误判
      // "任务未进入运行状态"并关门（现场表现为"点了开始没反应"）。
      // 放宽到 6 s。
      QTimer::singleShot(6000, this, [this, revision]() {
      if (revision == selection_revision_ && sensor_pause_) {
        // 断流暂停期间任务必然"没在运行"，顺延检查，等主控恢复后自动继续
        QTimer::singleShot(6000, this, [this, revision]() {
            if (revision == selection_revision_ && sensor_pause_) {return;}
          });
        return;
      }
      if (revision == selection_revision_ && real_task_active_ &&
        !flow_.saw_running_state())
      {
        stop_real_navigation("任务未进入运行状态，已关闭运动门");
      }
    });
  }

  void send_real_navigation_request(unsigned long revision, bool allow_resume_service = true)
  {
    if (revision != selection_revision_) {return;}
    if (allow_resume_service && resume_mode_ && resume_client_ &&
      resume_client_->service_is_ready())
    {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      resume_client_->async_send_request(request, [this, revision](
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (revision != selection_revision_) {return;}
        try {
          const auto response = future.get();
          if (response->success) {
            real_task_active_ = true;
            status_label_->setText(QString("已继续：%1")
              .arg(QString::fromStdString(response->message)));
            return;
          }
          const QString why = QString::fromStdString(response->message);
          if (why.contains("还在进行中")) {
            // 任务其实还在跑（例如连续按了两次"继续"）——重发路线会从第 1 点重走，
            // 这里直接当作"已在运行"处理。
            status_label_->setText(QString("任务仍在运行：%1").arg(why));
            real_task_active_ = true;
            return;
          }
          status_label_->setText(QString("继续失败：%1，正在按路线重发…").arg(why));
          send_real_navigation_request(revision, false);   // 回退：按路线重发
        } catch (const std::exception & error) {
          fail_real_start(QString("继续服务失败：%1").arg(error.what()), revision);
        }
      });
      return;
    }
    // 2026-11-15：只要当前选点 ≥3 个，"继续"也必须重发整条路线。
    // 原来还要求 resume_multi_（上一次任务是多点）→ 否则掉进下面的单点分支，
    // 只发最后一个点（它常在车附近）→ 被近目标保护拒绝/立刻到达。
    if (resume_mode_ && points_.size() >= 3) {
      if (!multi_nav_client_ || !multi_nav_client_->service_is_ready()) {
        fail_real_start("实车多点导航服务尚未就绪，已关闭运动门", revision);
        return;
      }
      const auto & pts = points_.points();
      // 2026-11-15：这里原来用"离车最近的点"当起点，会**静默跳过前面的航点**
      // （现场：4 点路线只发了最后 2 点 → 车直奔倒数第二个点再回终点）。
      // 继续服务失败后的回退 = 按完整路线重发，绝不跳点；真正的"从哪段接着走"
      // 由 frontier_explorer 自己的进度状态机负责。
      const std::size_t first = 0;
      auto request = std::make_shared<MultiRealGoal::Request>();
      for (std::size_t i = first; i < pts.size(); ++i) {
        request->xs.push_back(pts[i].x);
        request->ys.push_back(pts[i].y);
      }
      status_label_->setText(QString("继续导航：从第 %1 个点接着走（共 %2 点）…")
        .arg(first + 1).arg(pts.size()));
      multi_nav_client_->async_send_request(request, [this, revision](
        rclcpp::Client<MultiRealGoal>::SharedFuture future) {
        if (revision != selection_revision_) {return;}
        try {
          const auto response = future.get();
          handle_real_goal_response(response->accepted, response->reason, revision, false);
        } catch (const std::exception & error) {
          fail_real_start(QString("继续多点导航失败：%1").arg(error.what()), revision);
        }
      });
      return;
    }
    if (resume_mode_ && resume_two_point_ && points_.two_point() && points_.start()) {
      // 继续一个被中断的两点任务：重发「起点→终点」，
      // explorer 会从**当前位置**规划到起点，两条腿都不丢。
      if (!real_nav_client_->service_is_ready()) {
        fail_real_start("实车两点导航服务尚未就绪，已关闭运动门", revision);
        return;
      }
      auto req = std::make_shared<RealGoal::Request>();
      req->start_x = points_.start()->x;
      req->start_y = points_.start()->y;
      req->end_x = points_.goal()->x;
      req->end_y = points_.goal()->y;
      status_label_->setText("继续两点任务：先从当前位置前往起点，再到终点…");
      real_nav_client_->async_send_request(req, [this, revision](
        rclcpp::Client<RealGoal>::SharedFuture future) {
        if (revision != selection_revision_) {return;}
        try {
          const auto r = future.get();
          handle_real_goal_response(r->accepted, r->reason, revision, false);
        } catch (const std::exception & e) {
          fail_real_start(QString("启动服务失败：%1").arg(e.what()), revision);
        }
      });
      return;
    }
    if (resume_mode_) {
      if (!real_single_goal_client_->service_is_ready()) {
        fail_real_start("实车单点导航服务尚未就绪，已关闭运动门", revision);
        return;
      }
      const auto current = wait_map_pose();
      if (!current) {
        fail_real_start("无法获取当前位置（map→base_link），已关闭运动门", revision);
        return;
      }
      auto request = std::make_shared<RealSingleGoal::Request>();
      // 2026-11-12：现场出现"面板显示新终点、服务却收到旧终点"——
      // 继续导航用的是 remembered goal，操作者重选点后仍发旧点，
      // 于是被判"终点离车仅 0.29 m"而拒绝。这里以**当前选点**为准：
      // 只要当前选点与记住的终点差 > 0.1 m，就用当前选点。
      double goal_x = resume_goal_x_;
      double goal_y = resume_goal_y_;
      if (points_.has_goal()) {
        const auto selected = points_.goal();   // std::optional<Point>
        if (selected.has_value() &&
          std::hypot(selected->x - goal_x, selected->y - goal_y) > 0.10)
        {
          goal_x = selected->x;
          goal_y = selected->y;
          resume_goal_x_ = goal_x;
          resume_goal_y_ = goal_y;
        }
      }
      request->x = goal_x;
      request->y = goal_y;
      request->yaw = std::atan2(request->y - current->y, request->x - current->x);
      status_label_->setText("正在从当前位置继续前往原终点…");
      real_single_goal_client_->async_send_request(request, [this, revision](
        rclcpp::Client<RealSingleGoal>::SharedFuture future) {
        if (revision != selection_revision_) {return;}
        try {
          const auto response = future.get();
          handle_real_goal_response(response->accepted, response->reason, revision, true);
        } catch (const std::exception & error) {
          fail_real_start(QString("继续导航失败：%1").arg(error.what()), revision);
        }
      });
      return;
    }
    if (!points_.has_goal() || !preview_ready_) {
      fail_real_start("实车导航服务尚未就绪，已关闭运动门", revision);
      return;
    }
    if (points_.single()) {
      if (!real_single_goal_client_->service_is_ready()) {
        fail_real_start("实车单点导航服务尚未就绪，已关闭运动门", revision);
        return;
      }
      const auto current = wait_map_pose();
      if (!current) {
        fail_real_start("无法获取当前位置（map→base_link），已关闭运动门", revision);
        return;
      }
      auto request = std::make_shared<RealSingleGoal::Request>();
      request->x = points_.goal()->x;
      request->y = points_.goal()->y;
      request->yaw = std::atan2(request->y - current->y, request->x - current->x);
      status_label_->setText("正在请求从当前位置到终点的受控导航…");
      real_single_goal_client_->async_send_request(request, [this, revision](
        rclcpp::Client<RealSingleGoal>::SharedFuture future) {
        if (revision != selection_revision_) {return;}
        try {
          const auto response = future.get();
          handle_real_goal_response(response->accepted, response->reason, revision, true);
        } catch (const std::exception & error) {
          fail_real_start(QString("启动服务失败：%1").arg(error.what()), revision);
        }
      });
      return;
    }
    if (points_.size() >= 3) {
      if (!multi_nav_client_ || !multi_nav_client_->service_is_ready()) {
        fail_real_start("实车多点导航服务尚未就绪，已关闭运动门", revision);
        return;
      }
      auto request = std::make_shared<MultiRealGoal::Request>();
      for (const auto & point : points_.points()) {
        request->xs.push_back(point.x);
        request->ys.push_back(point.y);
      }
      status_label_->setText(QString("正在请求受控多点导航（%1 个点，途经点不停车）…")
        .arg(points_.size()));
      multi_nav_client_->async_send_request(request, [this, revision](
        rclcpp::Client<MultiRealGoal>::SharedFuture future) {
        if (revision != selection_revision_) {return;}
        try {
          const auto response = future.get();
          handle_real_goal_response(response->accepted, response->reason, revision, false);
        } catch (const std::exception & error) {
          fail_real_start(QString("启动服务失败：%1").arg(error.what()), revision);
        }
      });
      return;
    }
    if (!real_nav_client_->service_is_ready()) {
      fail_real_start("实车两点导航服务尚未就绪，已关闭运动门", revision);
      return;
    }
    auto request = std::make_shared<RealGoal::Request>();
    request->start_x = points_.start()->x;
    request->start_y = points_.start()->y;
    request->end_x = points_.goal()->x;
    request->end_y = points_.goal()->y;
    status_label_->setText("正在请求受控两点导航…");
    real_nav_client_->async_send_request(request, [this, revision](
      rclcpp::Client<RealGoal>::SharedFuture future) {
      if (revision != selection_revision_) {return;}
      try {
        const auto response = future.get();
        handle_real_goal_response(response->accepted, response->reason, revision, false);
      } catch (const std::exception & error) {
        fail_real_start(QString("启动服务失败：%1").arg(error.what()), revision);
      }
    });
  }

  void send_single_navigation_goal(const Navigate::Goal & goal, unsigned long revision)
  {
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = [this, revision](auto handle) {
      if (revision == selection_revision_ && !handle) {
        status_label_->setText("导航目标被拒绝");
        start_button_->setEnabled(true);
      }
    };
    options.result_callback = [this, revision](
      const rclcpp_action::ClientGoalHandle<Navigate>::WrappedResult & result) {
      if (revision != selection_revision_) {return;}
      status_label_->setText(result.code == rclcpp_action::ResultCode::SUCCEEDED ?
        "已到达终点" : "终点导航未完成，请检查 Nav2 日志和障碍物");
      start_button_->setEnabled(true);
    };
    nav_client_->async_send_goal(goal, options);
  }

  void send_navigation_leg(
    const std::array<Navigate::Goal, 2> & legs, size_t index, unsigned long revision)
  {
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = [this, revision](auto handle) {
      if (revision == selection_revision_ && !handle) {
        status_label_->setText("导航目标被拒绝");
        start_button_->setEnabled(true);
      }
    };
    options.result_callback = [this, legs, index, revision](
      const rclcpp_action::ClientGoalHandle<Navigate>::WrappedResult & result) {
      if (revision != selection_revision_) {return;}
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        status_label_->setText(index == 0 ?
          "未到达起点，终点任务未发送；请检查 Nav2 日志和障碍物" :
          "终点导航未完成，请检查 Nav2 日志和障碍物");
        start_button_->setEnabled(true);
        return;
      }
      if (index == 0) {
        status_label_->setText("已到达起点，正在前往终点…");
        send_navigation_leg(legs, 1, revision);
      } else {
        status_label_->setText("已到达终点");
        start_button_->setEnabled(true);
      }
    };
    nav_client_->async_send_goal(legs[index], options);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_clear_pub_;
  rclcpp_action::Client<Plan>::SharedPtr plan_client_;
  rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
  rclcpp::Client<Gate>::SharedPtr gate_client_;
  rclcpp::Client<RealGoal>::SharedPtr real_nav_client_;
  rclcpp::Client<MultiRealGoal>::SharedPtr multi_nav_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr
    through_plan_client_;
  rclcpp::Client<RealSingleGoal>::SharedPtr real_single_goal_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
  QComboBox * mode_box_ = nullptr;
  QWidget * nav_group_ = nullptr;
  QWidget * map_group_ = nullptr;
  QWidget * manual_group_ = nullptr;
  QPushButton * manual_fwd_button_ = nullptr;
  QPushButton * manual_back_button_ = nullptr;
  QPushButton * manual_left_button_ = nullptr;
  QPushButton * manual_right_button_ = nullptr;
  QPushButton * manual_stop_button_ = nullptr;
  QComboBox * manual_speed_box_ = nullptr;
  QCheckBox * manual_latch_box_ = nullptr;
  QTimer * manual_timer_ = nullptr;
  QTimer * manual_gate_timer_ = nullptr;
  bool manual_active_ = false;
  // 每个方向两个独立来源：键盘(manual_kb_) / 按钮(manual_btn_)，生效值取"或"。
  // 混在一起会出问题：键盘的去抖定时器会把按钮的按住状态清掉（实测按下 200 ms 后被清）。
  bool manual_kb_[4] = {false, false, false, false};    // 0 前 1 后 2 左 3 右
  std::atomic<bool> manual_btn_[4] = {false, false, false, false};
  std::thread manual_pub_thread_;
  std::atomic<bool> manual_pub_run_{false};
  std::atomic<double> manual_pub_speed_{0.12};
  int manual_click_gen_ = 0;
  qint64 manual_latch_release_ms_[4] = {-1000000, -1000000, -1000000, -1000000};  // 上次"真正松开"时刻
  qint64 manual_kb_until_ms_[4] = {0, 0, 0, 0};     // 键盘按下后的有效期（自动续期）
  Display * x_display_ = nullptr;                   // 直接查物理按键（RViz 会截断长按重复）
  std::map<int, qint64> manual_release_at_;
  QPushButton * seed_button_ = nullptr;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reseed_client_;
  QComboBox * speed_box_ = nullptr;
  QPushButton * recover_button_ = nullptr;
  int base_fault_ = 0;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fault_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_fault_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr speed_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_vel_pub_;
  std::shared_ptr<rclcpp::AsyncParametersClient> base_param_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mapping_save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr slam_save_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mapping_run_pub_;
  QPushButton * map_start_button_ = nullptr;
  QPushButton * map_stop_button_ = nullptr;
  QPushButton * save_map_button_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr notice_sub_;
  bool sensor_pause_ = false;   // 主控报告"断流暂停"期间为 true
  PointSelection points_;
  RealNavStartFlow flow_;
  bool guarded_ = false;
  bool real_task_active_ = false;
  QPushButton * resume_button_ = nullptr;
  bool resume_mode_ = false;
  int ready_retry_ = 0;   // 服务就绪检查的自动重试计数
  // 2026-11-02：中止的若是**两点任务**，继续时必须重发两点服务（先去起点再到终点）。
  // 原实现只要 resume 就走单点客户端，直接奔终点，把"先去起点"这一腿丢了。
  bool resume_two_point_ = false;
  bool resume_multi_ = false;
  bool has_resume_goal_ = false;
  double resume_goal_x_ = 0.0;
  double resume_goal_y_ = 0.0;
  bool base_ready_ = false;
  unsigned long selection_revision_ = 0;
  bool preview_ready_ = false;
  QLabel * start_label_;
  QLabel * goal_label_;
  QLabel * status_label_;
  QPushButton * preview_button_;
  QPushButton * start_button_;
  QPushButton * stop_button_;
  QPushButton * reset_button_;
  QTimer * timer_ = nullptr;
};

class RealTwoPointNavPanel : public TwoPointNavPanel
{
public:
  explicit RealTwoPointNavPanel(QWidget * parent = nullptr)
  : TwoPointNavPanel(parent, true) {}
};
}  // namespace fwr_rviz_nav

PLUGINLIB_EXPORT_CLASS(fwr_rviz_nav::TwoPointNavPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(fwr_rviz_nav::RealTwoPointNavPanel, rviz_common::Panel)
