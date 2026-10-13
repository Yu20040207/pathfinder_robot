#include <cmath>
#include <array>
#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fwr_rviz_nav/navigation_route.hpp"
#include "fwr_rviz_nav/point_selection.hpp"

namespace fwr_rviz_nav
{
class TwoPointNavPanel : public rviz_common::Panel
{
public:
  explicit TwoPointNavPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent), node_(std::make_shared<rclcpp::Node>("fwr_two_point_nav_panel"))
  {
    auto * layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel(
      "选中工具栏的 Publish Point，依次点击起点、终点。\n"
      "开始后机器人从当前位置先去起点，再去终点。", this));
    start_label_ = new QLabel("起点：未选择", this);
    goal_label_ = new QLabel("终点：未选择", this);
    status_label_ = new QLabel("等待地图点击", this);
    preview_button_ = new QPushButton("重新预览", this);
    start_button_ = new QPushButton("开始导航", this);
    reset_button_ = new QPushButton("清除两点", this);
    start_button_->setEnabled(false);
    preview_button_->setEnabled(false);
    for (auto * widget : {start_label_, goal_label_, status_label_}) {
      widget->setWordWrap(true);
      layout->addWidget(widget);
    }
    layout->addWidget(preview_button_);
    layout->addWidget(start_button_);
    layout->addWidget(reset_button_);
    layout->addStretch();

    auto qos = rclcpp::QoS(1).transient_local();
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/two_point_preview_path", qos);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/two_point_markers", qos);
    click_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (msg->header.frame_id != "map") {
          status_label_->setText("请把 RViz Fixed Frame 设为 map 后再选点");
          return;
        }
        points_.click(msg->point.x, msg->point.y);
        ++selection_revision_;
        preview_ready_ = false;
        start_button_->setEnabled(false);
        preview_button_->setEnabled(points_.ready());
        publish_empty_path();
        publish_markers();
        refresh_labels();
        if (points_.ready()) {
          request_preview();
        } else {
          status_label_->setText("请选择终点");
        }
      });
    plan_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      node_, "/compute_path_to_pose");
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "/navigate_to_pose");

    connect(preview_button_, &QPushButton::clicked, this, [this]() {request_preview();});
    connect(start_button_, &QPushButton::clicked, this, [this]() {start_navigation();});
    connect(reset_button_, &QPushButton::clicked, this, [this]() {
      points_.clear();
      ++selection_revision_;
      preview_ready_ = false;
      preview_button_->setEnabled(false);
      start_button_->setEnabled(false);
      publish_empty_path();
      publish_markers();
      refresh_labels();
      status_label_->setText("等待地图点击");
    });
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {rclcpp::spin_some(node_);});
    timer->start(50);
  }

private:
  using Plan = nav2_msgs::action::ComputePathToPose;
  using Navigate = nav2_msgs::action::NavigateToPose;

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

  void refresh_labels()
  {
    start_label_->setText(points_.start() ?
      QString("起点：(%1, %2)").arg(points_.start()->x, 0, 'f', 2).arg(points_.start()->y, 0, 'f', 2) :
      "起点：未选择");
    goal_label_->setText(points_.goal() ?
      QString("终点：(%1, %2)").arg(points_.goal()->x, 0, 'f', 2).arg(points_.goal()->y, 0, 'f', 2) :
      "终点：未选择");
  }

  void publish_empty_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    path_pub_->publish(path);
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
    if (points_.start()) {add_marker(*points_.start(), 0, 0.1f, 0.9f, 0.2f);}
    if (points_.goal()) {add_marker(*points_.goal(), 1, 0.9f, 0.15f, 0.1f);}
    marker_pub_->publish(markers);
  }

  void request_preview()
  {
    if (!points_.ready()) {return;}
    if (!plan_client_->action_server_is_ready()) {
      status_label_->setText("规划服务尚未就绪；启动 navigation:=true 后点“重新预览”");
      return;
    }
    preview_ready_ = false;
    start_button_->setEnabled(false);
    status_label_->setText("正在计算两点之间的路径…");
    const auto revision = selection_revision_;
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
      status_label_->setText(QString("起点到终点预览完成：%1 个路径点。开始后先前往起点。")
        .arg(result.result->path.poses.size()));
    };
    plan_client_->async_send_goal(request, options);
  }

  void start_navigation()
  {
    if (!points_.ready() || !preview_ready_) {return;}
    if (!nav_client_->action_server_is_ready()) {
      status_label_->setText("导航服务尚未就绪");
      return;
    }
    const auto legs = make_navigation_legs(points_, node_->now());
    if (!legs) {return;}
    start_button_->setEnabled(false);
    status_label_->setText("正在从当前位置前往起点…");
    const auto revision = selection_revision_;
    send_navigation_leg(*legs, 0, revision);
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
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<Plan>::SharedPtr plan_client_;
  rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
  PointSelection points_;
  unsigned long selection_revision_ = 0;
  bool preview_ready_ = false;
  QLabel * start_label_;
  QLabel * goal_label_;
  QLabel * status_label_;
  QPushButton * preview_button_;
  QPushButton * start_button_;
  QPushButton * reset_button_;
};
}  // namespace fwr_rviz_nav

PLUGINLIB_EXPORT_CLASS(fwr_rviz_nav::TwoPointNavPanel, rviz_common::Panel)
