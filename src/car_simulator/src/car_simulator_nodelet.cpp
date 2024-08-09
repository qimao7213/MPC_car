#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <deque>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

struct Car {
  double l;
  Eigen::Vector4d state;
  // state: x, y, phi, v
  // input: a, delta
  inline void setInitialState(const Eigen::Vector4d& s) {
    state = s;
  }
  //状态方程
  inline Eigen::Vector4d diff(const Eigen::Vector4d& s,
                              const Eigen::Vector2d& input) const {
    Eigen::Vector4d ds;
    double phi = s(2);
    double v = s(3);
    double a = input(0);
    double delta = input(1);
    ds(0) = v * cos(phi);
    ds(1) = v * sin(phi);
    ds(2) = v / l * tan(delta);
    ds(3) = a;
    return ds;
  }

  //用龙格库塔来算数值解
  void step(const Eigen::Vector2d& input, const double dt) {
    // Runge–Kutta
    Eigen::Vector4d k1 = diff(state, input);
    Eigen::Vector4d k2 = diff(state + k1 * dt / 2, input);
    Eigen::Vector4d k3 = diff(state + k2 * dt / 2, input);//////ERROR？？
    Eigen::Vector4d k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
  }
};

namespace car_simulator {
class Nodelet : public nodelet::Nodelet {
 private:
  Car car;
  double delay_ = 0.0;
  Eigen::Vector2d input_;
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber path_sub_;
  ros::Timer sim_timer_;

  bool has_path_ = false;
  struct DelayedMsg {
    ros::Time t;
    double a, delta;
    DelayedMsg() {}
    DelayedMsg(const ros::Time& _t, double _a, double _delta) : t(_t), a(_a), delta(_delta) {}
  };
  std::deque<DelayedMsg> delayedMsgs_;

  void cmd_callback(const car_msgs::CarCmd::ConstPtr& msg) {
    delayedMsgs_.emplace_back(ros::Time::now(), msg->a, msg->delta);
    // input_(0) = msg->a;
    // input_(1) = msg->delta;
  }

  void path_callback(const nav_msgs::Path::ConstPtr& pathMsg)
  {
    has_path_ = true;
    //更新初始状态？？路径的起点一定是初始的状态吗？
    Eigen::Vector4d initS;
    initS(0) = pathMsg->poses[0].pose.position.x;
    initS(1) = pathMsg->poses[0].pose.position.y;
    double q0 = pathMsg->poses[0].pose.orientation.w;
    double q1 = pathMsg->poses[0].pose.orientation.x;
    double q2 = pathMsg->poses[0].pose.orientation.y;
    double q3 = pathMsg->poses[0].pose.orientation.z;
    double yaw_deg = 1 * std::atan2(2*q1*q2 + 2*q0*q3, -2 * q2 * q2 -2 * q3 * q3 + 1);
    // std::cout << "yaw is " << yaw_deg * 57.3 << std::endl;    
    initS(2) = yaw_deg;

    initS(3) = 0;//速度

    //原程序
    car.setInitialState(initS);
    ROS_INFO("Car Simulator Init Success!!!!!");
  }

  void timer_callback(const ros::TimerEvent& event) {
    //原程序
    if(has_path_ == false) return;
    if (!delayedMsgs_.empty()) {
      auto& msg = delayedMsgs_.front();
      //只有当当前时间减去msg.t（控制量产生的时间）大于延迟了，才说明该控制量可以起效果
      if ((ros::Time::now() - msg.t).toSec() > delay_) {
        input_(0) = msg.a;
        input_(1) = msg.delta;
        delayedMsgs_.pop_front();
      }
    }
    // std::cout << "-----|输入：----|" << input_(0) << ", " << input_(1) << std::endl;
    // std::cout << "-----|step前：----|" << car.state.transpose() << std::endl;
    car.step(input_, 1.0 / 400);
    // std::cout << "-----|step后：----|" << car.state.transpose() << std::endl;
    //将car的状态作为Odometry数据发送出来可视化
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";
    odom_msg.pose.pose.position.x = car.state(0);
    odom_msg.pose.pose.position.y = car.state(1);
    odom_msg.pose.pose.position.z = 0.0;
    double phi = car.state(2);
    double v = car.state(3);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(phi / 2);
    odom_msg.pose.pose.orientation.w = cos(phi / 2);

    odom_msg.twist.twist.linear.x = v * cos(phi);
    odom_msg.twist.twist.linear.y = v * sin(phi);
    odom_msg.twist.twist.linear.z = 0.0;

    odom_pub_.publish(odom_msg);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(car.state(0), car.state(1), 0));

    static tf::TransformBroadcaster transform_broadcaster;
    tf::Quaternion q;
    q.setX(odom_msg.pose.pose.orientation.x);
    q.setY(odom_msg.pose.pose.orientation.y);
    q.setZ(odom_msg.pose.pose.orientation.z);
    q.setW(odom_msg.pose.pose.orientation.w);
    transform.setRotation(q);

    transform_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                              ros::Time::now(), "world",
                                                              "ground_link")
    );



  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    nh.getParam("l", car.l);
    Eigen::Vector4d initS;
    nh.getParam("x", initS(0));
    nh.getParam("y", initS(1));
    nh.getParam("phi", initS(2));
    nh.getParam("v", initS(3));
    nh.getParam("delay", delay_);
    input_.setZero();
    car.setInitialState(initS);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_car", 1);
    cmd_sub_ = nh.subscribe<car_msgs::CarCmd>("car_cmd", 1, &Nodelet::cmd_callback, this, ros::TransportHints().tcpNoDelay());
    path_sub_ = nh.subscribe<nav_msgs::Path>("/hybrid_a_star_zm0612/searched_path_smoothed_with_d",1, &Nodelet::path_callback, this, ros::TransportHints().tcpNoDelay());
    // 以400hz的频率一直在更新state。
    // 每次来的cmd都放到queue里面，如果queue里面有cmd则拿出来作为新的input使用，都则就还是用上一次的cmd更新state
    sim_timer_ = nh.createTimer(ros::Duration(1.0 / 400), &Nodelet::timer_callback, this);
  }
};
}  // namespace car_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(car_simulator::Nodelet, nodelet::Nodelet);