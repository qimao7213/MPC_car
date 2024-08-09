#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <mpc_car/mpc_car.hpp>
double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}

namespace mpc_car {
class Nodelet : public nodelet::Nodelet {
 private:
  std::shared_ptr<MpcCar> mpcPtr_;
  ros::Timer plan_timer_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pub_;
  ros::Subscriber path_sub_;
  VectorX state_;
  bool init_odom = false;
  bool init_path_seg = false;//原程序
  bool init_path_all = false;
  bool arrive_goal = false;
  double delay_ = 0.0;
  int path_seg_index = 0;//当前跟踪的seg的编号

  std::vector<std::vector<Eigen::Vector2d>> path_segs;
  std::vector<int> path_direction;

  void plan_timer_callback(const ros::TimerEvent& event) {
    if(init_path_all && !init_path_seg)//如果还没有初始化路径，就初始化
    {
      if(path_segs.empty()) return;
      if(path_segs[path_seg_index].size() > 2)
      {
        mpcPtr_->setPath(path_segs[path_seg_index], path_direction[path_seg_index]);
        for(int i = 0; i < path_segs[path_seg_index].size(); ++i)
        {
          std::cout << path_segs[path_seg_index][i].transpose() << std::endl;
        }
      }
      // 这里对于路径点size为2的，就用插值的方式来插成3个
      else if (path_segs[path_seg_index].size() == 2)
      {
        std::vector<Eigen::Vector2d> interpPoint(3);
        interpPoint[0] = path_segs[path_seg_index][0];
        interpPoint[2] = path_segs[path_seg_index][1];
        interpPoint[1] = 0.5 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
        mpcPtr_->setPath(interpPoint, path_direction[path_seg_index]);
        ROS_WARN("Get an interpolated traj!!");
      }
      init_path_seg = true;
    }
    if(init_odom && init_path_seg && !arrive_goal)
    {
      ros::Time t1 = ros::Time::now();
      std::cout << "x0: " << state_.transpose() << std::endl;
      int ret = mpcPtr_->solveQP(state_); //11表示到达终点了
      // assert(ret == 1 || ret == 11);
      if(ret == 11)
      {
        if(path_seg_index < path_segs.size() - 1)
        {
          path_seg_index += 1;
          if(path_segs[path_seg_index].size() > 2)
          {
            mpcPtr_->setPath(path_segs[path_seg_index], path_direction[path_seg_index]);
            for(int i = 0; i < path_segs[path_seg_index].size(); ++i)
            {
              std::cout << path_segs[path_seg_index][i].transpose() << std::endl;
            }
          }
          // 这里对于路径点size为2的，就用插值的方式来插成3个
          else if (path_segs[path_seg_index].size() == 2)
          {
            std::vector<Eigen::Vector2d> interpPoint(3);
            interpPoint[0] = path_segs[path_seg_index][0];
            interpPoint[2] = path_segs[path_seg_index][1];
            interpPoint[1] = 0.5 * (path_segs[path_seg_index][0] + path_segs[path_seg_index][1]);
            mpcPtr_->setPath(interpPoint, path_direction[path_seg_index]);
            ROS_WARN("Get an interpolated traj!!");
          }
          
            
          car_msgs::CarCmd msg;
          msg.header.frame_id = "world";
          msg.header.stamp = ros::Time::now();
            
          msg.a = 0;
          msg.delta = 0;
          cmd_pub_.publish(msg);
        }
        else
        {
          car_msgs::CarCmd msg;
          msg.header.frame_id = "world";
          msg.header.stamp = ros::Time::now();
            
          msg.a = 0;
          msg.delta = 0;
          cmd_pub_.publish(msg);
          arrive_goal = true;
          std::cout << "------已经到达终点了！----" << std::endl;
          return;
        }
      }
      ros::Time t2 = ros::Time::now();
      double solve_time = (t2 - t1).toSec();
      std::cout << "solve qp costs: " << 1e3 * solve_time << "ms" << std::endl;
      // TODO
      car_msgs::CarCmd msg;
      msg.header.frame_id = "world";
      msg.header.stamp = ros::Time::now();

      VectorX x;
      VectorU u;
      mpcPtr_->getPredictXU(0, x, u);
      std::cout << "u: " << u.transpose() << std::endl;
      std::cout << "x: " << x.transpose() << std::endl;
      std::cout << "---------------------" << std::endl;
      
      msg.a = u(0);
      msg.delta = u(1);
      cmd_pub_.publish(msg);
      mpcPtr_->visualization();
    }
    return;
  }
  void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    state_ << x, y, euler.z(), v.norm();
    double vel_angle = atan2(msg->twist.twist.linear.y, msg->twist.twist.linear.x);
    double angle_diff = vel_angle - euler.z();
    int forward_dir = path_direction[path_seg_index];

    angle_diff = Mod2Pi(angle_diff);
    // 如果此时的速度方向是倒车，但是路径是正向，则速度取负数
    if(forward_dir == 1 && abs(angle_diff) > 0.75 * M_PI)
    {
      state_(3) *= -1;
    }
    if(forward_dir == 0) state_(3) *= -1;//如果此时是倒车路径，那么需要取负数
    // 如果此时的速度方向是正车，但是路径是倒车，则速度取负数
    if(forward_dir == 0 && abs(angle_diff) < 0.25 * M_PI)
    {
      state_(3) *= -1;
    }

    
    init_odom = true;
    // std::cout << "car_mpc init success!" << std::endl;
  }
  void path_callback(const nav_msgs::Path::ConstPtr& pathMsg)
  {
    path_direction.resize(0);
    path_segs.resize(0);
    std::vector<Eigen::Vector2d> path_seg;
    int initDirection = pathMsg->poses[0].pose.position.z;
    path_direction.emplace_back(initDirection);
    for(int i = 0; i < pathMsg->poses.size(); ++i)
    {
      // std::cout << pathMsg->poses[i].pose.position.z << std::endl;
      if(pathMsg->poses[i].pose.position.z == path_direction.back())
      {
        path_seg.emplace_back(Eigen::Vector2d(pathMsg->poses[i].pose.position.x, pathMsg->poses[i].pose.position.y));
      }
      else
      {
        //新的方向
        path_direction.emplace_back(pathMsg->poses[i].pose.position.z);
        path_segs.emplace_back(path_seg);
        path_seg.resize(0);
        //把上一段的最后一个path point放进来
        // path_seg.emplace_back(path_segs.back().back());
        path_seg.emplace_back(Eigen::Vector2d(pathMsg->poses[i].pose.position.x, pathMsg->poses[i].pose.position.y));
      }
      if(i == pathMsg->poses.size() - 1)
      {
        path_segs.emplace_back(path_seg);
      }
    }
    //-----打印信息--------------
    std::cout << "一共生成 " << path_segs.size() << " 段轨迹, " << path_direction.size() << " 个方向" <<  std::endl;
    for(int i = 0; i < path_segs.size(); ++i)
    {
      std::cout << "第" << i << "段：" <<  std::endl;
      for(int j = 0; j < path_segs[i].size(); ++j)
      {
        std::cout << path_segs[i][j].transpose() << ", " << path_direction[i] << std::endl;
      }
    }

    // mpcPtr_->setPath(pathMsg);
    init_path_all = true;
    init_path_seg = false;
    arrive_goal = false;//每次有新的路径进来，说明就没有到达终点
    path_seg_index = 0;
    return;
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    mpcPtr_ = std::make_shared<MpcCar>(nh);
    double dt = 0;
    nh.getParam("dt", dt);
    nh.getParam("delay", delay_);

    plan_timer_ = nh.createTimer(ros::Duration(dt), &Nodelet::plan_timer_callback, this);
    // 这个"odom"是由car_simulator_nodelet发布的，就是里程计执行之后，然后更新状态？？
    // 然后这边又计算一个里程计发送到car_simulator_nodelet，让那边执行，再更新状态传回来
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/car_simulator/odom_car", 1, &Nodelet::odom_call_back, this);
    cmd_pub_ = nh.advertise<car_msgs::CarCmd>("car_cmd", 1);
    //原程序
    path_sub_ = nh.subscribe<nav_msgs::Path>("/hybrid_a_star_zm0612/searched_path_smoothed_with_d",1, 
                &Nodelet::path_callback, this, ros::TransportHints().tcpNoDelay());
  }
};
}  // namespace mpc_car

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mpc_car::Nodelet, nodelet::Nodelet);