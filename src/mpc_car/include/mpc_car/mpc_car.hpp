#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <arc_spline/arc_spline.hpp>
#include <deque>
#include <iosqp/iosqp.hpp>

namespace mpc_car {

static constexpr int n = 4;  // state [x y phi v]，状态的维度
static constexpr int m = 2;  // input [a delta]，输入的维度
typedef Eigen::Matrix<double, n, n> MatrixA;//线性化时的A
typedef Eigen::Matrix<double, n, m> MatrixB;//线性化时的B
typedef Eigen::Vector4d VectorG;//线性化时的G
typedef Eigen::Vector4d VectorX;//状态向量
typedef Eigen::Vector2d VectorU;//输入向量

class MpcCar {
 private:
  ros::NodeHandle nh_;
  ros::Publisher ref_pub_, traj_pub_, traj_delay_pub_;

  double ll_;//车长
  double dt_;//预测周期的时长
  double rho_;//过程的权重？
  int N_;//预测的horizon
  double rhoN_;//终端状态的权重？

  double v_max_, a_max_, delta_max_, ddelta_max_;//bound约束
  double delay_;//延迟

  bool path_direction_ = 1;//1表示前进

  arc_spline::ArcSpline s_;//参考轨迹
  double desired_v_;//期待的速度？预估的一个速度？？

  osqp::IOSQP qpSolver_;

  std::vector<VectorX> predictState_;//预测出来的state
  std::vector<VectorU> predictInput_;//预测出来的input
  std::deque<VectorU> historyInput_;//先前的input，用于有delay的场景
  int history_length_;//由于delay造成的延迟的预测周期数
  VectorX x0_observe_;//预测时的状态，由car_simulator直接龙格库塔积分得到
  std::vector<VectorX> compensateDelayX0_;
  bool useCompensate1 = 0, useCompensate2 = 0;
  // 线性化时用到的几个矩阵
  // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
  MatrixA Ad_;//4*4
  MatrixB Bd_;//4*2
  VectorG gd_;//4*1

  /**整合为这样的形式，就可以直接放到求解器里面计算了？
   * osqp interface:
   * minimize     0.5 x^T P_ x + q_^T x
   * subject to   l_ <= A_ x <= u_
   **/
  // 上面公式里面的x是指要优化的量，是输入u吧？？
  // P_和q_是cost的权重矩阵
  // A_，l_，u_是整合之后的约束矩阵
  Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;

  /* *
   *               /  x1  \
   *               |  x2  |
   *  lx_ <=  Cx_  |  x3  |  <= ux_
   *               | ...  |
   *               \  xN  /
   * */
  //和state相关的约束
  Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // p, v constrains
  /* *
   *               /  u0  \
   *               |  u1  |
   *  lu_ <=  Cu_  |  u2  |  <= uu_
   *               | ...  |
   *               \ uN-1 /
   * */
  //和input相关的约束
  Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // [a delta ] constrains
  Eigen::SparseMatrix<double> Qx_;//这个是Q_bar

  //在某点处线性化
  void linearization(const double& phi,
                     const double& v,
                     const double& delta) {
    // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
    // TODO: set values to Ad_, Bd_, gd_
    // ...
    Ad_.setIdentity();  // Ad for instance
    Bd_.setZero();
    gd_.setZero();

    MatrixA Ac_;
    Ac_.setZero();
    Ac_.coeffRef(0, 2) = -1 * v * sin(phi);
    Ac_.coeffRef(0, 3) = cos(phi);
    Ac_.coeffRef(1, 2) = v * cos(phi);
    Ac_.coeffRef(1, 3) = sin(phi);
    Ac_.coeffRef(2, 3) = tan(delta)/ll_;
    Ad_ += Ac_ * dt_;

    Bd_.coeffRef(3, 0) = 1;
    Bd_.coeffRef(2, 1) = v/(ll_ * cos(delta) * cos(delta));
    Bd_ *= dt_;

    gd_(0) = v*phi*sin(phi);
    gd_(1) = -1*v*phi*cos(phi);
    gd_(2) = -1*v*delta/(ll_ * cos(delta) * cos(delta));
    gd_(3) = 0;
    gd_ *= dt_;

    return;
  }

  // 在轨迹s_上，求某一点(x, y)处的状态和输入(phi, v, delta)
  void calLinPoint(const double& s0, double& phi, double& v, double& delta) {
    // std::cout << "所期望的x, y:" << s_(s0, 0) << std::endl;
    Eigen::Vector2d dxy = s_(s0, 1);
    Eigen::Vector2d ddxy = s_(s0, 2);
    double dx = dxy.x();
    double dy = dxy.y();
    double ddx = ddxy.x();
    double ddy = ddxy.y();
    double dphi = (ddy * dx - dy * ddx) / (dx * dx + dy * dy);//简化的曲率公式
    if(path_direction_ == 1) phi = atan2(dy, dx);
      else phi = atan2(dy, dx) - M_PI;//倒车
    
    if(s0 >= s_.arcL())
    {
      v = 0.0;
      delta = 0.0;
    }
    else
    {
      v = desired_v_;//这个速度为什么不使用机器人odomtery发布的速度呢？难道是那个速度也是和desired_v一致的吗
      // std::cout << v << std::endl;
      // v = std::max (x0_observe_(3), desired_v_);
      delta = atan2(ll_ * dphi / (v) , 1.0);// ERROR?这里不是应该再除以一个v吗？
      // if(path_direction_ == 0) delta *= -1; //倒车
      // delta = atan2(ll_ * dphi , 1.0);// ERROR?这里不是应该再除以一个v吗？
    }


  }

  inline VectorX diff(const VectorX& state,
                      const VectorU& input) const {
    VectorX ds;
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    ds(0) = v * cos(phi);
    ds(1) = v * sin(phi);
    ds(2) = v / ll_ * tan(delta);
    ds(3) = a;
    return ds;
  }

  inline void step(VectorX& state, const VectorU& input, const double dt) const {
    // Runge–Kutta
    VectorX k1 = diff(state, input);
    VectorX k2 = diff(state + k1 * dt / 2, input);
    VectorX k3 = diff(state + k2 * dt / 2, input);
    VectorX k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
  }
  VectorX compensateDelay0(const VectorX& x0) {
    VectorX x0_delay = x0;
    return x0_delay;
  }

  VectorX compensateDelay1(const VectorX& x0) {
    useCompensate1 = 1;
    VectorX x0_delay = x0;
    double dt = 0.001;
    for (double t = delay_; t > 0; t -= dt) {
      int i = std::ceil(t / dt_);
      VectorU input = historyInput_[history_length_ - i];
      step(x0_delay, input, dt);
    }
    return x0_delay;
  }

  VectorX compensateDelay2(const VectorX &x0){
    useCompensate2 = 1;
    compensateDelayX0_.clear();
    compensateDelayX0_.push_back(x0);
    VectorX x0_delay = x0;
    double s0 = s_.findS(x0_delay.head(2));
    double phi, v, delta;
    double last_phi = x0(2);
    for (int i = 0; i < history_length_; ++i) {
      calLinPoint(s0, phi, v, delta);//
      if (phi - last_phi > M_PI) {
        phi -= 2 * M_PI;
      } else if (phi - last_phi < -M_PI) {
        phi += 2 * M_PI;
      }
      last_phi = phi;
      linearization(phi, v, delta);
      x0_delay = Ad_ * x0_delay + Bd_ * historyInput_[i] + gd_;//这里要构造A
      compensateDelayX0_.push_back(x0_delay);
      s0 += abs(x0_delay[3]) * dt_;
      s0 = s0 < s_.arcL() ? s0 : s_.arcL();
    }
    return x0_delay;
  }

 public:
  MpcCar(ros::NodeHandle& nh) : nh_(nh) {
    // load map
    std::vector<double> track_points_x, track_points_y;
    nh.getParam("track_points_x", track_points_x);
    nh.getParam("track_points_y", track_points_y);
    nh.getParam("desired_v", desired_v_);
    s_.setWayPoints(track_points_x, track_points_y);
    // load parameters
    nh.getParam("ll", ll_);
    nh.getParam("dt", dt_);
    nh.getParam("rho", rho_);
    nh.getParam("N", N_);
    nh.getParam("rhoN", rhoN_);
    nh.getParam("v_max", v_max_);
    nh.getParam("a_max", a_max_);
    nh.getParam("delta_max", delta_max_);
    nh.getParam("ddelta_max", ddelta_max_);
    nh.getParam("delay", delay_);
    history_length_ = std::ceil(delay_ / dt_);//向上取整

    ref_pub_ = nh.advertise<nav_msgs::Path>("path_ref", 1);
    traj_pub_ = nh.advertise<nav_msgs::Path>("traj_mpc", 1);
    traj_delay_pub_ = nh.advertise<nav_msgs::Path>("traj_mpc_delay", 1);

    // TODO: set initial value of Ad, Bd, gd
    Ad_.setIdentity();  // Ad for instance
    Bd_.setZero();
    gd_.setZero();
    // ...
    // set size of sparse matrices
    P_.resize(m * N_, m * N_);
    q_.resize(m * N_, 1);
    Qx_.resize(n * N_, n * N_);
    // stage cost
    // every        1 0 0   0  
    //          Q = 0 1 0   0
    //              0 0 rho 0
    //              0 0 0   0
    //那就是看你cost function怎么设计了，前面的两个1是x和y的位置误差，rho是对phi，对方向的跟踪？对v没有跟踪
    Qx_.setIdentity();
    for (int i = 1; i < N_; ++i) {
      Qx_.coeffRef(i * n - 2, i * n - 2) = rho_;
      Qx_.coeffRef(i * n - 1, i * n - 1) = 0;
    }
    //这下面是设置Q_bar里面的F矩阵
    Qx_.coeffRef(N_ * n - 4, N_ * n - 4) = rhoN_;
    Qx_.coeffRef(N_ * n - 3, N_ * n - 3) = rhoN_;
    Qx_.coeffRef(N_ * n - 2, N_ * n - 2) = rhoN_ * rho_;//
    int n_cons = 4;  // [v a delta ddelta]，约束的维度。对每一个预测周期都需要约束
    A_.resize(n_cons * N_, m * N_);
    l_.resize(n_cons * N_, 1);
    u_.resize(n_cons * N_, 1);
    // [v] constrains
    Cx_.resize(1 * N_, n * N_);
    lx_.resize(1 * N_, 1);
    ux_.resize(1 * N_, 1);
    // [a delta ddelta] constrains
    Cu_.resize(3 * N_, m * N_);
    lu_.resize(3 * N_, 1);
    uu_.resize(3 * N_, 1);
    // set lower and upper boundaries
    for (int i = 0; i < N_; ++i) {
      // TODO: set stage constraints of inputs (a, delta, ddelta)
      // -a_max <= a <= a_max for instance:
      Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
      lu_.coeffRef(i * 3 + 0, 0) = -a_max_;
      uu_.coeffRef(i * 3 + 0, 0) = a_max_;
      // ...
      Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
      lu_.coeffRef(i * 3 + 1, 0) = -delta_max_;
      uu_.coeffRef(i * 3 + 1, 0) = delta_max_;

      // TODO: 这里可能还有问题
      if(i > 0)
      {
        Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1/dt_;
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1/dt_;
        lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_;
        uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_;        
      }
      else
      {
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
      }
      // if(i > 0) Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1;
      // Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
      // lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_ * dt_;
      // uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_ * dt_;  


      // TODO: set stage constraints of states (v)
      // -v_max <= v <= v_max
      Cx_.coeffRef(i, i * n + 3) = 1;
      lx_.coeffRef(i, 0) = -v_max_; 
      ux_.coeffRef(i, 0) = v_max_;
    }
    // set predict mats size
    predictState_.resize(N_);
    predictInput_.resize(N_);
    for (int i = 0; i < N_; ++i) {
      predictInput_[i].setZero();
    }
    for (int i = 0; i < history_length_; ++i) {
      historyInput_.emplace_back(0, 0);
    }
  }

  bool check_goal(const VectorX& x0)
  {
    
    Eigen::Vector2d dxy = s_(s_.arcL(), 1);
    double phi = atan2(dxy.y(), dxy.x());
    if(path_direction_ == 0) phi -= M_PI;
    if (phi - x0(2) > M_PI) {
      phi -= 2 * M_PI;
    } else if (phi - x0(2) < -M_PI) {
      phi += 2 * M_PI;
    } 
    
    Eigen::Vector2d gxy = s_(s_.arcL(), 0);
    double dx = gxy.x() - x0.x();
    double dy = gxy.y() - x0.y();
    std::cout << "dxy: " <<  (dx * dx + dy * dy) << ", v: " << std::abs(x0(3));
    std::cout << ", std::abs(phi - x0(2)): " << std::abs(phi - x0(2)) << std::endl;
    if((dx * dx + dy * dy) < 0.1 && std::abs(x0(3)) < 0.01 && std::abs(phi - x0(2)) < 0.05) return true;
    // else if((dx * dx + dy * dy) < 0.01 && std::abs(x0(3)) < 0.001) return true;
    return false;
  }

  int solveQP(const VectorX& x0_observe) {
    x0_observe_ = x0_observe;
    historyInput_.pop_front();
    historyInput_.push_back(predictInput_.front());
    //这是XX的约束，由预测出来的delta，结合ddelta_max来确定delta的上下界？
    lu_.coeffRef(2, 0) = predictInput_.front()(1) - ddelta_max_ * dt_;
    uu_.coeffRef(2, 0) = predictInput_.front()(1) + ddelta_max_ * dt_;
    VectorX x0 = compensateDelay2(x0_observe_);//当前应该的状态。如果是由延迟的情况下，那么x0_observe_是存在延迟的状态
    bool arrive_goal = check_goal(x0);
    // std::cout << "arrive the goal ?? : " << arrive_goal << std::endl;
    if(arrive_goal)
    {
      // for (int i = 0; i < N_; ++i) 
      // {
      //   predictInput_[i] = VectorU::Zero();
      //   predictState_[i] = predictMat.col(i);
      // }
      std::cout << "------------!!!!到达seg的终点!!!!-----------" << std::endl;
      std::cout << "到达终点时的状态为：" << x0.transpose() << std::endl;
      return 11;//表示已经到达终点了
    }
    // set BB, AA, gg
    Eigen::MatrixXd BB, AA, gg;
    BB.setZero(n * N_, m * N_);
    AA.setZero(n * N_, n);
    gg.setZero(n * N_, 1);
    double s0 = s_.findS(x0.head(2));//这个是在整体路径上的进度，是一个长度值，不是百分比
    double phi, v, delta;
    double last_phi = x0(2);
    Eigen::SparseMatrix<double> qx;
    qx.resize(n * N_, 1);
    for (int i = 0; i < N_; ++i) {
      calLinPoint(s0, phi, v, delta);//
      if (phi - last_phi > M_PI) {
        phi -= 2 * M_PI;
      } else if (phi - last_phi < -M_PI) {
        phi += 2 * M_PI;
      }
      last_phi = phi;
      linearization(phi, v, delta);//这些量都是当前状态所处轨迹上的点
      // calculate big state-space matrices
      /* *                BB                AA
       * x1    /       B    0  ... 0 \    /   A \
       * x2    |      AB    B  ... 0 |    |  A2 |
       * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
       * ...   |     ...  ...  ... 0 |    | ... |
       * xN    \A^(n-1)B  ...  ... B /    \ A^N /
       *
       *     X = BB * U + AA * x0 + gg
       * */
      if (i == 0) {
        BB.block(0, 0, n, m) = Bd_;
        AA.block(0, 0, n, n) = Ad_;
        gg.block(0, 0, n, 1) = gd_;
      } else {
        // TODO: set BB AA gg
        // ...
        BB.block(n*i, m*i, n, m) = Bd_;
        for(int j = i - 1; j >= 0; --j)
        {
          BB.block(n * i, m * j, n, m) = Ad_ * BB.block(n*(i-1), m*j, n, m);
        }
        AA.block(n * i, 0, n, n) = Ad_ * AA.block(n*(i-1), 0, n, n);
        gg.block(n * i, 0, n, 1) = Ad_ * gg.block(n*(i-1), 0, n, 1) + gd_;
      }

      //这个地方不能用矩阵幂的方式来求，因为Ad_之类的在每一次迭代的时候都是变化的
      // for(int j = 0; j <= i; ++j)
      // {
      //   BB.block(n * i, m * j, n, m ) = matrixAPower(Ad_, i-j) * Bd_;
      // }
      // AA.block(n * i, 0, n, n) = matrixAPower(Ad_, i+1);
      // for(int j = 0; j <= i; ++j)
      // {
      //   gg.block(n * i, 0, n, 1) += matrixAPower(Ad_, j) * gd_;
      // }

      // TODO: set qx
      Eigen::Vector2d xy = s_(s0);  // reference (x_r, y_r)。这里不应该是下一个期待到达的点吗？为什么是当前的点呢？

      // cost function should be represented as follows:
      /* *
       *           /  x1  \T       /  x1  \         /  x1  \
       *           |  x2  |        |  x2  |         |  x2  |
       *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
       *           | ...  |        | ...  |         | ...  |
       *           \  xN  /        \  xN  /         \  xN  /
       * */

      // qx.coeffRef(...
      // ...
      // qx = -Qx_.toDense().block(n * i, n * i, 4, 4) transpose() * VectorX(xy(0), xy(1), phi, 0);
      qx.coeffRef(n*i + 0, 0) = -Qx_.coeffRef(n * i + 0, n * i + 0) * xy(0);
      qx.coeffRef(n*i + 1, 0) = -Qx_.coeffRef(n * i + 1, n * i + 1) * xy(1);
      qx.coeffRef(n*i + 2, 0) = -Qx_.coeffRef(n * i + 2, n * i + 2) * phi;
      qx.coeffRef(n*i + 3, 0) = -Qx_.coeffRef(n * i + 3, n * i + 3) * v;
      // qx.coeffRef(n*i + 3, 0) = -0;//本来就是让期望的末速度为0了？？
      s0 += std::abs(desired_v_) * dt_;
      s0 = s0 < s_.arcL() ? s0 : s_.arcL();
      if(i == 0)
        std::cout << "desired: " << xy.transpose() << ", " << phi << ", " << v << std::endl;
    }
    Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
    Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
    Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
    Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

    // state constrants propogate to input constraints using "X = BB * U + AA * x0 + gg"
    /* *
     *               /  x1  \                              /  u0  \
     *               |  x2  |                              |  u1  |
     *  lx_ <=  Cx_  |  x3  |  <= ux_    ==>    lx <=  Cx  |  u2  |  <= ux
     *               | ...  |                              | ...  |
     *               \  xN  /                              \ uN-1 /
     * */
    Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse; // N_*mN_
    Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;//N_*1
    Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;//N_*1

    /* *      / Cx  \       / lx  \       / ux  \
     *   A_ = \ Cu_ /, l_ = \ lu_ /, u_ = \ uu_ /
     * */
    
    //这里为什么要先转置一遍呀
    //A_的维度为4N_*mN_，所以优化变量的维度是mN_*1，还是控制量，
    //不是PPT24页里面把状态量和控制量放在一起优化，而是把对状态量的约束转换为了对控制量的约束
    Eigen::SparseMatrix<double> A_T = A_.transpose();
    A_T.middleCols(0, Cx.rows()) = Cx.transpose();
    A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
    A_ = A_T.transpose();
    //这里就是直接拼接起来，那么优化变量就变成了？？
    for (int i = 0; i < lx.rows(); ++i) {
      l_.coeffRef(i, 0) = lx.coeff(i, 0);
      u_.coeffRef(i, 0) = ux.coeff(i, 0);
    }
    for (int i = 0; i < lu_.rows(); ++i) {
      l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
      u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
    }
    Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
    P_ = BBT_sparse * Qx_ * BB_sparse;
    q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;
    // osqp
    Eigen::VectorXd q_d = q_.toDense();
    Eigen::VectorXd l_d = l_.toDense();
    Eigen::VectorXd u_d = u_.toDense();
    qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
    qpSolver_.solve();
    int ret = qpSolver_.getStatus();
    if (ret != 1) {
      ROS_ERROR("fail to solve QP!");
      return ret;
    }
    //将很长的向量映射为矩阵
    //qpSolver_求解的只是input u，而状态x是计算出来的
    Eigen::VectorXd sol = qpSolver_.getPrimalSol();
    Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), m, N_);
    Eigen::VectorXd solState = BB * sol + AA * x0 + gg;
    Eigen::MatrixXd predictMat = Eigen::Map<const Eigen::MatrixXd>(solState.data(), n, N_);
    //然后将矩阵放到vector中去
    for (int i = 0; i < N_; ++i) {
      predictInput_[i] = solMat.col(i);
      predictState_[i] = predictMat.col(i);
    }
    return ret;
  }

  void getPredictXU(double t, VectorX& state, VectorU& input) {
    if (t <= dt_) {
      state = predictState_.front();
      input = predictInput_.front();
      return;
    }

    //输入的t一直都是0，那么下面这一段是用来干什么的呢?
    int horizon = std::floor(t / dt_);
    double dt = t - horizon * dt_;
    state = predictState_[horizon - 1];
    input = predictInput_[horizon - 1];
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    state(0) += dt * v * cos(phi);
    state(1) += dt * v * sin(phi);
    state(2) += dt * v / ll_ * tan(delta);
    state(3) += dt * a;
  }

  // visualization
  // 在可视化参考轨迹，预测的轨迹，带延迟的预测轨迹。第一个是不变的，后面两个和运行的时刻有关
  void visualization() {
    nav_msgs::Path msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p;
    for (double s = 0; s < s_.arcL(); s += 0.01) {
      p.pose.position.x = s_(s).x();
      p.pose.position.y = s_(s).y();
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    ref_pub_.publish(msg);
    msg.poses.clear();
    for (int i = 0; i < N_; ++i) {
      p.pose.position.x = predictState_[i](0);
      p.pose.position.y = predictState_[i](1);
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    traj_pub_.publish(msg);

    if(useCompensate1 == 1)
    {
      msg.poses.clear();
      VectorX x0_delay = x0_observe_;
      double dt = 0.001;
      for (double t = delay_; t > 0; t -= dt) {
        int i = std::ceil(t / dt_);
        VectorU input = historyInput_[history_length_ - i];
        step(x0_delay, input, dt);
        p.pose.position.x = x0_delay(0);
        p.pose.position.y = x0_delay(1);
        p.pose.position.z = 0.0;
        msg.poses.push_back(p);
      }
      traj_delay_pub_.publish(msg);
    }
    else if(useCompensate2 == 1)
    {
      msg.poses.clear();
      for(int i = 0; i < compensateDelayX0_.size(); i++)
      {
        p.pose.position.x = compensateDelayX0_[i](0);
        p.pose.position.y = compensateDelayX0_[i](1);
        p.pose.position.z = 0.0;
        msg.poses.push_back(p);
      }
      //这个只是delay的那一段吧
      traj_delay_pub_.publish(msg);
    }

  }

  void setPath(const nav_msgs::Path::ConstPtr& pathMsg)
  {
    std::vector<double> track_points_x, track_points_y;
    for(int i = 0; i < pathMsg->poses.size(); ++i)
    {
      track_points_x.push_back(pathMsg->poses[i].pose.position.x);
      track_points_y.push_back(pathMsg->poses[i].pose.position.y);
    }
    s_.setWayPoints(track_points_x, track_points_y);
    std::cout << "Set new path!!!" << std::endl;
  }

  void setPath(const std::vector<Eigen::Vector2d> &path_seg, int path_direction)
  {
    std::vector<double> track_points_x, track_points_y;
    for(int i = 0; i < path_seg.size(); ++i)
    {
      track_points_x.push_back(path_seg[i](0));
      track_points_y.push_back(path_seg[i](1));
    }
    s_.setWayPoints(track_points_x, track_points_y);

    if(path_direction == 1)//向前
    {
      path_direction_ = 1;
      desired_v_ = abs(desired_v_);
    }
    else//向后
    {
      path_direction_ = 0;
      desired_v_ = -1 * abs(desired_v_);
    }
    std::cout << "Set new path seg!!! The desired_v is " << desired_v_ << std::endl;
  }
};

}  // namespace mpc_car