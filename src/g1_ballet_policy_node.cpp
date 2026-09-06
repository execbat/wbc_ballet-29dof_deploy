#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "g1_ballet_onnx_deploy/g1_abi.hpp"
#include "g1_ballet_onnx_deploy/motor_crc_hg.hpp"

using namespace std::chrono_literals;

namespace g1_ballet {

static std::vector<std::string> split_strings(const std::string &s) {
  std::vector<std::string> out; std::stringstream ss(s); std::string item;
  while (std::getline(ss, item, ',')) out.push_back(item);
  return out;
}
static std::vector<float> split_floats(const std::string &s) {
  std::vector<float> out; std::stringstream ss(s); std::string item;
  while (std::getline(ss, item, ',')) { if (!item.empty()) out.push_back(std::stof(item)); }
  return out;
}

template <size_t N>
static std::array<float, N> to_array(const std::vector<float> &v, const char *name) {
  if (v.size() != N) throw std::runtime_error(std::string(name) + " must contain " + std::to_string(N) + " values; got " + std::to_string(v.size()));
  std::array<float, N> a{}; std::copy(v.begin(), v.end(), a.begin()); return a;
}

class OnnxPolicy {
 public:
  explicit OnnxPolicy(const std::string &path)
      : env_(ORT_LOGGING_LEVEL_WARNING, "g1_ballet"), opts_(), session_(nullptr) {
    opts_.SetIntraOpNumThreads(1);
    opts_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, path.c_str(), opts_);
    Ort::AllocatorWithDefaultOptions alloc;
    input_name_holder_ = session_.GetInputNameAllocated(0, alloc);
    output_name_holder_ = session_.GetOutputNameAllocated(0, alloc);
    input_name_ = input_name_holder_.get(); output_name_ = output_name_holder_.get();

    const auto ishape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto oshape = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (ishape.empty() || ishape.back() != static_cast<int64_t>(kObsDim))
      throw std::runtime_error("ONNX input last dimension must be 186");
    if (oshape.empty() || oshape.back() != static_cast<int64_t>(kNumJoints))
      throw std::runtime_error("ONNX output last dimension must be 29");

    auto md = session_.GetModelMetadata();
    auto get_md = [&](const char *key) -> std::string {
      auto p = md.LookupCustomMetadataMapAllocated(key, alloc);
      if (!p) throw std::runtime_error(std::string("ONNX metadata missing required key: ") + key);
      return std::string(p.get());
    };
    const auto names = split_strings(get_md("joint_names"));
    if (names.size() != kNumJoints) throw std::runtime_error("joint_names metadata is not 29D");
    for (size_t i=0;i<kNumJoints;++i) {
      if (names[i] != kJointNames[i]) throw std::runtime_error("Joint order mismatch at index " + std::to_string(i) + ": ONNX=" + names[i] + " expected=" + std::string(kJointNames[i]));
    }
    const auto obs_names = split_strings(get_md("observation_names"));
    static const std::vector<std::string> expected_obs = {
      "imu_gyro","imu_lin_acc","projected_gravity","velocity_commands",
      "joint_pos","joint_vel","actions","axis_actual_normalized","axis_target_normalized","axis_mask"};
    if (obs_names != expected_obs) throw std::runtime_error("ONNX observation_names do not match ballet actor ABI");
    default_q_ = to_array<kNumJoints>(split_floats(get_md("default_joint_pos")), "default_joint_pos");
    action_scale_ = to_array<kNumJoints>(split_floats(get_md("action_scale")), "action_scale");
    kp_ = to_array<kNumJoints>(split_floats(get_md("joint_stiffness")), "joint_stiffness");
    kd_ = to_array<kNumJoints>(split_floats(get_md("joint_damping")), "joint_damping");
  }

  std::array<float,kNumJoints> infer(const std::array<float,kObsDim> &obs) {
    std::array<int64_t,2> shape{1, static_cast<int64_t>(kObsDim)};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto tensor = Ort::Value::CreateTensor<float>(mem, const_cast<float*>(obs.data()), obs.size(), shape.data(), shape.size());
    const char *ins[] = {input_name_}; const char *outs[] = {output_name_};
    auto y = session_.Run(Ort::RunOptions{nullptr}, ins, &tensor, 1, outs, 1);
    const float *p = y[0].GetTensorData<float>();
    std::array<float,kNumJoints> a{}; std::copy(p,p+kNumJoints,a.begin()); return a;
  }
  const auto& default_q() const { return default_q_; }
  const auto& action_scale() const { return action_scale_; }
  const auto& kp() const { return kp_; }
  const auto& kd() const { return kd_; }
 private:
  Ort::Env env_; Ort::SessionOptions opts_; Ort::Session session_;
  Ort::AllocatedStringPtr input_name_holder_{nullptr}; Ort::AllocatedStringPtr output_name_holder_{nullptr};
  const char *input_name_{nullptr}; const char *output_name_{nullptr};
  std::array<float,kNumJoints> default_q_{}, action_scale_{}, kp_{}, kd_{};
};

class BalletUdpReceiver {
 public:
  static constexpr size_t kPacketFloats = 61;
  static constexpr size_t kPacketBytes = kPacketFloats * sizeof(float);

  BalletUdpReceiver(const std::string &host, int port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("socket(AF_INET,SOCK_DGRAM) failed");
    int reuse = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      ::close(fd_); throw std::runtime_error("failed to make UDP socket non-blocking");
    }
    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(static_cast<uint16_t>(port));
    if (host == "0.0.0.0" || host.empty()) addr.sin_addr.s_addr = htonl(INADDR_ANY);
    else {
      if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        ::close(fd_); throw std::runtime_error("udp_host must be an IPv4 address or 0.0.0.0");
      }
    }
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      const std::string why = std::strerror(errno); ::close(fd_);
      throw std::runtime_error("UDP bind " + host + ":" + std::to_string(port) + " failed: " + why);
    }
  }
  ~BalletUdpReceiver(){ if(fd_>=0) ::close(fd_); }

  bool poll(std::array<float,29> &targets, std::array<float,29> &mask, std::array<float,3> &velocity) {
    bool got_valid = false;
    while (true) {
      std::array<unsigned char,kPacketBytes> bytes{};
      const ssize_t n = ::recvfrom(fd_, bytes.data(), bytes.size(), 0, nullptr, nullptr);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        break;  // preserve last good command; freshness watchdog will stop policy if stream dies
      }
      if (static_cast<size_t>(n) != kPacketBytes) continue;
      std::array<float,kPacketFloats> v{};
      std::memcpy(v.data(), bytes.data(), kPacketBytes); // G1/PC are little-endian; wire ABI is <f4
      bool finite = true; for(float x:v) finite = finite && std::isfinite(x); if(!finite) continue;
      for(size_t i=0;i<29;++i) targets[i] = std::clamp(v[i], -1.0f, 1.0f);
      for(size_t i=0;i<29;++i) mask[i] = v[29+i] >= 0.5f ? 1.0f : 0.0f;
      for(size_t i=0;i<3;++i) velocity[i] = v[58+i];
      got_valid = true;
    }
    return got_valid;
  }
 private:
  int fd_{-1};
};

class G1BalletNode : public rclcpp::Node {
 public:
  G1BalletNode() : Node("g1_ballet_onnx") {
    const auto policy_path = declare_parameter<std::string>("policy_path", "policy/policy.onnx");
    lowstate_topic_ = declare_parameter<std::string>("lowstate_topic", "lowstate");
    lowcmd_topic_ = declare_parameter<std::string>("lowcmd_topic", "lowcmd");
    udp_host_ = declare_parameter<std::string>("udp_host", "0.0.0.0");
    udp_port_ = declare_parameter<int>("udp_port", 55001);
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/ballet/enable");
    control_hz_ = declare_parameter<double>("control_hz", 50.0);
    ramp_seconds_ = declare_parameter<double>("ramp_seconds", 3.0);
    timeout_s_ = declare_parameter<double>("state_timeout_s", 0.15);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    max_tilt_rad_ = declare_parameter<double>("max_tilt_rad", 0.65);
    action_clip_ = declare_parameter<double>("action_clip", 1.0);
    target_limit_margin_ = declare_parameter<double>("target_limit_margin", 0.98);
    damping_kd_ = declare_parameter<double>("damping_kd", 2.0);

    policy_ = std::make_unique<OnnxPolicy>(policy_path);
    RCLCPP_INFO(get_logger(), "Loaded ONNX policy and verified ballet 186D/29DoF ABI: %s", policy_path.c_str());
    udp_ = std::make_unique<BalletUdpReceiver>(udp_host_, udp_port_);
    RCLCPP_INFO(get_logger(), "Listening for training-compatible ballet gamepad UDP on %s:%d (61 x float32, 244 bytes)", udp_host_.c_str(), udp_port_);

    lowcmd_pub_ = create_publisher<unitree_hg::msg::LowCmd>(lowcmd_topic_, 10);
    lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(lowstate_topic_, rclcpp::SensorDataQoS(), [this](unitree_hg::msg::LowState::SharedPtr m){
      std::lock_guard<std::mutex> l(mu_); state_ = *m; have_state_=true; state_stamp_=now();
    });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(enable_topic_, 10, [this](std_msgs::msg::Bool::SharedPtr m){
      std::lock_guard<std::mutex> l(mu_); requested_enable_=m->data;
      if(!m->data){active_=false; ramping_=false; RCLCPP_WARN(get_logger(), "Policy disabled; damping command will be used after first arm.");}
    });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/control_hz_), [this]{control();});
  }

 private:
  static std::array<float,3> projected_gravity(const std::array<double,4> &q) {
    const float w=q[0], x=-q[1], y=-q[2], z=-q[3];
    const float x2=x*x,y2=y*y,z2=z*z,w2=w*w,xy=x*y,xz=x*z,yz=y*z,wx=w*x,wy=w*y;
    return {-2.0f*(xz+wy), -2.0f*(yz-wx), -(w2-x2-y2+z2)};
  }
  bool fresh(const rclcpp::Time &t,double timeout) const {return (now()-t).seconds()<=timeout;}
  void publish_damping(const unitree_hg::msg::LowState &s) {
    unitree_hg::msg::LowCmd cmd; cmd.mode_pr=0; cmd.mode_machine=s.mode_machine;
    for(size_t i=0;i<35;++i){cmd.motor_cmd[i].mode=1;cmd.motor_cmd[i].q=0;cmd.motor_cmd[i].dq=0;cmd.motor_cmd[i].kp=0;cmd.motor_cmd[i].kd=0;cmd.motor_cmd[i].tau=0;}
    for(size_t i=0;i<kNumJoints;++i){cmd.motor_cmd[i].q=s.motor_state[i].q;cmd.motor_cmd[i].kd=static_cast<float>(damping_kd_);} set_crc(cmd); lowcmd_pub_->publish(cmd);
  }
  void publish_position(const unitree_hg::msg::LowState &s,const std::array<float,kNumJoints>& qdes,float gain_scale) {
    unitree_hg::msg::LowCmd cmd; cmd.mode_pr=0; cmd.mode_machine=s.mode_machine;
    for(size_t i=0;i<35;++i){cmd.motor_cmd[i].mode=1;cmd.motor_cmd[i].q=0;cmd.motor_cmd[i].dq=0;cmd.motor_cmd[i].kp=0;cmd.motor_cmd[i].kd=0;cmd.motor_cmd[i].tau=0;}
    for(size_t i=0;i<kNumJoints;++i){cmd.motor_cmd[i].q=qdes[i];cmd.motor_cmd[i].dq=0;cmd.motor_cmd[i].kp=policy_->kp()[i]*gain_scale;cmd.motor_cmd[i].kd=policy_->kd()[i]*gain_scale;cmd.motor_cmd[i].tau=0;}
    set_crc(cmd); lowcmd_pub_->publish(cmd);
  }
  void fail_safe(const unitree_hg::msg::LowState &s,const std::string &reason) {
    if(active_||ramping_) RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"FAIL-SAFE: %s",reason.c_str());
    active_=false;ramping_=false;requested_enable_=false;publish_damping(s);
  }
  void control() {
    // Poll every queued gamepad datagram and keep the newest valid command, exactly like
    // training UdpCommandReceiver's last-value semantics. Wire layout:
    // [target_normalized(29), mask(29), velocity(vx,vy,yaw)(3)].
    std::array<float,29> udp_targets{}, udp_mask{}; std::array<float,3> udp_velocity{};
    if (udp_->poll(udp_targets, udp_mask, udp_velocity)) {
      std::lock_guard<std::mutex> l(mu_);
      targets_=udp_targets; mask_=udp_mask; velocity_cmd_=udp_velocity;
      have_command_=true; command_stamp_=now();
    }
    unitree_hg::msg::LowState s; std::array<float,3> velcmd; std::array<float,29> targets,mask; bool req,hs,hc; rclcpp::Time st,ct;
    {std::lock_guard<std::mutex> l(mu_); s=state_;velcmd=velocity_cmd_;targets=targets_;mask=mask_;req=requested_enable_;hs=have_state_;hc=have_command_;st=state_stamp_;ct=command_stamp_;}
    if(!hs) return;
    if(!fresh(st,timeout_s_)){if(ever_armed_) publish_damping(s);return;}
    if(!req){if(ever_armed_)publish_damping(s);return;}
    if(!hc||!fresh(ct,command_timeout_s_)){fail_safe(s,"ballet gamepad UDP missing/stale");return;}
    const float roll=s.imu_state.rpy[0],pitch=s.imu_state.rpy[1];
    if(std::abs(roll)>max_tilt_rad_||std::abs(pitch)>max_tilt_rad_){fail_safe(s,"tilt limit exceeded");return;}

    if(!active_&&!ramping_){ramping_=true;ever_armed_=true;ramp_start_=now();for(size_t i=0;i<29;++i)ramp_q0_[i]=s.motor_state[i].q;RCLCPP_WARN(get_logger(),"ARMED: ramping current pose -> policy default pose for %.2f s",ramp_seconds_);}
    if(ramping_){
      float a=std::clamp(static_cast<float>((now()-ramp_start_).seconds()/ramp_seconds_),0.0f,1.0f); float smooth=a*a*(3-2*a); std::array<float,29> qd{};
      for(size_t i=0;i<29;++i)qd[i]=ramp_q0_[i]+smooth*(policy_->default_q()[i]-ramp_q0_[i]); publish_position(s,qd,0.5f);
      if(a>=1.0f){ramping_=false;active_=true;last_action_.fill(0);RCLCPP_WARN(get_logger(),"POLICY ACTIVE");} return;
    }

    std::array<float,kObsDim> obs{}; size_t k=0;
    auto put3=[&](const std::array<float,3>&v){for(float x:v)obs[k++]=x;};
    // Actor ABI starts with the raw pelvis IMU gyroscope from rt/lowstate.
    put3({s.imu_state.gyroscope[0],s.imu_state.gyroscope[1],s.imu_state.gyroscope[2]});
    put3({0.1f*std::clamp(s.imu_state.accelerometer[0],-30.0f,30.0f),0.1f*std::clamp(s.imu_state.accelerometer[1],-30.0f,30.0f),0.1f*std::clamp(s.imu_state.accelerometer[2],-30.0f,30.0f)});
    std::array<double,4> quat={s.imu_state.quaternion[0],s.imu_state.quaternion[1],s.imu_state.quaternion[2],s.imu_state.quaternion[3]}; put3(projected_gravity(quat));
    put3(velcmd);
    for(size_t i=0;i<29;++i)obs[k++]=s.motor_state[i].q-policy_->default_q()[i];
    for(size_t i=0;i<29;++i)obs[k++]=s.motor_state[i].dq;
    for(float x:last_action_)obs[k++]=x;
    for(size_t i=0;i<29;++i){float n=2.0f*(s.motor_state[i].q-kJointLower[i])/(kJointUpper[i]-kJointLower[i])-1.0f;obs[k++]=std::clamp(n,-1.0f,1.0f);}
    for(size_t i=0;i<29;++i)obs[k++]=mask[i]>=0.5f?targets[i]:0.0f;
    for(float x:mask)obs[k++]=x;
    if(k!=kObsDim){fail_safe(s,"internal observation dimension error");return;}
    for(float x:obs)if(!std::isfinite(x)){fail_safe(s,"NaN/Inf observation");return;}

    auto action=policy_->infer(obs); std::array<float,29> qdes{};
    for(size_t i=0;i<29;++i){if(!std::isfinite(action[i])){fail_safe(s,"NaN/Inf action");return;} action[i]=std::clamp(action[i],static_cast<float>(-action_clip_),static_cast<float>(action_clip_)); last_action_[i]=action[i]; float q=policy_->default_q()[i]+policy_->action_scale()[i]*action[i]; const float mid=0.5f*(kJointLower[i]+kJointUpper[i]),half=0.5f*(kJointUpper[i]-kJointLower[i])*target_limit_margin_;qdes[i]=std::clamp(q,mid-half,mid+half);}
    publish_position(s,qdes,1.0f);
  }

  std::unique_ptr<OnnxPolicy> policy_; std::unique_ptr<BalletUdpReceiver> udp_; std::mutex mu_; unitree_hg::msg::LowState state_{};
  std::array<float,3> velocity_cmd_{};std::array<float,29>targets_{},mask_{},last_action_{},ramp_q0_{};
  bool have_state_=false,have_command_=false,requested_enable_=false,active_=false,ramping_=false,ever_armed_=false;
  rclcpp::Time state_stamp_{0,0,RCL_ROS_TIME},command_stamp_{0,0,RCL_ROS_TIME},ramp_start_{0,0,RCL_ROS_TIME};
  std::string lowstate_topic_,lowcmd_topic_,enable_topic_,udp_host_; int udp_port_; double control_hz_,ramp_seconds_,timeout_s_,command_timeout_s_,max_tilt_rad_,action_clip_,target_limit_margin_,damping_kd_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_; rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_; rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_; rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace g1_ballet

int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<g1_ballet::G1BalletNode>());}catch(const std::exception&e){std::cerr<<"FATAL: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
