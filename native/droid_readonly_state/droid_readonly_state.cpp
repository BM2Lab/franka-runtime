// D7 reader: no control(), move(), grasp(), stop(), setters or recovery calls.
#include <franka/robot.h>
#include <franka/gripper.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
namespace {
volatile std::sig_atomic_t quitting=0;
std::atomic<bool> stopping{false};
void quit(int){quitting=1;}
int64_t now(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))throw std::runtime_error("clock failed");return t.tv_sec*1000000000LL+t.tv_nsec;}
template<size_t N> void array(const std::array<double,N>& a){std::cout<<'[';for(size_t i=0;i<N;++i){if(i)std::cout<<',';std::cout<<a[i];}std::cout<<']';}
}
int main(int argc,char** argv){
  if(argc==2 && std::string(argv[1])=="--describe"){
    std::cout<<"{\"reader\":\"libfranka_readonly_v1\",\"libfranka\":\"0.21.2\",\"commands_enabled\":false}\n";return 0;
  }
  if(argc!=4){std::cerr<<"Usage: droid_readonly_state HOST EXPECTED_SERVER HZ\n";return 2;}
  std::signal(SIGTERM,quit);std::signal(SIGINT,quit);
  try{
    const int expected=std::stoi(argv[2]),hz=std::stoi(argv[3]);
    if(expected!=10 || hz<50 || hz>1000)throw std::runtime_error("Invalid server family or sample rate");
    franka::Robot robot(argv[1],franka::RealtimeConfig::kIgnore);
    if(robot.serverVersion()!=expected)throw std::runtime_error("Unexpected arm server version");
    franka::Gripper gripper(argv[1]);
    std::mutex mutex;franka::GripperState hand{};int64_t hand_ns=0;bool hand_failed=false;
    auto period=std::chrono::nanoseconds(1000000000LL/hz);
    std::thread hand_thread([&]{try{
      while(!quitting && !stopping.load()){auto s=gripper.readOnce();auto stamp=now();
        {std::lock_guard<std::mutex> l(mutex);hand=s;hand_ns=stamp;}
        std::this_thread::sleep_for(period);
      }
    }catch(const std::exception& e){std::cerr<<"Hand read: "<<std::string(e.what()).substr(0,512)<<'\n';
      std::lock_guard<std::mutex> l(mutex);hand_failed=true;
    }});
    int code=0;
    try{
      std::cout<<std::setprecision(17);int64_t next=0;
      while(!quitting && !stopping.load()){
        auto state=robot.readOnce();auto stamp=now();
        if(stamp<next)continue;next=stamp+period.count();
        franka::GripperState h{};int64_t hs;bool failed;
        {std::lock_guard<std::mutex> l(mutex);h=hand;hs=hand_ns;failed=hand_failed;}
        if(failed)throw std::runtime_error("Hand reader failed");
        if(!hs)continue;
        bool contact=std::any_of(state.joint_contact.begin(),state.joint_contact.end(),[](double x){return x!=0;}) ||
          std::any_of(state.cartesian_contact.begin(),state.cartesian_contact.end(),[](double x){return x!=0;}) ||
          std::any_of(state.joint_collision.begin(),state.joint_collision.end(),[](double x){return x!=0;}) ||
          std::any_of(state.cartesian_collision.begin(),state.cartesian_collision.end(),[](double x){return x!=0;});
        std::cout<<"{\"schema_version\":1,\"server_version\":"<<robot.serverVersion()
          <<",\"arm_ns\":"<<stamp<<",\"arm_counter_ms\":"<<state.time.toMSec()<<",\"q\":";array(state.q);
        std::cout<<",\"dq\":";array(state.dq);
        std::cout<<",\"robot_mode\":"<<static_cast<int>(state.robot_mode)
          <<",\"errors\":"<<(bool(state.current_errors)?"true":"false")<<",\"contact\":"<<(contact?"true":"false")
          <<",\"m_ee\":"<<state.m_ee<<",\"m_load\":"<<state.m_load<<",\"F_T_EE\":";array(state.F_T_EE);
        std::cout<<",\"hand_ns\":"<<hs<<",\"hand_counter_ms\":"<<h.time.toMSec()<<",\"width\":"<<h.width
          <<",\"max_width\":"<<h.max_width<<",\"grasped\":"<<(h.is_grasped?"true":"false")<<"}\n"<<std::flush;
        if(!std::cout)throw std::runtime_error("Output pipe closed");
      }
    }catch(const std::exception& e){std::cerr<<"Arm read: "<<std::string(e.what()).substr(0,512)<<'\n';code=1;}
    stopping.store(true);hand_thread.join();return code;
  }catch(const std::exception& e){std::cerr<<"Read-only reader: "<<std::string(e.what()).substr(0,512)<<'\n';return 1;}
}
