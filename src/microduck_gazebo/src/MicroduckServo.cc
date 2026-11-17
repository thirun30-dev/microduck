// MicroduckServo: XL330 position servos with rotor inertia (armature) for Gazebo Harmonic.
//
// The Microduck policies were trained in MuJoCo with joint armature 0.0018 kg m^2, which is
// 100x the link-side inertia of the head and ankle joints. Gazebo/SDF has no armature, and
// without it the policy falls within ~2 s of walking (reproduced in MuJoCo with armature=0).
//
// Each joint gets a virtual rotor (inertia = armature) driven by the servo law of the
// training model, coupled to the real joint through a stiff spring-damper:
//
//   tau_motor = clamp(kp * (target - theta), +-max_torque) - damping * theta_dot
//   tau_link  = ks * (theta - q) + ds * (theta_dot - q_dot)        (applied to the joint)
//   armature * theta_ddot = tau_motor - tau_link
//
// With ks=10, ds=0.002 this is explicit-stable at the 1 ms step and walks the same as
// MuJoCo's armature model. Targets arrive on <topic_prefix><joint> as gz.msgs.Double, the
// same topics JointPositionController used, so the ROS bridge is unchanged.
//
//   <plugin filename="MicroduckServo" name="microduck_gazebo::MicroduckServo">
//     <topic_prefix>/microduck/cmd/</topic_prefix>
//     <kp>0.55</kp> <max_torque>0.96</max_torque> <damping>0.053</damping>
//     <armature>0.0018</armature> <coupling_stiffness>10</coupling_stiffness>
//     <coupling_damping>0.002</coupling_damping>
//     <joint name="left_hip_yaw" initial="0.0"/> ...
//   </plugin>

#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>

namespace microduck_gazebo
{
struct Servo
{
  std::string name;
  gz::sim::Entity joint{gz::sim::kNullEntity};
  double target{0.0};
  double theta{0.0}, thetaDot{0.0};
  bool rotorInit{false};
};

class MicroduckServo : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(entity);
    auto get = [&](const char *key, double def) {
      return sdf->HasElement(key) ? sdf->Get<double>(key) : def;
    };
    this->kp = get("kp", 0.55);
    this->maxTorque = get("max_torque", 0.96);
    this->damping = get("damping", 0.053);
    this->armature = get("armature", 0.0018);
    this->ks = get("coupling_stiffness", 10.0);
    this->ds = get("coupling_damping", 0.002);
    std::string prefix = sdf->HasElement("topic_prefix")
                             ? sdf->Get<std::string>("topic_prefix") : "/microduck/cmd/";

    for (auto el = sdf->FindElement("joint"); el; el = el->GetNextElement("joint"))
    {
      Servo s;
      s.name = el->Get<std::string>("name");
      s.target = el->HasAttribute("initial") ? el->Get<double>("initial") : 0.0;
      s.joint = this->model.JointByName(ecm, s.name);
      if (s.joint == gz::sim::kNullEntity)
      {
        gzerr << "MicroduckServo: no joint [" << s.name << "]\n";
        continue;
      }
      this->servos.push_back(s);
    }
    for (size_t i = 0; i < this->servos.size(); ++i)
    {
      std::string topic = prefix + this->servos[i].name;
      std::function<void(const gz::msgs::Double &)> cb =
          [this, i](const gz::msgs::Double &m) {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->servos[i].target = m.data();
          };
      this->node.Subscribe(topic, cb);
    }
    gzmsg << "MicroduckServo: " << this->servos.size() << " joints, kp=" << this->kp
          << " armature=" << this->armature << " ks=" << this->ks << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    using namespace gz::sim;
    // World reset or time jump backwards: re-seat the rotors on the joints.
    if (info.simTime < this->lastSimTime)
      for (auto &s : this->servos) s.rotorInit = false;
    this->lastSimTime = info.simTime;

    for (auto &s : this->servos)
    {
      if (!ecm.Component<components::JointPosition>(s.joint))
        ecm.CreateComponent(s.joint, components::JointPosition());
      if (!ecm.Component<components::JointVelocity>(s.joint))
        ecm.CreateComponent(s.joint, components::JointVelocity());
    }
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0.0) return;

    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto &s : this->servos)
    {
      auto *qc = ecm.Component<components::JointPosition>(s.joint);
      auto *vc = ecm.Component<components::JointVelocity>(s.joint);
      if (!qc || qc->Data().empty() || !vc || vc->Data().empty()) continue;
      const double q = qc->Data()[0], qd = vc->Data()[0];
      if (!s.rotorInit)
      {
        s.theta = q;
        s.thetaDot = qd;
        s.rotorInit = true;
      }
      const double tauLink = this->ks * (s.theta - q) + this->ds * (s.thetaDot - qd);
      const double tauMotor =
          std::clamp(this->kp * (s.target - s.theta), -this->maxTorque, this->maxTorque) -
          this->damping * s.thetaDot;
      s.thetaDot += dt * (tauMotor - tauLink) / this->armature;  // semi-implicit Euler
      s.theta += dt * s.thetaDot;

      auto *fc = ecm.Component<components::JointForceCmd>(s.joint);
      if (fc)
        *fc = components::JointForceCmd({tauLink});
      else
        ecm.CreateComponent(s.joint, components::JointForceCmd({tauLink}));
    }
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::transport::Node node;
  std::mutex mutex;
  std::vector<Servo> servos;
  std::chrono::steady_clock::duration lastSimTime{0};
  double kp{0.55}, maxTorque{0.96}, damping{0.053}, armature{0.0018}, ks{10.0}, ds{0.002};
};
}  // namespace microduck_gazebo

GZ_ADD_PLUGIN(microduck_gazebo::MicroduckServo, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
