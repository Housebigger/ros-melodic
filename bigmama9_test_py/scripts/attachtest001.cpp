#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class StickyRobotPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: physics::ModelPtr target; // 目标物体
    private: event::ConnectionPtr updateConnection;
    private: double threshold = 1.0; // 黏附阈值

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

      // 获取目标物体
      if (_sdf->HasElement("target"))
      {
        std::string targetName = _sdf->Get<std::string>("target");
        this->target = model->GetWorld()->ModelByName(targetName);
      }

      // 监听更新事件
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&StickyRobotPlugin::OnUpdate, this));
    }

    // 更新事件的处理函数
    public: void OnUpdate()
    {
      if (!this->model || !this->target)
        return;

      ignition::math::Vector3d robotPos = this->model->WorldPose().Pos();
      ignition::math::Vector3d targetPos = this->target->WorldPose().Pos();

      // 检查距离
      if (robotPos.Distance(targetPos) < this->threshold)
      {
        // 黏附逻辑
        this->model->SetWorldPose(this->target->WorldPose());
      }
    }
  };

  // 注册这个插件
  GZ_REGISTER_MODEL_PLUGIN(StickyRobotPlugin)
}
