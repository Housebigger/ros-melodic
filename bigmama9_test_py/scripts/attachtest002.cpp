#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  class FlexibleStickyPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: physics::ModelPtr target;
    private: event::ConnectionPtr updateConnection;
    private: bool isSticky = false;
    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

      // 初始化transport节点
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->Name());

      // 订阅服务以控制黏附
      this->sub = this->node->Subscribe("~/sticky_control", &FlexibleStickyPlugin::OnStickyControl, this);

      // 获取目标物体
      if (_sdf->HasElement("target"))
      {
        std::string targetName = _sdf->Get<std::string>("target");
        this->target = model->GetWorld()->ModelByName(targetName);
      }

      // 监听更新事件
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FlexibleStickyPlugin::OnUpdate, this));
    }

    // 更新事件的处理函数
    public: void OnUpdate()
    {
      if (!this->model || !this->target || !this->isSticky)
        return;

      this->model->SetWorldPose(this->target->WorldPose());
    }

    // 处理黏附控制消息
    public: void OnStickyControl(ConstIntPtr &msg)
    {
      this->isSticky = (msg->data() != 0);
    }
  };

  // 注册这个插件
  GZ_REGISTER_MODEL_PLUGIN(FlexibleStickyPlugin)
}
