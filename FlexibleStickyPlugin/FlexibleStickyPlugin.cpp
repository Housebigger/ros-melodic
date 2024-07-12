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

      
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->Name());

      
      this->sub = this->node->Subscribe("~/sticky_control", &FlexibleStickyPlugin::OnStickyControl, this);

      
      if (_sdf->HasElement("target"))
      {
        std::string targetName = _sdf->Get<std::string>("target");
        this->target = model->GetWorld()->ModelByName(targetName);
      }

      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FlexibleStickyPlugin::OnUpdate, this));
    }

    
    public: void OnUpdate()
    {
      if (!this->model || !this->target || !this->isSticky)
        return;

      this->model->SetWorldPose(this->target->WorldPose());
    }

   
    public: void OnStickyControl(ConstIntPtr &msg)
    {
      this->isSticky = (msg->data() != 0);
    }
  };

  
  GZ_REGISTER_MODEL_PLUGIN(FlexibleStickyPlugin)
}
