#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

namespace gazebo {
	class LiftModulePlugin: public ModelPlugin {
		private:
			ros::Subscriber joint_state;
			ros::NodeHandle nh_;
			physics::ModelPtr model;
			event::ConnectionPtr updateConnection;
			std::string model_name;
			physics::JointPtr lift;
			boost::thread* server_thread;
			float position = 0;
			bool status;

		public:
			void Init() {
				this->model->Reset();
			}

		public:
			void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
				// Store the pointer to the model
				this->model = _parent;
				this->model_name = model->GetName();
				ROS_INFO("lift_module_plugin - %s Loaded.", model_name.c_str());

				this->lift = this->model->GetJoint("lift");
				this->joint_state = this->nh_.subscribe( "/joint_states_lift_module", 200, &LiftModulePlugin::set_joint_states, this);

				// Listen to the update event. This event is broadcast every
				// simulation iteration.
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LiftModulePlugin::OnUpdate, this, _1));
    			// server_thread = new boost::thread(boost::bind(&LiftModulePlugin::serverThread, this, _1));
        		server_thread = new boost::thread(boost::bind(&LiftModulePlugin::serverThread, this));
			}
			// Called by the world update start event
		public:
			void OnUpdate(const common::UpdateInfo & /*_info*/) {
				ros::spinOnce();
			}
			void serverThread(){
				while(true){
					// ROS_INFO("lift_module_plugin - Lift by pos: %f", position);
					status = this->lift->SetPosition(0,  position);
                    ros::Duration(0.1).sleep();
				}
			}

		private:
			void set_joint_states(const sensor_msgs::JointState& joint_msg) {
				ROS_INFO("lift_module_plugin - set_joint_states Callback triggered: %s", joint_msg.name[0].c_str());
				ROS_INFO("lift_module_plugin - Pos: %f", joint_msg.position[0]);
				
				if(!strcmp("lift", joint_msg.name[0].c_str())){
					ROS_INFO("lift_module_plugin - Lift by pos: %f", joint_msg.position[0]);
					position = joint_msg.position[0];
					// status = this->lift->SetPosition(0,  joint_msg.position[0]);
					ROS_INFO("lift_module_plugin - Status: %d", status);
				}
			}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(LiftModulePlugin)
}
