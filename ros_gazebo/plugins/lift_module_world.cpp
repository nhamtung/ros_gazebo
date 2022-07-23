#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>

#define NUM_BOTS 				1

namespace gazebo {
	class LiftModuleWorld: public WorldPlugin {
		private:
			physics::WorldPtr world;

		private:
			void addModel(std::string filename, std::string model_name, double x, double y, double z) {
				sdf::SDFPtr modelSDF;
				modelSDF.reset(new sdf::SDF);
				std::string file_name = common::ModelDatabase::Instance()->GetModelFile(filename);
				ROS_INFO("lift_module_world - file_name: %s", file_name.c_str());
				sdf::initFile("root.sdf", modelSDF);
				sdf::readFile(file_name, modelSDF);
				sdf::ElementPtr modelElem;

				if (modelSDF->Root()->HasElement("model")) {
					modelElem = modelSDF->Root()->GetElement("model");
				}

				std::string modelName = modelElem->GetAttribute("name")->GetAsString();

				modelName = model_name;
				modelElem->GetAttribute("name")->Set(modelName);
				modelElem->GetElement("pose")->Set(
				ignition::math::Pose3d(ignition::math::Vector3d(x, y, z), ignition::math::Quaterniond(0, 0, 0)));
				this->world->InsertModelSDF(*modelSDF);
			}

		public:
			void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {

				this->world = _parent;

				/* Add Bots */
				ROS_INFO("lift_module_world - Total bots: %d", NUM_BOTS);

				addModel("model://lift_module", "liftmodule" , 0.0, 0.0, 0.0);
			}

	};

	GZ_REGISTER_WORLD_PLUGIN(LiftModuleWorld)
}
