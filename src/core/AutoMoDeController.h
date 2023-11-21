/*
 * @file <src/core/.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#ifndef AUTOMODE_BT_CONTROLLER_H
#define AUTOMODE_BT_CONTROLLER_H

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>

#include "./AutoMoDeBehaviorTree.h"
#include "./AutoMoDeBehaviorTreeBuilder.h"

#include <argos3/demiurge/rvr-dao/RVRDAO.h>
#include <argos3/demiurge/rvr-dao/ReferenceModel1Dot2.h>

#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_wheels_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_proximity_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_lidar_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_light_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_ground_color_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_colored_blob_omnidirectional_camera_sensor.h>

namespace argos{
	class AutoMoDeControllerBehaviorTree: public CCI_Controller {
		public:
			/*
			 * Class constructor.
			 */
			AutoMoDeControllerBehaviorTree();

			/*
			 * Class desctructor.
			 */
			virtual ~AutoMoDeControllerBehaviorTree();

			/*
			 * Controller initializer.
			 */
			virtual void Init(TConfigurationNode& t_node);

			/*
			 * Core of the controller.
			 */
			virtual void ControlStep();

			/*
			 *
			 */
			virtual void Reset();

			/*
			 *
			 */
			virtual void Destroy();

			/*
			 * Setter for the AutoMoDeBehaviorTree.
			 */
			void SetBehaviorTree(AutoMoDeBehaviorTree* pc_behavior_tree);

		private:
			/*
			 * Function that contains all actuations required at the start of an experiment or during the entire experiment.
			 * Example of what you might add in the future: display LED colors, start omnidirectional camera, etc.
			 * This function needs to be called by Reset() in order for the experiment to properly restart.
			 */
			void InitializeActuation();

			/*
			 * Pointer to the finite state machine object that represents the behaviour
			 * of the robot.
			 */
			AutoMoDeBehaviorTree* m_pcBehaviorTree;

			/*
			 * Pointer to the object representing the state of the robot. This object is
			 * shared with the finite state object AutoMoDeBehaviorTree.
			 */
			RVRDAO* m_pcRobotState;

			/*
			 * Time step variable.
			 */
			UInt32 m_unTimeStep;

			/*
			 * Integer part of the robot identifier.
			 */
			UInt32 m_unRobotID;

			/*
			 * String that contains the configuration of the behavior tree.
			 */
			std::string m_strBtConfiguration;

			/*
			 * Flag that tells whether an history is maintained or not.
			 */
			bool m_bMaintainHistory;

			/*
			 * Flag that tells whether an URL containing a DOT description of the
			 * finite state machine is to be displayed or not.
			 */
			bool m_bPrintReadableBt;
			
			/*
		       * Flag that tells whether we are using the real robot or not.
		       */
		      bool m_bRealRobot;

			/*
			 * The path to where the history shall be stored.
			 */
			std::string m_strHistoryFolder;

			/*
			 * Pointer to the object in charge of creating the AutoMoDeBehaviorTree.
			 */
			AutoMoDeBehaviorTreeBuilder* m_pcBehaviorTreeBuilder;

			/*
		       * Pointer to the robot wheels actuator.
		       */
		      CCI_RVRWheelsActuator *m_pcWheelsActuator;

		      /*
		       * Pointer to the robot proximity sensor.
		       */
		      CCI_RVRProximitySensor *m_pcProximitySensor;

		      /*
		       * Pointer to the robot lidar sensor.
		       */
		      CCI_RVRLidarSensor *m_pcLidarSensor;

		      /*
		       * Pointer to the robot light sensor.
		       */
		      CCI_RVRLightSensor *m_pcLightSensor;

		      /*
		       * Pointer to the robot ground sensor.
		       */
		      CCI_RVRGroundColorSensor *m_pcGroundSensor;

		      /*
		       * Pointer to the robot omnidirectional camera sensor.
		       */
		      CCI_RVRColoredBlobOmnidirectionalCameraSensor *m_pcOmnidirectionalCameraSensor;

			bool m_bBehaviorTreeGiven;
	};
}

#endif
