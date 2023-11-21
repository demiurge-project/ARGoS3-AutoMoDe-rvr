/*
 * @file <src/core/AutoMoDeControllerBehaviorTree.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeController.h"

namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeControllerBehaviorTree::AutoMoDeControllerBehaviorTree() {
		m_pcRobotState = new ReferenceModel1Dot2();
		m_unTimeStep = 0;
		m_strBtConfiguration = "";
		m_bMaintainHistory = false;
		m_bPrintReadableBt = false;
		m_strHistoryFolder = "./";
		m_bBehaviorTreeGiven = false;
		m_bRealRobot = false;
	}

	/****************************************/
	/****************************************/

	AutoMoDeControllerBehaviorTree::~AutoMoDeControllerBehaviorTree() {
		delete m_pcRobotState;
		delete m_pcBehaviorTreeBuilder;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerBehaviorTree::Init(TConfigurationNode& t_node) {
		// Parsing parameters
		Real ptVelocity = m_pcRobotState->GetMaxVelocity();
		try {
			GetNodeAttributeOrDefault(t_node, "bt-config", m_strBtConfiguration, m_strBtConfiguration);
			GetNodeAttributeOrDefault(t_node, "history", m_bMaintainHistory, m_bMaintainHistory);
			GetNodeAttributeOrDefault(t_node, "hist-folder", m_strHistoryFolder, m_strHistoryFolder);
			GetNodeAttributeOrDefault(t_node, "readable", m_bPrintReadableBt, m_bPrintReadableBt);
			GetNodeAttributeOrDefault(t_node, "real-robot", m_bRealRobot, m_bRealRobot);
			GetNodeAttributeOrDefault(t_node, "velocity", ptVelocity, ptVelocity);
		} catch (CARGoSException& ex) {
			THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
		}

		m_unRobotID = atoi(GetId().substr(3, 6).c_str());
		m_pcRobotState->SetMaxVelocity(ptVelocity);
		m_pcRobotState->SetRobotIdentifier(m_unRobotID);

		/*
		 * If a BT configuration is given as parameter of the experiment file, create a FSM from it
		 */
		if (m_strBtConfiguration.compare("") != 0 && !m_bBehaviorTreeGiven) {
			m_pcBehaviorTreeBuilder = new AutoMoDeBehaviorTreeBuilder();
			SetBehaviorTree(m_pcBehaviorTreeBuilder->BuildBehaviorTree(m_strBtConfiguration));
			if (m_bMaintainHistory) {
				//m_pcBehaviorTree->SetHistoryFolder(m_strHistoryFolder);
				//m_pcBehaviorTree->MaintainHistory();
			}
			if (m_bPrintReadableBt) {
				std::cout << "Behavior Tree description: " << std::endl;
				std::cout << m_pcBehaviorTree->GetReadableFormat() << std::endl;
			}
		} else {
			LOGERR << "Warning: No behavior tree configuration found in .argos" << std::endl;
		}

		/*
		 *  Initializing sensors and actuators
		 */
		try{
			m_pcProximitySensor = GetSensor<CCI_RVRProximitySensor>("rvr_proximity");
			m_pcLightSensor = GetSensor<CCI_RVRLightSensor>("rvr_light");
			m_pcGroundSensor = GetSensor<CCI_RVRGroundColorSensor>("rvr_ground");
			m_pcLidarSensor = GetSensor<CCI_RVRLidarSensor>("rvr_lidar");
			m_pcOmnidirectionalCameraSensor = GetSensor<CCI_RVRColoredBlobOmnidirectionalCameraSensor>("rvr_colored_blob_omnidirectional_camera");
			m_pcOmnidirectionalCameraSensor->Enable();
		} catch (CARGoSException ex) {
			LOGERR<<"Error while initializing a Sensor!\n";
		}

		try{
			m_pcWheelsActuator = GetActuator<CCI_RVRWheelsActuator>("rvr_wheels");
		} catch (CARGoSException ex) {
			LOGERR<<"Error while initializing an Actuator!\n";
		}

		/*
		 * Starts actuation.
		 */
		 InitializeActuation();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerBehaviorTree::ControlStep() {
		/*
		 * 1. Update RobotDAO
		 */
		if (m_pcGroundSensor != NULL) {
			const CCI_RVRGroundColorSensor::SReading &reading = m_pcGroundSensor->GetReading();
			m_pcRobotState->SetGroundInput(reading);
		}
		if (m_pcLightSensor != NULL) {
			const CCI_RVRLightSensor::SReading &reading = m_pcLightSensor->GetReading();
			m_pcRobotState->SetLightInput(reading);
		}
		if (m_pcProximitySensor != NULL) {
			const CCI_RVRProximitySensor::TReadings &readings = m_pcProximitySensor->GetReadings();
			m_pcRobotState->SetProximityInput(readings);
		}
		if (m_pcLidarSensor != NULL) {
			const CCI_RVRLidarSensor::TReadings &readings = m_pcLidarSensor->GetReadings();
			m_pcRobotState->SetLidarInput(readings);
		}
		if (m_pcOmnidirectionalCameraSensor != NULL) {
			const CCI_RVRColoredBlobOmnidirectionalCameraSensor::SReadings &readings = m_pcOmnidirectionalCameraSensor->GetReadings();
			m_pcRobotState->SetOmnidirectionalCameraInput(readings);
		}

		/*
		 * 2. Execute step of running Action of BT
		 */
		m_pcBehaviorTree->ControlStep();

		/*
		 * 3. Update Actuators
		 */
		if (m_pcWheelsActuator != NULL) {
			m_pcWheelsActuator->SetLinearVelocity(m_pcRobotState->GetLeftWheelVelocity(),m_pcRobotState->GetRightWheelVelocity());
		}

		/*
		 * 4. Update variables and sensors
		 */
		m_unTimeStep++;

	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerBehaviorTree::Destroy() {}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerBehaviorTree::Reset() {
		m_pcBehaviorTree->Reset();
		m_pcRobotState->Reset();
		// Restart actuation.
		InitializeActuation();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerBehaviorTree::SetBehaviorTree(AutoMoDeBehaviorTree* pc_behaviour_tree) {
		m_pcBehaviorTree = pc_behaviour_tree;
		m_pcBehaviorTree->SetRobotDAO(m_pcRobotState);
		m_pcBehaviorTree->Init();
		m_bBehaviorTreeGiven = true;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeControllerBehaviorTree::InitializeActuation() {}

	REGISTER_CONTROLLER(AutoMoDeControllerBehaviorTree, "automode_controller_bt");
}
