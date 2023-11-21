/**
  * @file <src/modules/AutoMoDeBehaviourRotation.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "AutoMoDeBehaviourRotation.h"


namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRotation::AutoMoDeBehaviourRotation() {
		m_strLabel = "Rotation";
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRotation::AutoMoDeBehaviourRotation(AutoMoDeBehaviourRotation* pc_behaviour) {
		m_strLabel = pc_behaviour->GetLabel();
		m_bLocked = pc_behaviour->IsLocked();
		m_bOperational = pc_behaviour->IsOperational();
		m_unIndex = pc_behaviour->GetIndex();
		m_unIdentifier = pc_behaviour->GetIdentifier();
		m_mapParameters = pc_behaviour->GetParameters();
		Init();
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRotation::~AutoMoDeBehaviourRotation() {}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourRotation* AutoMoDeBehaviourRotation::Clone() {
		return new AutoMoDeBehaviourRotation(this);
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRotation::ControlStep() {
		// switch (m_eRotationState) {
		// 	case IDLE: {
		// 		LOG << m_pcRobotDAO->GetProximityReading().Angle << std::endl;
		// 		if ((m_pcRobotDAO->GetProximityReading().Angle > CRadians::ZERO) || m_pcRobotDAO->GetRandomNumberGenerator()->Bernoulli(0.5)) {
		// 			m_pcRobotDAO->SetWheelsVelocity(m_pcRobotDAO->GetMaxVelocity(), -m_pcRobotDAO->GetMaxVelocity());
		// 			m_eRotationState = RIGHT;
		// 		} else {
		// 			m_pcRobotDAO->SetWheelsVelocity(-m_pcRobotDAO->GetMaxVelocity(), m_pcRobotDAO->GetMaxVelocity());
		// 			m_eRotationState = LEFT;
		// 		}
		// 		break;
		// 	}
		// 	case LEFT: {
		// 		m_pcRobotDAO->SetWheelsVelocity(-m_pcRobotDAO->GetMaxVelocity(), m_pcRobotDAO->GetMaxVelocity());
		// 		break;
		// 	}
		// 	case RIGHT: {
		// 		m_pcRobotDAO->SetWheelsVelocity(m_pcRobotDAO->GetMaxVelocity(), -m_pcRobotDAO->GetMaxVelocity());
		// 		break;
		// 	}
		// }

		if ((m_pcRobotDAO->GetProximityReading().Angle > CRadians::ZERO)) {
			m_pcRobotDAO->SetWheelsVelocity(m_pcRobotDAO->GetMaxVelocity(), -m_pcRobotDAO->GetMaxVelocity());
			//LOG << "RIGHT" << std::endl;
		} else {
			m_pcRobotDAO->SetWheelsVelocity(-m_pcRobotDAO->GetMaxVelocity(), m_pcRobotDAO->GetMaxVelocity());
			//LOG << "LEFT" << std::endl;
		}
		m_bLocked = false;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRotation::Init() {
		// Success probability
		std::map<std::string, Real>::iterator it = m_mapParameters.find("p");
		if (it != m_mapParameters.end()) {
			m_fSuccessProbabilityParameter = it->second;
		} else {
			LOGERR << "[FATAL] Missing probability parameter for the following behaviour:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRotation::Reset() {
		m_eRotationState = IDLE;
		m_bOperational = false;
		ResumeStep();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourRotation::ResumeStep() {
		m_bOperational = true;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeBehaviourRotation::Succeeded() {
		if (EvaluateBernoulliProbability(m_fSuccessProbabilityParameter)) {
			m_eRotationState = IDLE;
			return true;
		};
		return false;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeBehaviourRotation::Failed() {
		return false;
	}
}
