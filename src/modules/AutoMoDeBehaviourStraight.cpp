/**
  * @file <src/modules/AutoMoDeBehaviourStraight.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "AutoMoDeBehaviourStraight.h"


namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourStraight::AutoMoDeBehaviourStraight() {
		m_strLabel = "Straight";
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourStraight::AutoMoDeBehaviourStraight(AutoMoDeBehaviourStraight* pc_behaviour) {
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

	AutoMoDeBehaviourStraight::~AutoMoDeBehaviourStraight() {}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourStraight* AutoMoDeBehaviourStraight::Clone() {
		return new AutoMoDeBehaviourStraight(this);
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourStraight::ControlStep() {
		m_pcRobotDAO->SetWheelsVelocity(m_pcRobotDAO->GetMaxVelocity(), m_pcRobotDAO->GetMaxVelocity());
		m_bLocked = false;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourStraight::Init() {
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

	void AutoMoDeBehaviourStraight::Reset() {
		m_bOperational = false;
		ResumeStep();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourStraight::ResumeStep() {
		m_bOperational = true;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeBehaviourStraight::Succeeded() {
		return EvaluateBernoulliProbability(m_fSuccessProbabilityParameter);
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeBehaviourStraight::Failed() {
		return ObstacleInFront();
	}
}
