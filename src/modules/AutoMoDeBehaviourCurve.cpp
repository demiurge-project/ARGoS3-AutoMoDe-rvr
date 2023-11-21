/**
  * @file <src/modules/AutoMoDeBehaviourCurve.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "AutoMoDeBehaviourCurve.h"


namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourCurve::AutoMoDeBehaviourCurve() {
		m_strLabel = "Curve";
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourCurve::AutoMoDeBehaviourCurve(AutoMoDeBehaviourCurve* pc_behaviour) {
		m_strLabel = pc_behaviour->GetLabel();
		m_bLocked = pc_behaviour->IsLocked();
		m_bOperational = pc_behaviour->IsOperational();
		m_unIndex = pc_behaviour->GetIndex();
		m_unIdentifier = pc_behaviour->GetIdentifier();
		m_mapParameters = pc_behaviour->GetParameters();
		m_eCurveState = RIGHT;
		Init();
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourCurve::~AutoMoDeBehaviourCurve() {}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviourCurve* AutoMoDeBehaviourCurve::Clone() {
		return new AutoMoDeBehaviourCurve(this);
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourCurve::ControlStep() {
		Real fDistanceBetweenWheels = 5.3;
		Real fLeftSpeed;
		Real fRightSpeed;
		switch (m_eCurveState) {
			case IDLE: {
				if (m_pcRobotDAO->GetRandomNumberGenerator()->Bernoulli(0.5)) {
					fRightSpeed = (m_fRadiusParameter) * m_pcRobotDAO->GetMaxVelocity();
					fLeftSpeed = (m_fRadiusParameter + fDistanceBetweenWheels) * m_pcRobotDAO->GetMaxVelocity();
					m_eCurveState = RIGHT;
				} else {
					fLeftSpeed = (m_fRadiusParameter) * m_pcRobotDAO->GetMaxVelocity();
					fRightSpeed = (m_fRadiusParameter + fDistanceBetweenWheels) * m_pcRobotDAO->GetMaxVelocity();
					m_eCurveState = LEFT;
				}
				break;
			}
			case LEFT: {
				fLeftSpeed = (m_fRadiusParameter) * m_pcRobotDAO->GetMaxVelocity();
				fRightSpeed = (m_fRadiusParameter + fDistanceBetweenWheels) * m_pcRobotDAO->GetMaxVelocity();
				break;
			}
			case RIGHT: {
				fRightSpeed = (m_fRadiusParameter) * m_pcRobotDAO->GetMaxVelocity();
				fLeftSpeed = (m_fRadiusParameter + fDistanceBetweenWheels) * m_pcRobotDAO->GetMaxVelocity();
				break;
			}
		}

		Real fVelocityFactor = m_pcRobotDAO->GetMaxVelocity() / Max<Real>(std::abs(fRightSpeed), std::abs(fLeftSpeed));
		m_pcRobotDAO->SetWheelsVelocity(fLeftSpeed * fVelocityFactor, fRightSpeed * fVelocityFactor);

		m_bLocked = false;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourCurve::Init() {
		// Success probability
		std::map<std::string, Real>::iterator itProba = m_mapParameters.find("p");
		std::map<std::string, Real>::iterator itRadius = m_mapParameters.find("r");
		if ((itProba != m_mapParameters.end()) && (itRadius != m_mapParameters.end())) {
			m_fSuccessProbabilityParameter = itProba->second;
			m_fRadiusParameter = itRadius->second * 10;  // expressed in centimeters in grammar -> integer from 1 to 20
		} else {
			LOGERR << "[FATAL] Missing probability parameter for the following behaviour:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourCurve::Reset() {
		m_bOperational = false;
		ResumeStep();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviourCurve::ResumeStep() {
		m_bOperational = true;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeBehaviourCurve::Succeeded() {
		if (EvaluateBernoulliProbability(m_fSuccessProbabilityParameter)) {
			m_eCurveState = IDLE;
			return true;
		};
		return false;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeBehaviourCurve::Failed() {
		if (ObstacleInFront()) {
			m_eCurveState = IDLE;
			return true;
		};
		return false;
	}
}
