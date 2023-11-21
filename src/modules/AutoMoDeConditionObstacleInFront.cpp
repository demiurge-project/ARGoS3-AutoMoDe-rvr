/**
  * @file <src/modules/AutoMoDeConditionObstacleInFront.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

 #include "AutoMoDeConditionObstacleInFront.h"

 namespace argos {

  /****************************************/
  /****************************************/

	AutoMoDeConditionObstacleInFront::AutoMoDeConditionObstacleInFront() {
		m_strLabel = "OIF";
	}

  /****************************************/
  /****************************************/

	AutoMoDeConditionObstacleInFront::~AutoMoDeConditionObstacleInFront() {}

  /****************************************/
  /****************************************/

	AutoMoDeConditionObstacleInFront::AutoMoDeConditionObstacleInFront(AutoMoDeConditionObstacleInFront* pc_condition) {
		m_strLabel = pc_condition->GetLabel();
		m_unIndex = pc_condition->GetIndex();
		m_unIdentifier = pc_condition->GetIndex();
		m_unFromBehaviourIndex = pc_condition->GetOrigin();
		m_unToBehaviourIndex = pc_condition->GetExtremity();
		m_mapParameters = pc_condition->GetParameters();
    Init();
	}

  /****************************************/
  /****************************************/

  void AutoMoDeConditionObstacleInFront::Init() {
	  std::map<std::string, Real>::iterator it = m_mapParameters.find("p");
    if (it != m_mapParameters.end()) {
      m_fProbability = it->second;
    } else {
      LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
      THROW_ARGOSEXCEPTION("Missing Parameter");
	  }
  }

  /****************************************/
  /****************************************/

	AutoMoDeConditionObstacleInFront* AutoMoDeConditionObstacleInFront::Clone() {
		return new AutoMoDeConditionObstacleInFront(this);
	}

  /****************************************/
  /****************************************/

	bool AutoMoDeConditionObstacleInFront::Verify() {
    CRadians cAngle = m_pcRobotDAO->GetProximityReading().Angle;
    if ((m_pcRobotDAO->GetProximityReading().Value >= 0.5) && ((cAngle <= CRadians::PI_OVER_TWO) || (cAngle >= (CRadians::TWO_PI - CRadians::PI_OVER_TWO)))) {
      //LOG << "ObstacleInFront" << std::endl;
      return EvaluateBernoulliProbability(m_fProbability);
;
    }
    return false;
	}

  /****************************************/
  /****************************************/

	void AutoMoDeConditionObstacleInFront::Reset() {
    Init();
  }

 }
