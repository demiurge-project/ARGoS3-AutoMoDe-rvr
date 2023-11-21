/**
  * @file <src/modules/AutoMoDeConditionLight.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

 #include "AutoMoDeConditionLight.h"

 namespace argos {

  /****************************************/
  /****************************************/

	AutoMoDeConditionLight::AutoMoDeConditionLight() {
		m_strLabel = "Light";
	}

  /****************************************/
  /****************************************/

	AutoMoDeConditionLight::~AutoMoDeConditionLight() {}

  /****************************************/
  /****************************************/

	AutoMoDeConditionLight::AutoMoDeConditionLight(AutoMoDeConditionLight* pc_condition) {
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

  void AutoMoDeConditionLight::Init() {
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

	AutoMoDeConditionLight* AutoMoDeConditionLight::Clone() {
		return new AutoMoDeConditionLight(this);
	}

  /****************************************/
  /****************************************/

	bool AutoMoDeConditionLight::Verify() {
		CCI_RVRLightSensor::SReading cLightReading = m_pcRobotDAO->GetLightReading();
		if (cLightReading.Value == 1) {
      return EvaluateBernoulliProbability(m_fProbability);
    } else {
      return false;
    }
	}

  /****************************************/
  /****************************************/

	void AutoMoDeConditionLight::Reset() {
    Init();
  }

 }
