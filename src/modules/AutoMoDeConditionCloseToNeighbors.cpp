/**
  * @file <src/modules/AutoMoDeConditionCloseToNeighbors.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

 #include "AutoMoDeConditionCloseToNeighbors.h"

 namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeConditionCloseToNeighbors::AutoMoDeConditionCloseToNeighbors() {
		m_strLabel = "CloseToNeighbors";
	}

	/****************************************/
	/****************************************/

	AutoMoDeConditionCloseToNeighbors::~AutoMoDeConditionCloseToNeighbors() {}

	/****************************************/
	/****************************************/

	AutoMoDeConditionCloseToNeighbors::AutoMoDeConditionCloseToNeighbors(AutoMoDeConditionCloseToNeighbors* pc_condition) {
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

	AutoMoDeConditionCloseToNeighbors* AutoMoDeConditionCloseToNeighbors::Clone() {
		return new AutoMoDeConditionCloseToNeighbors(this);
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeConditionCloseToNeighbors::Verify() {
    auto cLidarReading = m_pcRobotDAO->GetAttractionVectorToNeighbors(1.0f);
    //LOG << "[" << m_pcRobotDAO->GetRobotIdentifier() << "] " << cLidarReading.Value << std::endl;
    if (cLidarReading.Value <= m_fDistanceParameter) {
		    return EvaluateBernoulliProbability(m_fProbaParameter);
    }
    return false;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionCloseToNeighbors::Reset() {

	}

	/****************************************/
	/****************************************/

	void AutoMoDeConditionCloseToNeighbors::Init() {
    std::map<std::string, Real>::iterator itProba = m_mapParameters.find("p");
		std::map<std::string, Real>::iterator itDistance = m_mapParameters.find("d");
		if ((itProba != m_mapParameters.end()) && (itDistance != m_mapParameters.end())) {
			m_fProbaParameter = itProba->second;
			m_fDistanceParameter = itDistance->second;
		} else {
			LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
			THROW_ARGOSEXCEPTION("Missing Parameter");
		}
	}

 }
