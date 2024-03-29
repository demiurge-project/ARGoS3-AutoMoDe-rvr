/**
 * @file <src/modules/AutoMoDeConditionBlackFloor.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeConditionBlackFloor.h"

namespace argos
{

  /****************************************/
  /****************************************/

  AutoMoDeConditionBlackFloor::AutoMoDeConditionBlackFloor()
  {
    m_strLabel = "BlackFloor";
  }

  /****************************************/
  /****************************************/

  AutoMoDeConditionBlackFloor::~AutoMoDeConditionBlackFloor() {}

  /****************************************/
  /****************************************/

  AutoMoDeConditionBlackFloor::AutoMoDeConditionBlackFloor(AutoMoDeConditionBlackFloor *pc_condition)
  {
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

  void AutoMoDeConditionBlackFloor::Init()
  {
    m_fGroundThreshold = 0.10f;
    std::map<std::string, Real>::iterator it = m_mapParameters.find("p");
    if (it != m_mapParameters.end())
    {
      m_fProbability = it->second;
    }
    else
    {
      LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
      THROW_ARGOSEXCEPTION("Missing Parameter");
    }
  }

  /****************************************/
  /****************************************/

  AutoMoDeConditionBlackFloor *AutoMoDeConditionBlackFloor::Clone()
  {
    return new AutoMoDeConditionBlackFloor(this);
  }

  /****************************************/
  /****************************************/

  bool AutoMoDeConditionBlackFloor::Verify()
  {
    if (m_pcRobotDAO->GetGroundReading().ToGrayScale() <= m_fGroundThreshold)
    {
      return EvaluateBernoulliProbability(m_fProbability);
    }
    else
    {
      return false;
    }
  }

  /****************************************/
  /****************************************/

  void AutoMoDeConditionBlackFloor::Reset()
  {
    Init();
  }

}
