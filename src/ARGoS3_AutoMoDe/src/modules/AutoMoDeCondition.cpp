/**
 * @file <src/modules/AutoMoDeCondition.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeCondition.h"
#include <limits>

namespace argos
{

	/****************************************/
	/****************************************/

	const std::string AutoMoDeCondition::GetDOTDescription()
	{
		std::stringstream ss;
		ss << m_strLabel;
		if (!m_mapParameters.empty())
		{
			std::map<std::string, Real>::iterator it;
			for (it = m_mapParameters.begin(); it != m_mapParameters.end(); it++)
			{
				ss << "\\n"
				   << it->first << "=" << it->second;
			}
		}
		return ss.str();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::AddParameter(const std::string &str_identifier, const Real &f_value)
	{
		m_mapParameters.insert(std::pair<std::string, Real>(str_identifier, f_value));
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetOrigin() const
	{
		return m_unFromBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetExtremity() const
	{
		return m_unToBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOrigin(const UInt32 &un_from)
	{
		m_unFromBehaviourIndex = un_from;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetExtremity(const UInt32 &un_to)
	{
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOriginAndExtremity(const UInt32 &un_from, const UInt32 &un_to)
	{
		m_unFromBehaviourIndex = un_from;
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	const std::string &AutoMoDeCondition::GetLabel() const
	{
		return m_strLabel;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIndex(const UInt32 &un_index)
	{
		m_unIndex = un_index;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetIndex() const
	{
		return m_unIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIdentifier(const UInt32 &un_id)
	{
		m_unIdentifier = un_id;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetIdentifier() const
	{
		return m_unIdentifier;
	}

	/****************************************/
	/****************************************/

	std::map<std::string, Real> AutoMoDeCondition::GetParameters() const
	{
		return m_mapParameters;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetRobotDAO(RVRDAO *pc_robot_dao)
	{
		m_pcRobotDAO = pc_robot_dao;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeCondition::EvaluateBernoulliProbability(const Real &f_probability) const
	{
		return m_pcRobotDAO->GetRandomNumberGenerator()->Bernoulli(f_probability);
	}

	/****************************************/
	/****************************************/

	// Return the color parameter
	CColor AutoMoDeCondition::GetColorParameter(const UInt32 &un_value)
	{
		CColor cColorParameter;
		switch (un_value)
		{
		case 0:
			cColorParameter = CColor::BLACK;
			break;
		case 1:
			cColorParameter = CColor::GREEN;
			break;
		case 2:
			// blue physical patches
			cColorParameter = CColor(0, 123, 194);
			break;
		case 3:
			// red physical patches
			cColorParameter = CColor(228, 53, 64);
			break;
		case 4:
			// yellow physical patches
			cColorParameter = CColor(252, 238, 33);
			break;
		case 5:
			// purple physical patches
			cColorParameter = CColor(126, 79, 154);
			break;
		case 6:
			cColorParameter = CColor::CYAN;
			break;
		default:
			cColorParameter = CColor::BLACK;
		}
		return cColorParameter;
	}

	CColor AutoMoDeCondition::Saturate(CColor pc_color)
	{
		CColor fSaturatedColor = CColor();
		UInt8 cMaxChannelValue = pc_color.GetRed();
		if (pc_color.GetGreen() > cMaxChannelValue)
		{
			cMaxChannelValue = pc_color.GetGreen();
		}
		if (pc_color.GetBlue() > cMaxChannelValue)
		{
			cMaxChannelValue = pc_color.GetBlue();
		}
		if (cMaxChannelValue <= 10)
		{
			return CColor::BLACK;
		}
		Real fFactor = 255.0 / cMaxChannelValue;
		fSaturatedColor.SetRed(UInt8(pc_color.GetRed() * fFactor));
		fSaturatedColor.SetGreen(UInt8(pc_color.GetGreen() * fFactor));
		fSaturatedColor.SetBlue(UInt8(pc_color.GetBlue() * fFactor));
		if (fSaturatedColor.GetRed() > 255)
		{
			fSaturatedColor.SetRed(255);
		}
		if (fSaturatedColor.GetGreen() > 255)
		{
			fSaturatedColor.SetGreen(255);
		}
		if (fSaturatedColor.GetBlue() > 255)
		{
			fSaturatedColor.SetBlue(255);
		}
		return fSaturatedColor;
	}

	/****************************************/
	/****************************************/

	CColor AutoMoDeCondition::GetClosestLabel(CColor pc_color)
	{
		CColor cClosestLabel = CColor();
		Real fMinDistance = std::numeric_limits<Real>::max();
		for (UInt32 i = 0; i < 7; i++)
		{
			CColor cLabel = GetColorParameter(i);
			Real fDistance = (CVector3(cLabel.GetRed(), cLabel.GetGreen(), cLabel.GetBlue()) - CVector3(pc_color.GetRed(), pc_color.GetGreen(), pc_color.GetBlue())).Length();
			if (fDistance < fMinDistance)
			{
				fMinDistance = fDistance;
				cClosestLabel = cLabel;
			}
		}
		return cClosestLabel;
	}

}
