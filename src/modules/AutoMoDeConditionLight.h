/**
  * @file <src/modules/AutoMoDeConditionLight.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef AUTOMODE_CONDITION_LIGHT_H
#define AUTOMODE_CONDITION_LIGHT_H

#include "AutoMoDeCondition.h"

namespace argos {
	class AutoMoDeConditionLight: public AutoMoDeCondition {
		public:
			AutoMoDeConditionLight();
			virtual ~AutoMoDeConditionLight();

			AutoMoDeConditionLight(AutoMoDeConditionLight* pc_condition);
			virtual AutoMoDeConditionLight* Clone();

			virtual bool Verify();
			virtual void Reset();
			virtual void Init();

		private:
			Real m_fProbability;
	};
}

#endif
