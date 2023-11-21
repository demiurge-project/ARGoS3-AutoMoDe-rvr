/**
  * @file <src/modules/AutoMoDeConditionObstacleInFront.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef AUTOMODE_CONDITION_OIF_H
#define AUTOMODE_CONDITION_OIF_H

#include "AutoMoDeCondition.h"

namespace argos {
	class AutoMoDeConditionObstacleInFront: public AutoMoDeCondition {
		public:
			AutoMoDeConditionObstacleInFront();
			virtual ~AutoMoDeConditionObstacleInFront();

			AutoMoDeConditionObstacleInFront(AutoMoDeConditionObstacleInFront* pc_condition);
			virtual AutoMoDeConditionObstacleInFront* Clone();

			virtual bool Verify();
			virtual void Reset();
			virtual void Init();

		private:
			Real m_fProbability;
	};
}

#endif
