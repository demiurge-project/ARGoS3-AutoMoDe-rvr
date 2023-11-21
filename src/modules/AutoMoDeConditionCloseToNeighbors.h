/**
  * @file <src/modules/AutoMoDeConditionCloseToNeighbors.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef AUTOMODE_CONDITION_CLOSE_TO_NEIGHBORS_H
#define AUTOMODE_CONDITION_CLOSE_TO_NEIGHBORS_H

#include "AutoMoDeCondition.h"

namespace argos {
	class AutoMoDeConditionCloseToNeighbors: public AutoMoDeCondition {
		public:
			AutoMoDeConditionCloseToNeighbors();
			virtual ~AutoMoDeConditionCloseToNeighbors();

			AutoMoDeConditionCloseToNeighbors(AutoMoDeConditionCloseToNeighbors* pc_condition);
			virtual AutoMoDeConditionCloseToNeighbors* Clone();

			virtual bool Verify();
			virtual void Reset();
			virtual void Init();

		private:
			Real m_fProbaParameter;
			Real m_fDistanceParameter;
	};
}

#endif
