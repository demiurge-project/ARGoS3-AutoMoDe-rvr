/**
  * @file <src/modules/AutoMoDeBehaviourStraight.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */


#ifndef AUTOMODE_BEHAVIOUR_STRAIGHT_H
#define AUTOMODE_BEHAVIOUR_STRAIGHT_H

#include "AutoMoDeBehaviour.h"

namespace argos {
	class AutoMoDeBehaviourStraight: public AutoMoDeBehaviour {
		public:
			AutoMoDeBehaviourStraight();
			AutoMoDeBehaviourStraight(AutoMoDeBehaviourStraight* pc_behaviour);
			virtual ~AutoMoDeBehaviourStraight();

			virtual void ControlStep();
			virtual void Reset();
			virtual void ResumeStep();
			virtual void Init();

			virtual AutoMoDeBehaviourStraight* Clone();

			virtual bool Succeeded();
			virtual bool Failed();

		private:
	};
}


#endif
