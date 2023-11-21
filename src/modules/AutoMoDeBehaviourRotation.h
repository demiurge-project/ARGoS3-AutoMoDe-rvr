/**
  * @file <src/modules/AutoMoDeBehaviourRotation.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */


#ifndef AUTOMODE_BEHAVIOUR_ROTATION_H
#define AUTOMODE_BEHAVIOUR_ROTATION_H

#include "AutoMoDeBehaviour.h"

namespace argos {
	class AutoMoDeBehaviourRotation: public AutoMoDeBehaviour {
		public:
			AutoMoDeBehaviourRotation();
			AutoMoDeBehaviourRotation(AutoMoDeBehaviourRotation* pc_behaviour);
			virtual ~AutoMoDeBehaviourRotation();

			virtual void ControlStep();
			virtual void Reset();
			virtual void ResumeStep();
			virtual void Init();

			virtual AutoMoDeBehaviourRotation* Clone();

			virtual bool Succeeded();
			virtual bool Failed();

		private:

			enum RotationState {
				IDLE, LEFT, RIGHT
			};

			RotationState m_eRotationState;
	};
}


#endif
