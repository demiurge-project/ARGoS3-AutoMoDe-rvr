/**
  * @file <src/modules/AutoMoDeBehaviourCurve.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */


#ifndef AUTOMODE_BEHAVIOUR_CURVE_H
#define AUTOMODE_BEHAVIOUR_CURVE_H

#include "AutoMoDeBehaviour.h"

namespace argos {
	class AutoMoDeBehaviourCurve: public AutoMoDeBehaviour {
		public:
			AutoMoDeBehaviourCurve();
			AutoMoDeBehaviourCurve(AutoMoDeBehaviourCurve* pc_behaviour);
			virtual ~AutoMoDeBehaviourCurve();

			virtual void ControlStep();
			virtual void Reset();
			virtual void ResumeStep();
			virtual void Init();

			virtual AutoMoDeBehaviourCurve* Clone();

			virtual bool Succeeded();
			virtual bool Failed();

		private:
			Real m_fRadiusParameter;

			enum CurveState {
				IDLE, LEFT, RIGHT
			};

			CurveState m_eCurveState;

	};
}


#endif
