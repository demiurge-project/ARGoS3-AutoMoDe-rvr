/**
  * @file <src/core/bt/Condition.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe-BT
  *
  * @license MIT License
  */

#ifndef CONDITION_H
#define CONDITION_H

#include "Node.h"

namespace argos {
	class Condition: public Node {
		public:
			Condition();
			Condition(Condition* pc_behaviour);
			virtual ~Condition();

			virtual ReturnState Tick();

			virtual void Reset();

			virtual Condition* Clone();

			virtual std::string GetLabel();

      virtual void FillDOTDescription(std::ostringstream& ss_dot_string);

			// virtual void AddCondition(AutoMoDeCondition* pc_condition);
			//
			// virtual void AddCondition(AutoMoDeBehaviour* pc_action);

			virtual void AddChildNode(Node* pc_new_child_node);

			virtual void ShareRobotDAO(RVRDAO* pc_robot_dao);

			UInt32 m_unRobotID;

			void SetCondition(AutoMoDeCondition* pc_condition);

			AutoMoDeCondition* GetCondition();

		private:
			AutoMoDeCondition* m_pcCondition;
	};
}

#endif
