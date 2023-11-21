/**
  * @file <src/core/bt/Action.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe-BT
  *
  * @license MIT License
  */

#ifndef ACTION_H
#define ACTION_H

#include "Node.h"

namespace argos {
	class Action: public Node {
		public:
			Action();
			Action(Action* pc_behaviour);
			virtual ~Action();

			virtual ReturnState Tick();

			virtual void Reset();

			virtual Action* Clone();

			virtual std::string GetLabel();

      virtual void FillDOTDescription(std::ostringstream& ss_dot_string);

			// virtual void AddCondition(AutoMoDeCondition* pc_condition);
			//
			// virtual void AddAction(AutoMoDeBehaviour* pc_action);

			virtual void AddChildNode(Node* pc_new_child_node);

			virtual void ShareRobotDAO(RVRDAO* pc_robot_dao);

			UInt32 m_unRobotID;

			void SetBehaviour(AutoMoDeBehaviour* pc_behaviour);

			AutoMoDeBehaviour* GetBehaviour();

		private:
			AutoMoDeBehaviour* m_pcBehaviour;
	};
}

#endif
