/**
  * @file <src/core/bt/NegationDecorator.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe-BT
  *
  * @license MIT License
  */

#ifndef NEGATION_DECORATOR_H
#define NEGATION_DECORATOR_H

#include "Node.h"

namespace argos {
	class NegationDecorator: public Node {
		public:
			NegationDecorator();
			NegationDecorator(NegationDecorator* pc_behaviour);
			virtual ~NegationDecorator();

			virtual ReturnState Tick();

			virtual void Reset();

			virtual NegationDecorator* Clone();

			virtual std::string GetLabel();

      virtual void FillDOTDescription(std::ostringstream& ss_dot_string);

			// virtual void AddCondition(AutoMoDeCondition* pc_condition);
			//
			// virtual void AddAction(AutoMoDeBehaviour* pc_action);

			virtual void AddChildNode(Node* pc_new_child_node);

			virtual void ShareRobotDAO(RVRDAO* pc_robot_dao);

			UInt32 m_unRobotID;

	};
}

#endif
