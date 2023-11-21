/**
  * @file <src/core/bt/Selector.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe-BT
  *
  * @license MIT License
  */

#ifndef SELECTOR_H
#define SELECTOR_H

#include "Node.h"

namespace argos {
	class Selector: public Node {
		public:
			Selector();
			Selector(Selector* pc_behaviour);
			virtual ~Selector();

			virtual ReturnState Tick();

			virtual void Reset();

			virtual Selector* Clone();

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
