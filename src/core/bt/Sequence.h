/**
  * @file <src/core/bt/Sequence.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe-BT
  *
  * @license MIT License
  */

#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "Node.h"

namespace argos {
	class Sequence: public Node {
		public:
			Sequence();
			Sequence(Sequence* pc_behaviour);
			virtual ~Sequence();

			virtual ReturnState Tick();

			virtual void Reset();

			virtual Sequence* Clone();

			virtual std::string GetLabel();

      virtual void FillDOTDescription(std::ostringstream& ss_dot_string);

			// virtual void AddCondition(AutoMoDeCondition* pc_condition);
      //
			// virtual void AddAction(AutoMoDeBehaviour* pc_action);

      virtual void AddChildNode(Node* pc_new_child_node);

			virtual void ShareRobotDAO(RVRDAO* pc_robot_dao);
	};
}

#endif
