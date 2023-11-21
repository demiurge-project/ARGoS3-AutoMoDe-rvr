/**
  * @file <src/core/bt/SequenceStar.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe-BT
  *
  * @license MIT License
  */

#ifndef SEQUENCESTAR_H
#define SEQUENCESTAR_H

#include "Node.h"

namespace argos {
	class SequenceStar: public Node {
		public:
			SequenceStar();
			SequenceStar(SequenceStar* pc_behaviour);
			virtual ~SequenceStar();

			virtual ReturnState Tick();

			virtual void Reset();

			virtual SequenceStar* Clone();

			virtual std::string GetLabel();

      virtual void FillDOTDescription(std::ostringstream& ss_dot_string);

			// virtual void AddCondition(AutoMoDeCondition* pc_condition);
			//
			// virtual void AddAction(AutoMoDeBehaviour* pc_action);

      virtual void AddChildNode(Node* pc_new_child_node);

			virtual void ShareRobotDAO(RVRDAO* pc_robot_dao);

			UInt32 GetIndexRunningChild();

    private:
      UInt8 m_unIndexRunningChild;
	};
}

#endif
