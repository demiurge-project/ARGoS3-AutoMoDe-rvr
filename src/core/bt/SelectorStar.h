/**
  * @file <src/core/bt/SelectorStar.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe-BT
  *
  * @license MIT License
  */

#ifndef SELECTORSTAR_H
#define SELECTORSTAR_H

#include "Node.h"

namespace argos {
	class SelectorStar: public Node {
		public:
			SelectorStar();
			SelectorStar(SelectorStar* pc_behaviour);
			virtual ~SelectorStar();

			virtual ReturnState Tick();

			virtual void Reset();

			virtual SelectorStar* Clone();

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
