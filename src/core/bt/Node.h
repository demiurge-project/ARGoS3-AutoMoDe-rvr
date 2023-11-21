/*
 * @file <src/core/bt/Node.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe-BT
 *
 * @license MIT License
 */

#ifndef NODE_H
#define NODE_H

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/argos_exception.h>

#include <string>
#include <sstream>
#include <iostream>
#include <map>

#include "../../modules/AutoMoDeBehaviour.h"
#include "../../modules/AutoMoDeCondition.h"

namespace argos {
	class Node {
		protected:
			/*
			 * The type of the node.
			 */
			std::string m_strLabel;

      /*
       * List of child nodes.
       */
      std::vector<Node*> m_vecChilds;

			std::vector<AutoMoDeCondition*> m_vecConditions;
			std::vector<AutoMoDeBehaviour*> m_vecActions;

			std::string m_strBranchId;

			std::string m_strDOTLabel;

		public:

			enum ReturnState {
				SUCCESS,
				FAILURE,
				RUNNING
			};


		  virtual ~Node();

      /*
       * Cloning function.
       */
      virtual Node* Clone() = 0;

      /*
			 */
			virtual ReturnState Tick() = 0;

			virtual void Reset() = 0;

			virtual std::string GetLabel() = 0;

			/*
			 * Returns a string containing the DOT description of the node.
			 */
			virtual void FillDOTDescription(std::ostringstream& ss_dot_string) = 0;

			// /*
			//  * Adds condition to list of conditions.
			//  * If particular node should not have a condition leaf as child, throw an error!
			//  */
			// virtual void AddCondition(AutoMoDeCondition* pc_condition) = 0;
			//
			// /*
			//  * Adds action to list of actions.
			//  * If particular node should not have a action leaf as child, throw an error!
			//  */
			// virtual void AddAction(AutoMoDeBehaviour* pc_action) = 0;

			/*
			 * Adds node (Selector or SequenceStar) to list of childs.
			 */
			virtual void AddChildNode(Node* pc_new_child_node) = 0;

			virtual void ShareRobotDAO(RVRDAO* pc_robot_dao) = 0;

			std::vector<Node*> GetChildNodes();

			std::vector<AutoMoDeBehaviour*> GetActions();

			std::vector<AutoMoDeCondition*> GetConditions();

			std::string GetBranchId();

			void SetBranchId(std::string str_branch_id);

			std::string GetDOTLabel();

	};
}

#endif
