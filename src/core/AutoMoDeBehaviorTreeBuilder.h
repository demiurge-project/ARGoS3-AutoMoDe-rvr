/*
 * @file <src/core/AutoMoDeBehaviorTreeBuilder.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 *
 * @brief This class is used to parse the configuration of
 * 				the stochastic finite state machine and instanciate the
 * 				different modules (behaviours and conditions). These modules
 * 				are added to the AutoMoDeFiniStateMachine created.
 */

#ifndef AUTOMODE_BEHAVIOR_TREE_BUILDER_H
#define AUTOMODE_BEHAVIOR_TREE_BUILDER_H

#include "AutoMoDeBehaviorTree.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <stdlib.h>

namespace argos {
	class AutoMoDeBehaviorTreeBuilder {
		public:
			/*
			 * Class constructor.
			 */
			AutoMoDeBehaviorTreeBuilder();

			/**
			 * Creates an AutoMoDeBehaviorTree based on a configuration as a vector of strings.
			 * This method should be called when the BehaviorTree is created from the AutoMoDeMain.cpp.
			 */
			AutoMoDeBehaviorTree* BuildBehaviorTree(std::vector<std::string>& vec_bt_config);

			/**
			 * Creates an AutoMoDeBehaviorTree based on a configuration as a string.
			 * This method should be called when the BehaviorTree is created from the AutoMoDeController.cpp.
			 */
			AutoMoDeBehaviorTree* BuildBehaviorTree(const std::string& str_bt_config);

			/*
			 * Class destructor.
			 */
			virtual ~AutoMoDeBehaviorTreeBuilder();

		private:
			/*
			 * Creates a AutoMoDeBehaviour from an action configuration.
			 */
			Action* HandleAction(std::vector<std::string>& vec_bt_action_config);

			/*
			 * Creates a AutoMoDeCondition from a condition configuration and add it to the
			 * parent node in construction.
			 */
			Condition* HandleCondition(std::vector<std::string>& vec_bt_condition_config);

			void HandleRootNode(AutoMoDeBehaviorTree* pc_behaviour_tree, std::vector<std::string>& vec_bt_root_node_config);

			void HandleChild(Node* pc_parent_node, std::vector<std::string>& vec_bt_child_config);

			/*
			 * Splits a subtree into more subtrees.
			 */
			std::vector<std::vector<std::string> > ExtractBranches(std::vector<std::string>& vec_sub_tree);

			/*
			 * Transforms action description into selector subtree with said action as first child, and Rotation as second child
			 */
			std::vector<std::string> TransformIntoSelectorSubtree(std::vector<std::string>& vecRemainingTree, std::string strNodeIdentifier);

			Node* ParseSubTree(std::vector<std::string>& vec_sub_tree);

			Node* GetNodeFromType(UInt8 un_node_type);

			AutoMoDeBehaviorTree* cBehaviorTree;

			UInt8 m_unMaxTreeLevel;

	};
}

#endif
