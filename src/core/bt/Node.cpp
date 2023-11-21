/*
 * @file <src/core/bt/Node.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe-BT
 *
 * @license MIT License
 */

#include "Node.h"

namespace argos {

  /****************************************/
  /****************************************/

	Node::~Node() {}

	/****************************************/
	/****************************************/

	std::string Node::GetBranchId(){
		return m_strBranchId;
	}

	/****************************************/
	/****************************************/

	void Node::SetBranchId(std::string str_branch_id) {
		m_strBranchId = str_branch_id;
	}

	/****************************************/
	/****************************************/

	std::string Node::GetDOTLabel() {
		return m_strDOTLabel;
	}

	/****************************************/
	/****************************************/

	std::vector<Node*> Node::GetChildNodes() {
		return m_vecChilds;
	}

	/****************************************/
	/****************************************/

	std::vector<AutoMoDeBehaviour*> Node::GetActions() {
		return m_vecActions;
	}

	/****************************************/
	/****************************************/

	std::vector<AutoMoDeCondition*> Node::GetConditions() {
		return m_vecConditions;
	}
}
