/*
 * @file <src/core/bt/SequenceStar.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe-BT
 *
 * @license MIT License
 */

#include "SequenceStar.h"

namespace argos {

  /****************************************/
  /****************************************/

	SequenceStar::SequenceStar() {
    m_strLabel = "SequenceStar";
		m_strDOTLabel = "   -->*   ";
    m_unIndexRunningChild = 0;
		m_strBranchId = "";
  }

  /****************************************/
	/****************************************/

  SequenceStar::~SequenceStar() {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			delete m_vecChilds.at(i);
		}
	}

  /****************************************/
  /****************************************/

	SequenceStar::SequenceStar(SequenceStar* pc_sequencestar) {
		m_unIndexRunningChild = pc_sequencestar->GetIndexRunningChild();

		std::vector<Node*> vecChildNodes = pc_sequencestar->GetChildNodes();
		m_vecChilds.clear();
		for (std::vector<Node*>::iterator it = vecChildNodes.begin(); it != vecChildNodes.end(); ++it) {
			m_vecChilds.push_back((*it)->Clone());
		}
	}

	/****************************************/
	/****************************************/

	Node::ReturnState SequenceStar::Tick() {
		//LOG << m_strLabel << m_strBranchId << std::endl;
		Node::ReturnState eCurrentState;
		for (UInt8 i = m_unIndexRunningChild; i < m_vecChilds.size(); i++) {
			eCurrentState = m_vecChilds.at(i)->Tick();
			if (eCurrentState == Node::RUNNING) {
				m_unIndexRunningChild = i;
				return Node::RUNNING;
			} else if (eCurrentState == Node::FAILURE) {
				m_unIndexRunningChild = 0;
				return Node::FAILURE;
			}
		}
		m_unIndexRunningChild = 0;
		return Node::SUCCESS;
 	}

	/****************************************/
	/****************************************/

	void SequenceStar::Reset() {
		m_unIndexRunningChild = 0;
	}

	/****************************************/
	/****************************************/

	SequenceStar* SequenceStar::Clone() {
		return new SequenceStar(this);
	}

	/****************************************/
	/****************************************/

	std::string SequenceStar::GetLabel() {
		return m_strLabel;
	}

  /****************************************/
  /****************************************/

  void SequenceStar::FillDOTDescription(std::ostringstream& ss_dot_string) {
		// Creation of graphical nodes
		ss_dot_string << "node [shape = square];";
		ss_dot_string << m_strLabel << m_strBranchId << " [label=\"" << m_strDOTLabel << "\"];";
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			ss_dot_string << m_vecChilds.at(i)->GetLabel() << m_vecChilds.at(i)->GetBranchId() << " [label=\"" << m_vecChilds.at(i)->GetDOTLabel() << "\"];";
		}

		// Linking nodes
    for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			ss_dot_string << m_strLabel << m_strBranchId << " -> " << m_vecChilds.at(i)->GetLabel() << m_vecChilds.at(i)->GetBranchId() << ";";
		}

		// Ask childs to fill DOT description
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			 m_vecChilds.at(i)->FillDOTDescription(ss_dot_string);
		}
  }

  /****************************************/
  /****************************************/

  void SequenceStar::AddChildNode(Node* pc_new_child_node) {
    m_vecChilds.push_back(pc_new_child_node);
  }

	/****************************************/
	/****************************************/

	UInt32 SequenceStar::GetIndexRunningChild() {
		return m_unIndexRunningChild;
	}

	/****************************************/
	/****************************************/

	// void SequenceStar::AddAction(AutoMoDeBehaviour* pc_action) {
	// 	THROW_ARGOSEXCEPTION("As of now, a SequenceStar node should not have an Action as child");
	// }
	//
	// /****************************************/
	// /****************************************/
	//
	// void SequenceStar::AddCondition(AutoMoDeCondition* pc_condition) {
	// 	THROW_ARGOSEXCEPTION("As of now, a SequenceStar node should not have a Condition as child");
	// }

	/****************************************/
	/****************************************/

	void SequenceStar::ShareRobotDAO(RVRDAO* pc_robot_dao) {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			m_vecChilds.at(i)->ShareRobotDAO(pc_robot_dao);
		}
	}
}
