/*
 * @file <src/core/bt/Sequence.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe-BT
 *
 * @license MIT License
 */

#include "Sequence.h"

namespace argos {

  /****************************************/
  /****************************************/

	Sequence::Sequence() {
    m_strLabel = "Sequence";
		m_strDOTLabel = "    -->    ";
		m_strBranchId = "";
  }

  /****************************************/
	/****************************************/

  Sequence::~Sequence() {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			delete m_vecChilds.at(i);
		}
	}

  /****************************************/
  /****************************************/

	Sequence::Sequence(Sequence* pc_sequence) {
		std::vector<Node*> vecChildNodes = pc_sequence->GetChildNodes();
		m_vecChilds.clear();
		for (std::vector<Node*>::iterator it = vecChildNodes.begin(); it != vecChildNodes.end(); ++it) {
			m_vecChilds.push_back((*it)->Clone());
		}
	}

	/****************************************/
	/****************************************/

	Node::ReturnState Sequence::Tick() {
		//LOG << m_strLabel << m_strBranchId << std::endl;
		Node::ReturnState eCurrentState;
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			eCurrentState = m_vecChilds.at(i)->Tick();
			if (eCurrentState == Node::RUNNING) {
				return Node::RUNNING;
			} else if (eCurrentState == Node::FAILURE) {
        return Node::FAILURE;
      }
		}
		return Node::SUCCESS;
 	}

	/****************************************/
	/****************************************/

	void Sequence::Reset() {}

	/****************************************/
	/****************************************/

	Sequence* Sequence::Clone() {
		return new Sequence(this);
	}

	/****************************************/
	/****************************************/

	std::string Sequence::GetLabel() {
		return m_strLabel;
	}

  /****************************************/
  /****************************************/

  void Sequence::FillDOTDescription(std::ostringstream& ss_dot_string) {
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

  void Sequence::AddChildNode(Node* pc_new_child_node) {
    m_vecChilds.push_back(pc_new_child_node);
  }

	/****************************************/
	/****************************************/

	// void Sequence::AddAction(AutoMoDeBehaviour* pc_action) {
	// 	THROW_ARGOSEXCEPTION("As of now, a Sequence node should not have an Action as child");
	// }
  //
	// /****************************************/
	// /****************************************/
  //
	// void Sequence::AddCondition(AutoMoDeCondition* pc_condition) {
	// 	THROW_ARGOSEXCEPTION("As of now, a Sequence node should not have a Condition as child");
	// }

	/****************************************/
	/****************************************/

	void Sequence::ShareRobotDAO(RVRDAO* pc_robot_dao) {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			m_vecChilds.at(i)->ShareRobotDAO(pc_robot_dao);
		}
	}
}
