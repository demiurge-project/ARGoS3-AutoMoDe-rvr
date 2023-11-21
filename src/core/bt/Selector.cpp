/*
 * @file <src/core/bt/Selector.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe-BT
 *
 * @license MIT License
 */

#include "Selector.h"

namespace argos {

  /****************************************/
  /****************************************/

	Selector::Selector() {
    m_strLabel = "Selector";
		m_strDOTLabel = "     ?     ";
		m_strBranchId = "";
  }

  /****************************************/
	/****************************************/

  Selector::~Selector() {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			delete m_vecChilds.at(i);
		}
	}

  /****************************************/
  /****************************************/


	Selector::Selector(Selector* pc_selector) {
		std::vector<Node*> vecChildNodes = pc_selector->GetChildNodes();
		m_vecChilds.clear();
		for (std::vector<Node*>::iterator it = vecChildNodes.begin(); it != vecChildNodes.end(); ++it) {
			m_vecChilds.push_back((*it)->Clone());
		}
	}

	/****************************************/
	/****************************************/

	Node::ReturnState Selector::Tick() {
		//LOG << m_strLabel << m_strBranchId << std::endl;
		Node::ReturnState eCurrentState;
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			eCurrentState = m_vecChilds.at(i)->Tick();
			if (eCurrentState == Node::RUNNING) {
				return Node::RUNNING;
			} else if (eCurrentState == Node::SUCCESS) {
        return Node::SUCCESS;
      }
		}
		return Node::FAILURE;
	}

	/****************************************/
	/****************************************/

	void Selector::Reset() {}

	/****************************************/
	/****************************************/

	Selector* Selector::Clone() {
		return new Selector(this);
	}

  /****************************************/
  /****************************************/

  void Selector::FillDOTDescription(std::ostringstream& ss_dot_string){
		ss_dot_string << "node [shape=square;];";
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

	std::string Selector::GetLabel() {
	 return m_strLabel;
	}

	/****************************************/
	/****************************************/

	void Selector::AddChildNode(Node* pc_new_child_node) {
		m_vecChilds.push_back(pc_new_child_node);
	}

	/****************************************/
	/****************************************/

	void Selector::ShareRobotDAO(RVRDAO* pc_robot_dao) {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			m_vecChilds.at(i)->ShareRobotDAO(pc_robot_dao);
		}
	}
}
