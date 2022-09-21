#ifndef LIBTREE_H
#define LIBTREE_H

#include <algorithm>
#include <vector>

#include <QString>
#include <QDebug>

#include "occ.h"

using NodeId = uint32_t;

struct Assemly_Data{
    QString name;
    bool isAssembly;
    TDF_Label label;
};

struct Assemly_Node{
public:
    NodeId siblingPrevious;
    NodeId siblingNext;
    NodeId childFirst;
    NodeId childLast;
    NodeId parent;

    Assemly_Data data;
    bool isDeleted;
};

class Assemly_Tree{
public:
    Assemly_Tree(){}
    Assemly_Node* AddNode(Assemly_Node* ParentNode,Assemly_Data& data)
    {
        return  nullptr;
    }

    Assemly_Node* AddNode(NodeId parentId,Assemly_Data& data)
    {
        Assemly_Node* node = this->appendChild(parentId);
        qDebug()<<"AddNode";
        node->data = data;
        return node;
    }
    Assemly_Node* GetRootNode()
    {
        return  nullptr;
    }
    NodeId lastNodeId() const
    {
        return static_cast<NodeId>(m_vecNode.size());
    }

    Assemly_Node* ptrNode(NodeId id)
    {
        return id != 0 && id <= m_vecNode.size() ? &m_vecNode.at(id - 1) : nullptr;
    }
    const Assemly_Node* ptrNode(NodeId id) const
    {
        return id != 0 && id <= m_vecNode.size() ? &m_vecNode.at(id - 1) : nullptr;
    }

    Assemly_Node* appendChild(NodeId parentId)
    {
        m_vecNode.push_back({});
        const NodeId nodeId = this->lastNodeId();
        Assemly_Node* node = &m_vecNode.back();
        node->parent = parentId;
        node->siblingPrevious = this->nodeChildLast(parentId);
        if (parentId != 0) {
            Assemly_Node* parentNode = this->ptrNode(parentId);
            if (parentNode->childFirst == 0)
                parentNode->childFirst = nodeId;

            if (parentNode->childLast != 0)
                this->ptrNode(parentNode->childLast)->siblingNext = nodeId;

            parentNode->childLast = nodeId;
        }
        else {
            m_vecRoot.push_back(nodeId);
        }

        return node;
    }

    QString findLabelName(const TDF_Label& lbl)
    {


        //        Handle_TDataStd_Name attrName;
        //        if (lbl.FindAttribute(TDataStd_Name::GetID(), attrName)) {
        //            return attrName.;
        //        }

        //        QString(attrName->Get().Value());

        //        else {
        //            static const TCollection_ExtendedString nullStr = {};
        //            return nullStr;
        //        }

        QString name;

        if (name.isEmpty()) {
            if (XCAFDoc_ShapeTool::IsShape(lbl)) {

                const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(lbl);
                switch (shape.ShapeType()) {
                case TopAbs_COMPOUND: name = "Compound"; break;
                case TopAbs_COMPSOLID: name = "CompSolid"; break;
                case TopAbs_SOLID: name = "Solid"; break;
                case TopAbs_SHELL: name = "Shell"; break;
                case TopAbs_FACE: name = "Face"; break;
                case TopAbs_WIRE: name = "Wire"; break;
                case TopAbs_EDGE: name = "Edge"; break;
                case TopAbs_VERTEX: name = "Vertex"; break;
                case TopAbs_SHAPE: name = "Shape"; break;
                }
                name = QString("%1 %2").arg(name).arg(lbl.Tag());
            }
            else {
                static thread_local TCollection_AsciiString entry;
                TDF_Tool::Entry(lbl, entry);
                //name = QString("[[%1]]").arg(entry);
            }
        }

        return name;
    }

    NodeId appendChildID(NodeId parentId,const TDF_Label& label)
    {
        Assemly_Node* node = this->appendChild(parentId);
        auto name= findLabelName(label);
        node->data.label = label;
        node->data.name=name;
        return this->lastNodeId();
    }
    bool isNodeDeleted(NodeId id) const
    {
        auto ptrNode = this->ptrNode(id);
        return !ptrNode || ptrNode->isDeleted;
    }

    NodeId nodeSiblingPrevious(NodeId id) const
    {
        const Assemly_Node* node = this->ptrNode(id);
        return node ? node->siblingPrevious : 0;
    }
    NodeId nodeSiblingNext(NodeId id) const
    {
        const Assemly_Node* node = this->ptrNode(id);
        return node ? node->siblingNext : 0;
    }
    NodeId nodeChildFirst(NodeId id) const
    {
        const Assemly_Node* node = this->ptrNode(id);
        return node ? node->childFirst : 0;
    }
    NodeId nodeChildLast(NodeId id) const
    {
        const Assemly_Node* node = this->ptrNode(id);
        return node ? node->childLast : 0;
    }
    NodeId nodeParent(NodeId id) const
    {
        const Assemly_Node* node = this->ptrNode(id);
        return node ? node->parent : 0;
    }
    NodeId nodeRoot(NodeId id) const
    {
        while (!this->nodeIsRoot(id))
            id = this->nodeParent(id);

        return id;
    }
    const Assemly_Data& nodeData(NodeId id) const
    {
        static const Assemly_Data nullObject = {};
        const Assemly_Node* node = this->ptrNode(id);
        return node ? node->data : nullObject;
    }
    bool nodeIsRoot(NodeId id) const
    {
        const Assemly_Node* node = this->ptrNode(id);
        return node ? node->parent  == 0 : false;
    }
    bool nodeIsLeaf(NodeId id) const
    {
        return this->nodeChildFirst(id) == 0;
    }

    std::vector<Assemly_Node> m_vecNode;
    std::vector<NodeId> m_vecRoot; //所有的document的根目录
};








#endif // LIBTREE_H
