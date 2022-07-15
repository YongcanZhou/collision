#ifndef LIBTREE_H
#define LIBTREE_H
#include <algorithm>
#include <vector>
#include <QString>
#include "src/visualization/occ.h"


using NodeId = uint32_t;


struct Assemly_Data{
    QString name;
    bool isAssembly;
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
    Assemly_Tree();
    Assemly_Node* AddNode(Assemly_Node* ParentNode,Assemly_Data& data)
    {
        return  nullptr;
    }

    Assemly_Node* AddNode(NodeId parentId,Assemly_Data& data)
    {
        Assemly_Node* node = this->appendChild(parentId);
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
    std::vector<NodeId> m_vecRoot;
};


static Assemly_Data GetData(const Handle(XCAFDoc_ShapeTool)& ShapeTool,
                     const Handle(XCAFDoc_ColorTool)& ColorTool,
                     const TDF_Label & Label,
                     TopLoc_Location Location)
{
    Assemly_Data data;

    if(!ShapeTool->IsAssembly(Label)){
        data.isAssembly=false;
        //data.shape=ShapeTool->GetShape(Label);
        data.name=ShapeTool->get_type_name();
    }
    else {
        data.isAssembly=true;
        //data.shape=ShapeTool->GetShape(Label);
        data.name=ShapeTool->get_type_name();
    }

    return  data;
}

static void MakeTree(const Handle(XCAFDoc_ShapeTool)& ShapeTool,
              const Handle(XCAFDoc_ColorTool)& ColorTool,
              const TDF_Label & Label,
              TopLoc_Location Location,
              Assemly_Node * ParentNode,
              Assemly_Tree & Tree)
{
    TDF_LabelSequence components;
    if (ShapeTool->GetComponents(Label, components))
    {
        for (Standard_Integer compIndex = 1; compIndex <= components.Length(); ++compIndex)
        {
            TDF_Label ChildLabel = components.Value(compIndex);
            if (ShapeTool->IsReference(ChildLabel))
            {
                TDF_Label ShapeLabel;
                if (ShapeTool->GetReferredShape(ChildLabel, ShapeLabel))
                {
                    TopLoc_Location LocalLocation = Location * ShapeTool->GetLocation(ChildLabel);
                    Assemly_Data AssemlyData = GetData(ShapeTool, ColorTool, ShapeLabel, LocalLocation);
                    Assemly_Node * Node = Tree.AddNode(ParentNode, AssemlyData);
                    MakeTree(ShapeTool, ColorTool, ShapeLabel, LocalLocation, Node, Tree);
                }
            }
        }
    }
}





#endif // LIBTREE_H
