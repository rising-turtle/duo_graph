#ifndef DUO_GRAPH_H
#define DUO_GRAPH_H

#include "boost/shared_ptr.hpp"
#include "GraphWrapper.h"
#include <QFuture>
#include "ClientNet.h"

class Node;

class CDuoGraph
{
public:
    CDuoGraph();
    ~CDuoGraph();
    bool addNode(Node*);
    bool m_bhas_switched;
    CGraphWrapper* getFrontGraph(){return m_mgr_fr.get();}
    CGraphWrapper* getBackGraph(){return m_mgr_bk.get();}
    boost::shared_ptr<CGraphWrapper> m_mgr_fr;
    boost::shared_ptr<CGraphWrapper> m_mgr_bk;
    QFuture<void> future_;

    ClientNet m_client;
};


#endif
