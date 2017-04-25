#ifndef SUBMAP_GRAPH_H
#define SUBMAP_GRAPH_H

#include "GraphWrapper.h"
class CSubmapNode;

class CSubmapGraph : public CGraphWrapper
{
public:
            CSubmapGraph();
    virtual ~CSubmapGraph();
    bool addNode(CSubmapNode* );
    void saveCloud2File(string );
    void saveCloudByOctomap(string );
    void saveFeature2File(string );
    void saveFeaturePosDes2File(string );
    void saveRefinedSubmaps(string path="");
    void optimizeSubmapGraph();
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



#endif
