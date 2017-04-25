#include "SubmapStore.h"

void print(ostream& out, submap::Pose6d& p)
{
     out<<"pose6d: "<<p.tx<<" "<<p.ty<<" "<<p.tz<<" "<<p.rx<<" "<<p.ry<<" "<<p.rz<<" "<<p.rw<<endl;
}
void print(ostream& out, submap::SubmapHeader& h)
{
    out<<"id: "<<h.id<<" time: "<<h.timestamp<<endl;
    out<<"N_of_nodes: "<<h.n_nodes<<endl;
    out<<"N_of_feature: "<<h.n_features<<endl;
    out<<"root pose: "<<endl;
    print(out, h.root_pose);
}

ostream& operator<<( ostream& out, const submap::Pose6d& p)
{
    out<<p.tx<<" "<<p.ty<<" "<<p.tz<<" "<<p.rx<<" "<<p.ry<<" "<<p.rz<<" "<<p.rw<<" ";
    return out;
}
ostream& operator<<( ostream& out, const submap::SubmapHeader& h)
{
    out<<h.id<<" "<<h.timestamp<<" "<<h.n_nodes<<" "<<h.n_features<<" ";
    out<<h.root_pose;
    return out;
}
istream& operator>>(istream& in, submap::Pose6d& p)
{
    in>>p.tx>>p.ty>>p.tz>>p.rx>>p.ry>>p.rz>>p.rw;
    return in;
}
istream& operator>>(istream& in, submap::SubmapHeader& h)
{
    in>>h.id>>h.timestamp>>h.n_nodes>>h.n_features;
    in>>h.root_pose;
    return in;
}
