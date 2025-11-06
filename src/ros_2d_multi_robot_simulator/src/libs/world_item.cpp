#include "world_item.h"
#include <iostream>

using namespace std;
using Eigen::Rotation2Df;

WorldItem::~WorldItem() {
  if (parent)
    parent->children.erase(this);
}

WorldItem::WorldItem(const GridMap* g, WorldItem* p, const Isometry2f& iso) :
  grid_map(g),
  parent(p),
  pose_in_parent(iso){
  if (!p)
    return;
  p->children.insert(this);
}

bool WorldItem::isAncestor(const WorldItem& other) const {
  const WorldItem* a=this;
  while (a) {
    if (a==&other)
      return true;
    a=a->parent;
  }
  return false;
}

Isometry2f WorldItem::globalPose() const {
  if (!parent) return pose_in_parent;
  return parent->globalPose() * pose_in_parent;
}

const GridMap& WorldItem::gridMap() const {
  if (grid_map) return *grid_map;

  WorldItem* p = parent;
  if (p) {
    while (p->parent) {
      if (p->grid_map) return (*p->grid_map);
      p = p->parent;
    }
  }

  throw std::runtime_error("No GridMap available in this branch");
}

bool WorldItem::checkCollision(const WorldItem& other) const {
  if (isAncestor(other))
    return false;
  if (other.isAncestor(*this))
    return false;
  // calculate the distance between me and other
  Isometry2f my_pose=globalPose();
  Isometry2f other_pose=other.globalPose();
  Vector2f delta=my_pose.translation()-other_pose.translation();
  float distance=delta.norm();
  if (distance<(radius+other.radius))
    return true;
  for(auto child: children)
    if (child->checkCollision(other))
      return true;

  return false;
  
}

void WorldItem::tick(float dt){
  for (auto child: children)
    child->tick(dt);
}

World::World(const GridMap& gmap):
  WorldItem(gmap){}


UnicyclePlatform::UnicyclePlatform(World& w, const Isometry2f& iso):
  WorldItem(w, iso){}

void UnicyclePlatform::tick(float dt) {
  WorldItem::tick(dt);
  Isometry2f motion=Isometry2f::Identity();
  motion.translation() << tv*dt, 0;
  motion.linear()=Rotation2Df(rv*dt).matrix();
  if (! move(motion)){
    tv=0;
    rv=0;
  }
}


