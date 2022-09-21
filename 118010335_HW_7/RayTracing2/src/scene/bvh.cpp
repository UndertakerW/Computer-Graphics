#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;
  for (auto p = start; p != end; p++) 
  {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }
  BVHNode *node = new BVHNode(bbox);
  size_t num_primitives = (size_t) (end - start);
  // Build a leaf node
  if (num_primitives <= max_leaf_size)
  {
    node->start = start;
    node->end = end;
  }
  // Build an internal node
  else
  {
    // Find the best splitting axis -> the longest axis
    size_t best_split = 0;
    for (int i = 1; i < 3; ++i)
    {
      if (bbox.extent[i] > bbox.extent[best_split])
      {
        best_split = i;
      }
    }

    double avg_centroids = (double) 0;
    for (auto p = start; p != end; p++) 
    {
      avg_centroids += (*p)->get_bbox().centroid()[best_split];
    }
    avg_centroids = avg_centroids / num_primitives;
    // Swap the pointers so that
    // [start, i) < split point
    // [i, end) >= split point
    std::vector<Primitive *>::iterator i = start;
    std::vector<Primitive *>::iterator j = end - 1;
    while (i != j)
    {
      // 1 2 3 4 5 600
      while ((*i)->get_bbox().centroid()[best_split] < avg_centroids && i != j)
      {
        i++;
      }
      while ((*j)->get_bbox().centroid()[best_split] >= avg_centroids && i != j)
      {
        j--;
      }
      if (i != j)
      {
        Primitive *temp = *i;
        *i = *j;
        *j = temp;
      }
    }
    node->l = construct_bvh(start, i, max_leaf_size);
    node->r = construct_bvh(i, end, max_leaf_size);
  }

  return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  bool hit = false;
	if (node->isLeaf()) 
  {
		for (auto p = node->start; p != node->end; ++p)
    {
      total_isects++;
			if ((*p)->has_intersection(ray))
      {
				hit = true;
      }
    }
	}
	else 
  {
		double t1, t2, t3, t4;
		bool leftBboxHit = node->l->bb.intersect(ray, t1, t2);
		bool rightBboxHit = node->r->bb.intersect(ray, t3, t4);
    if (leftBboxHit)
    {
      hit = has_intersection(ray, node->l);
    }
    if (!hit && rightBboxHit)
    {
      hit = has_intersection(ray, node->r);
    }
	}
  return hit;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1))
  {
    return false;
  }
  bool hit = false;
	if (node->isLeaf()) 
  {
		for (auto p = node->start; p != node->end; p++)
    {
      total_isects++;
			if ((*p)->intersect(ray, i))
      {
				hit = true;
      }
    }
	}
	else 
  {
		double t1, t2, t3, t4;
		bool leftBboxHit = node->l->bb.intersect(ray, t1, t2);
		bool rightBboxHit = node->r->bb.intersect(ray, t3, t4);
    if (leftBboxHit)
    {
      hit = intersect(ray, i, node->l);
    }
    if (rightBboxHit && t3 < i->t)
    {
      bool rightHit = intersect(ray, i, node->r);
      if (!hit)
      {
        hit = rightHit;
      }
    }
	}
  return hit;
}

} // namespace SceneObjects
} // namespace CGL
