// $Id$

#ifndef _CAKE_HPP_
#define _CAKE_HPP_

#include "network.hpp"
#include <vector>
#include <utility>
#include <map>

// Cake: stacked 2D unidirectional tori (X/Y) with sparse vertical links (Z) at configured elevator coordinates
class Cake : public Network {

  // Sizes
  int _x, _y, _layers;

  // Elevator coordinates (x,y) that have vertical links
  std::vector<std::pair<int,int>> _elevators;
  std::map<long long, int> _elevator_index; // key = (x<<32)|y -> index into _elevators

  // For each (x,y), which elevator (x,y) to use when changing layers
  // Dimensions: _y rows, each with _x entries storing pair<x,y>
  std::vector<std::vector<std::pair<int,int>>> _elevator_map;

  bool _debug;

  void _ComputeSize( const Configuration &config );
  void _ParseElevators( const Configuration &config );
  void _BuildNet( const Configuration &config );

  // Helpers
  inline long long _Key(int x, int y) const { return (static_cast<long long>(x) << 32) | static_cast<unsigned int>(y); }
  int _NodeId(int x, int y, int z) const;
  void _IdToXYZ(int id, int &x, int &y, int &z) const;

  // Channel indexing
  int _InplaneChannel(int node, int dim) const;           // dim: 0=X+, 1=Y+
  int _UpChannel(int elev_idx, int layer) const;          // Z+
  int _DownChannel(int elev_idx, int layer) const;        // Z-

  // Counts
  int _inplane_channels;  // _size * 2
  int _vertical_channels; // _elevators.size() * _layers * 2 (up+down)

public:
  Cake( const Configuration &config, const string & name );
  static void RegisterRoutingFunctions();

  inline int X() const { return _x; }
  inline int Y() const { return _y; }
  inline int Layers() const { return _layers; }
  const std::vector<std::pair<int,int>>& Elevators() const { return _elevators; }
  const std::vector<std::vector<std::pair<int,int>>>& ElevatorMap() const { return _elevator_map; }
};

#endif
