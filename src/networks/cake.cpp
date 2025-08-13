// $Id$
//
// Cake topology implementation
// --------------------------------------
// Stacked 2D layers of unidirectional tori (X+ and Y+ only) with sparse
// vertical links ("elevators") at specified (x,y) coordinates. Each
// elevator provides two unidirectional links per layer: Z+ (up) and Z- (down),
// both wrap around in the layer index. Nodes inject/eject locally.
//
// Key configuration keys (see README/config):
//  - topology = cake
//  - routing_function = dor_cake (registered elsewhere)
//  - dim_sizes = {x,y[,layers]} or x=<int>, y=<int>, layers=<int>
//  - elevator_coords = "{{ex,ey},{...}}" list of (x,y) coordinates that have
//    vertical links
//  - elevator_mapping_coords = 2D matrix of size Y rows x X columns encoding
//    per-node preferred elevator as pairs (ex,ey); defaults to identity
//
// Routing relies on per-router metadata (sizes, coordinates, preferred
// elevator, and recorded output port indices for X+/Y+/Z+/Z-/eject) set here
// during construction, avoiding any cake-specific global state.

#include "booksim.hpp"
#include <vector>
#include <sstream>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <string>

#include "cake.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "routefunc.hpp"

using std::pair;
using std::make_pair;

Cake::Cake( const Configuration &config, const string & name ) : Network(config, name) {
  _debug = config.GetInt("unitorus_debug"); // reuse debug flag
  // Size first, then elevators (affects channel count), then allocate and wire.
  _ComputeSize(config);
  _ParseElevators(config);
  _Alloc();
  _BuildNet(config);
}

void Cake::_ComputeSize( const Configuration &config ) {
  // Parse sizes via dim_sizes for consistency with UniTorus, e.g., "{X,Y[,Z]}".
  // Z represents number of layers; if omitted, defaults to 1.
  string dim_sizes_str = config.GetStr("dim_sizes");
  if(!dim_sizes_str.empty()) {
    // parse {x,y} or {x,y,l}
    string s = dim_sizes_str;
    if(!s.empty() && s.front()=='{') s = s.substr(1);
    if(!s.empty() && s.back()=='}') s.pop_back();
    std::vector<string> toks;
    // Split on commas
    size_t p=0; while(true){ size_t q=s.find(',',p); string t=(q==string::npos)?s.substr(p):s.substr(p,q-p); 
      // trim
      size_t a=t.find_first_not_of(" \t"); size_t b=t.find_last_not_of(" \t");
      // remove surrounding whitespace
      if(a!=string::npos) t=t.substr(a,b-a+1); else t="";
      // skip empty tokens
      if(!t.empty()) toks.push_back(t); if(q==string::npos) break; p=q+1; }
    if(toks.size() < 2) { cerr<<"dim_sizes must have at least 2 values (x,y)"<<endl; exit(-1);} 
  _x = atoi(toks[0].c_str());
  _y = atoi(toks[1].c_str());
  // layers (Z) default to 1 if not specified
  _layers = (toks.size()>=3)? atoi(toks[2].c_str()): 1;
  } else {
  // For Cake, dim_sizes is required (align with UniTorus usage)
  cerr << "Cake requires 'dim_sizes' to specify {x,y[,layers]}" << endl;
  exit(-1);
  }
  if(_x<=0 || _y<=0 || _layers<=0) { cerr<<"Invalid sizes for Cake: x="<<_x<<" y="<<_y<<" layers="<<_layers<<endl; exit(-1);} 

  // Booksim global dimension hints used by some routing helpers
  gK = _x; gN = 2; // base DOR uses 2 in-plane dims; Z handled as special
  _size = _x * _y * _layers;
  _nodes = _size;

  _inplane_channels = _size * 2; // per node: X+ and Y+ output channels
  // vertical channels depend on elevator count; computed after parsing
  _vertical_channels = 0; 
  _channels = -1; // finalized after elevator parsing
}

void Cake::_ParseElevators( const Configuration &config ) {
  // elevator_coords: e.g., "{{0,1},{2,2}}" (preferred key)
  // Also accept legacy key "elevatorcoords" for compatibility.
  string elev_str = config.GetStr("elevator_coords");
  if(elev_str.empty()) elev_str = config.GetStr("elevatorcoords");
  // Clear previous elevators
  _elevators.clear();
  _elevator_index.clear();
  if(elev_str.empty()) {
    // default: none
  } else {
    // Strip outer braces and parse as a list of "x,y" tokens.
    string s = elev_str;
    // Replace braces with spaces to ease parsing
    for(char &c: s){ if(c=='{'||c=='}') c=' '; }
    // Now tokens are of form x,y pairs; parse by scanning numbers
    // We'll split on spaces, then inside token split by comma
    std::vector<string> toks;
    std::stringstream ss(s);
    string tok;
    while(ss >> tok){ if(!tok.empty()) toks.push_back(tok); }
    // Tokens may be like "0,1"; gather pairs
    for(string &t : toks){
      size_t comma = t.find(',');
      if(comma==string::npos) continue; // ignore
      int ex = atoi(t.substr(0, comma).c_str());
      int ey = atoi(t.substr(comma+1).c_str());
      if(ex<0||ex>=_x||ey<0||ey>=_y){ cerr<<"elevator coord out of range: ("<<ex<<","<<ey<<")"<<endl; exit(-1);} 
      long long key = _Key(ex,ey);
      if(_elevator_index.find(key)==_elevator_index.end()){
        int idx = (int)_elevators.size();
        _elevators.push_back(make_pair(ex,ey));
        _elevator_index[key] = idx;
      }
    }
  }

  // elevator_mapping_coords: 2D array of size Y rows, each row has X pairs
  // Example:
  // {
  //  {{0,1},{0,1},{2,2}},
  //  {{0,1},{0,1},{2,2}},
  //  {{2,2},{2,2},{2,2}},
  // }
  // elevator_mapping_coords: preferred key; also accept legacy "elevatormapping".
  string map_str = config.GetStr("elevator_mapping_coords");
  if(map_str.empty()) map_str = config.GetStr("elevatormapping");
  _elevator_map.assign(_y, std::vector<pair<int,int>>(_x, make_pair(0,0)));
  if(!map_str.empty()){
    // Convert to a linear stream of numbers extracting pairs in row-major order
    // We accept various bracket styles and commas; anything not numeric acts as a delimiter.
    string s = map_str;
    for(char &c: s){ if(c=='{'||c=='}' || c=='[' || c==']') c=' '; }
    // Expect 2*_x*_y integers
    std::vector<int> nums; nums.reserve(_x*_y*2);
    std::stringstream ss(s);
    string tok;
    while(ss >> tok){
      if(tok == ",") continue; // if commas separated as tokens
      // tokens may include commas, strip trailing commas
      if(!tok.empty() && tok.back()==',') tok.pop_back();
      if(tok.empty()) continue;
      // skip commas
      if(tok==",") continue;
      // split by comma inside token
      size_t start = 0; while(true){
        size_t comma = tok.find(',', start);
        string part = (comma==string::npos)? tok.substr(start): tok.substr(start, comma-start);
        if(!part.empty()) nums.push_back(atoi(part.c_str()));
        if(comma==string::npos) break; start = comma+1;
      }
    }
    if((int)nums.size() != _x*_y*2){
      cerr<<"elevator_mapping_coords expects "<<(_x*_y*2)<<" integers (x,y pairs), got "<<nums.size()<<endl; exit(-1);
    }
    int idx=0; for(int ry=0; ry<_y; ++ry){ for(int rx=0; rx<_x; ++rx){ int ex = nums[idx++]; int ey = nums[idx++];
      if(ex<0||ex>=_x||ey<0||ey>=_y){ cerr<<"elevator_mapping_coords out of range at ("<<rx<<","<<ry<<") -> ("<<ex<<","<<ey<<")"<<endl; exit(-1);} 
      _elevator_map[ry][rx] = make_pair(ex,ey);
    }}
  } else {
    // default: identity (use own column/row)
    for(int ry=0; ry<_y; ++ry) for(int rx=0; rx<_x; ++rx) _elevator_map[ry][rx] = make_pair(rx,ry);
  }

  // Count vertical channels: for each elevator location there are up and down per layer
  //
  int E = (int)_elevators.size();
  _vertical_channels = E * _layers * 2;
  _channels = _inplane_channels + _vertical_channels;
  // No cake-specific globals; routers will carry metadata used by routing function.
}

int Cake::_NodeId(int x, int y, int z) const {
  // Flatten 3D coordinates to node id (row-major within each layer, then layers)
  return z * (_x * _y) + y * _x + x;
}

void Cake::_IdToXYZ(int id, int &x, int &y, int &z) const {
  // Inverse of _NodeId
  int plane = _x * _y;
  z = id / plane;
  int rem = id % plane;
  y = rem / _x;
  x = rem % _x;
}

int Cake::_InplaneChannel(int node, int dim) const {
  // dim 0=X+, 1=Y+
  return node * 2 + dim;
}

int Cake::_UpChannel(int elev_idx, int layer) const { // from layer -> layer+1 at same x,y
  // Channels are laid out as [in-plane] first, then vertical per (elevator,layer): {up,down}
  return _inplane_channels + (elev_idx * _layers + layer) * 2 + 0;
}
int Cake::_DownChannel(int elev_idx, int layer) const { // from layer -> layer-1
  return _inplane_channels + (elev_idx * _layers + layer) * 2 + 1;
}

void Cake::_BuildNet( const Configuration &config )
{
  ostringstream router_name;

  // Create routers with exact port counts and set per-router metadata
  for(int id=0; id<_size; ++id){
    int x,y,z; _IdToXYZ(id,x,y,z);
    router_name << "router_" << x << "_" << y << "_" << z;
    bool is_elev = (_elevator_index.find(_Key(x,y)) != _elevator_index.end());
    // Each router has: X+ and Y+ always, plus Z+/Z- only if this (x,y) hosts an elevator.
    int net_ports = 2 + (is_elev ? 2 : 0);
    int in_cnt = net_ports + 1;  // + injection
    int out_cnt = net_ports + 1; // + ejection
    _routers[id] = Router::NewRouter(config, this, router_name.str(), id, in_cnt, out_cnt);
    _timed_modules.push_back(_routers[id]);
    // set sizes, coordinates, and preferred elevator for this (x,y)
    _routers[id]->SetCakeSizes(_x, _y, _layers);
    _routers[id]->SetCakeXYZ(x, y, z);
    pair<int,int> pref = _elevator_map[y][x];
    _routers[id]->SetCakeElevatorTarget(pref.first, pref.second);
    // Reset router_name for next iteration
    router_name.str("");
  }

  // Connect in-plane X+ links (unidirectional wrap)
  for(int z=0; z<_layers; ++z){
    for(int y=0; y<_y; ++y){
      for(int x=0; x<_x; ++x){
        int from = _NodeId(x,y,z);
        int to = _NodeId((x+1)%_x, y, z);
        int ch = _InplaneChannel(from, 0);
        // Record the output index before adding the channel; after AddOutputChannel,
        // the recorded index will refer to this X+ output for routing time selection.
        int out_idx = _routers[from]->OutputIndexCount();
        _routers[from]->AddOutputChannel(_chan[ch], _chan_cred[ch]);
        _routers[from]->SetCakePortXPlus(out_idx);
        _routers[to]->AddInputChannel(_chan[ch], _chan_cred[ch]);
        // latency 1 in-plane
        _chan[ch]->SetLatency(1); _chan_cred[ch]->SetLatency(1);
      }
    }
  }

  // Connect in-plane Y+ links (unidirectional wrap)
  for(int z=0; z<_layers; ++z){
    for(int y=0; y<_y; ++y){
      for(int x=0; x<_x; ++x){
        int from = _NodeId(x,y,z);
        int to = _NodeId(x, (y+1)%_y, z);
        int ch = _InplaneChannel(from, 1);
        // Record and store Y+ output index
        int out_idx = _routers[from]->OutputIndexCount();
        _routers[from]->AddOutputChannel(_chan[ch], _chan_cred[ch]);
        _routers[from]->SetCakePortYPlus(out_idx);
        _routers[to]->AddInputChannel(_chan[ch], _chan_cred[ch]);
        _chan[ch]->SetLatency(1); _chan_cred[ch]->SetLatency(1);
      }
    }
  }

  // Connect vertical links at elevator locations
  for(size_t ei=0; ei<_elevators.size(); ++ei){
    int ex = _elevators[ei].first;
    int ey = _elevators[ei].second;
    for(int z=0; z<_layers; ++z){
      int from = _NodeId(ex,ey,z);
      // Up link (z -> z+1 modulo layers) unidirectional
      int to_up = _NodeId(ex,ey,(z+1)%_layers);
      int ch_u = _UpChannel((int)ei, z);
      // Record and store Z+ output index if present at this (x,y)
      int out_idx_u = _routers[from]->OutputIndexCount();
      _routers[from]->AddOutputChannel(_chan[ch_u], _chan_cred[ch_u]);
      _routers[from]->SetCakePortZUp(out_idx_u);
      _routers[to_up]->AddInputChannel(_chan[ch_u], _chan_cred[ch_u]);
      _chan[ch_u]->SetLatency(1); _chan_cred[ch_u]->SetLatency(1);
      // Down link (z -> z-1 modulo layers) unidirectional
      int to_dn = _NodeId(ex,ey,(z-1+_layers)%_layers);
      int ch_d = _DownChannel((int)ei, z);
      // Record and store Z- output index
      int out_idx_d = _routers[from]->OutputIndexCount();
      _routers[from]->AddOutputChannel(_chan[ch_d], _chan_cred[ch_d]);
      _routers[from]->SetCakePortZDown(out_idx_d);
      _routers[to_dn]->AddInputChannel(_chan[ch_d], _chan_cred[ch_d]);
      _chan[ch_d]->SetLatency(1); _chan_cred[ch_d]->SetLatency(1);
    }
  }

  // Add injection/ejection
  for(int id=0; id<_size; ++id){
    // Injection is an input; ejection is an output. Record the output index for routing.
    _routers[id]->AddInputChannel(_inject[id], _inject_cred[id]);
    int out_idx_ej = _routers[id]->OutputIndexCount();
    _routers[id]->AddOutputChannel(_eject[id], _eject_cred[id]);
    _routers[id]->SetCakePortEject(out_idx_ej);
    _inject[id]->SetLatency(1); _inject_cred[id]->SetLatency(1);
    _eject[id]->SetLatency(1); _eject_cred[id]->SetLatency(1);
  }
}

void Cake::RegisterRoutingFunctions() {
  // Register name used as routing_function for cake
  // TODO

}
