// $Id$

/*unitorus.cpp
 *
 * Unidirectional Torus with dimension-ordered routing
 * Each link direction has configurable bandwidth and penalties
 *
 */

#include "booksim.hpp"
#include <vector>
#include <sstream>
#include <ctime>
#include <cassert>
#include <iostream>
#include <cstdlib>
#include <string>
#include "unitorus.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "routefunc.hpp"

UniTorus::UniTorus( const Configuration &config, const string & name ) :
Network( config, name )
{
  _debug = config.GetInt("unitorus_debug");
  _ComputeSize( config );
  _ParseDirectionConfig( config );
  _Alloc( );
  
  // Verify allocation worked
  if (_debug) {
    cout << "Verifying channel allocation:" << endl;
    for (int c = 0; c < _channels; ++c) {
      if (_chan[c] == nullptr || _chan_cred[c] == nullptr) {
        cerr << "ERROR: Channel " << c << " not allocated properly" << endl;
        exit(-1);
      }
    }
    cout << "All " << _channels << " channels allocated successfully" << endl;
  }
  
  _BuildNet( config );
}

void UniTorus::_ComputeSize( const Configuration &config )
{
  // Parse dimension sizes from comma-separated string
  string dim_sizes_str = config.GetStr("dim_sizes");
  
  if (dim_sizes_str.empty() || dim_sizes_str == "0") {
    cerr << "Error: dim_sizes must be specified as comma-separated values (e.g., dim_sizes = 4,6,8)" << endl;
    exit(-1);
  }
  
  // Parse comma-separated dimension sizes
  _dim_sizes.clear();
  
  // Remove braces if present (config parser format: {val1,val2,val3})
  string clean_str = dim_sizes_str;
  if (!clean_str.empty() && clean_str.front() == '{') {
    clean_str = clean_str.substr(1);
  }
  if (!clean_str.empty() && clean_str.back() == '}') {
    clean_str = clean_str.substr(0, clean_str.length() - 1);
  }
  
  vector<string> tokens;
  size_t start = 0, end = 0;
  while ((end = clean_str.find(',', start)) != string::npos) {
    tokens.push_back(clean_str.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(clean_str.substr(start));
  
  for (const string& token : tokens) {
    int dim_size = atoi(token.c_str());
    if (dim_size <= 0) {
      cerr << "Error: All dimension sizes must be positive integers. Found: " << token << endl;
      cerr << "Expected format: dim_sizes = {size1,size2,...,sizeN} (e.g., dim_sizes = {4,6,8})" << endl;
      exit(-1);
    }
    _dim_sizes.push_back(dim_size);
  }
  
  // Calculate total network size
  _size = 1;
  for (int i = 0; i < (int)_dim_sizes.size(); ++i) {
    _size *= _dim_sizes[i];
  }
  
  // For global compatibility (some routing functions may still use these)
  gN = _dim_sizes.size();
  gK = _dim_sizes[0]; // Use first dimension as default for legacy compatibility
  gDimSizes = _dim_sizes; // Set global dimension sizes for routing functions
  
  // Unidirectional: only n channels per node (one per dimension)
  _channels = _dim_sizes.size() * _size;

  _nodes = _size;
  
  if (_debug) {
    cout << "UniTorus dimensions: ";
    for (int i = 0; i < (int)_dim_sizes.size(); ++i) {
      cout << _dim_sizes[i];
      if (i < (int)_dim_sizes.size() - 1) cout << "x";
    }
    cout << " = " << _size << " nodes" << endl;
  }
}

void UniTorus::_ParseDirectionConfig( const Configuration &config )
{
  int num_dims = _dim_sizes.size();
  
  // Initialize vectors for each dimension
  _dim_bandwidth.resize(num_dims, 1);  // Default bandwidth = 1
  _dim_latency.resize(num_dims, 1);    // Default latency = 1
  _dim_penalty.resize(num_dims, 0);    // Default penalty = 0

  // Helper function to parse and validate comma-separated values
  auto parseAndValidate = [num_dims](const string& param_str, const string& param_name) -> vector<int> {
    vector<int> values;
    if (param_str.empty() || param_str == "0") {
      return values; // Return empty vector for default handling
    }
    
    // Remove braces if present (config parser format: {val1,val2,val3})
    string clean_str = param_str;
    if (!clean_str.empty() && clean_str.front() == '{') {
      clean_str = clean_str.substr(1);
    }
    if (!clean_str.empty() && clean_str.back() == '}') {
      clean_str = clean_str.substr(0, clean_str.length() - 1);
    }
    
    vector<string> tokens;
    size_t start = 0, end = 0;
    while ((end = clean_str.find(',', start)) != string::npos) {
      string token = clean_str.substr(start, end - start);
      // Trim whitespace
      token.erase(0, token.find_first_not_of(" \t"));
      token.erase(token.find_last_not_of(" \t") + 1);
      if (!token.empty()) {
        tokens.push_back(token);
      }
      start = end + 1;
    }
    string last_token = clean_str.substr(start);
    last_token.erase(0, last_token.find_first_not_of(" \t"));
    last_token.erase(last_token.find_last_not_of(" \t") + 1);
    if (!last_token.empty()) {
      tokens.push_back(last_token);
    }
    
    // Validate count matches number of dimensions
    if ((int)tokens.size() != num_dims) {
      cerr << "Error: " << param_name << " has " << tokens.size() 
           << " values but topology has " << num_dims << " dimensions." << endl;
      cerr << "Expected format: " << param_name << " = {val1,val2,...,val" << num_dims << "}" << endl;
      exit(-1);
    }
    
    // Convert to integers and validate
    for (const string& token : tokens) {
      int val = atoi(token.c_str());
      if (val <= 0) {
        cerr << "Error: All values in " << param_name << " must be positive integers. Found: " << token << endl;
        exit(-1);
      }
      values.push_back(val);
    }
    
    return values;
  };

  // Helper function for penalty parsing (allows zero)
  auto parseAndValidatePenalty = [num_dims](const string& param_str, const string& param_name) -> vector<int> {
    vector<int> values;
    if (param_str.empty() || param_str == "0") {
      return values; // Return empty vector for default handling
    }
    
    // Remove braces if present (config parser format: {val1,val2,val3})
    string clean_str = param_str;
    if (!clean_str.empty() && clean_str.front() == '{') {
      clean_str = clean_str.substr(1);
    }
    if (!clean_str.empty() && clean_str.back() == '}') {
      clean_str = clean_str.substr(0, clean_str.length() - 1);
    }
    
    vector<string> tokens;
    size_t start = 0, end = 0;
    while ((end = clean_str.find(',', start)) != string::npos) {
      string token = clean_str.substr(start, end - start);
      // Trim whitespace
      token.erase(0, token.find_first_not_of(" \t"));
      token.erase(token.find_last_not_of(" \t") + 1);
      if (!token.empty()) {
        tokens.push_back(token);
      }
      start = end + 1;
    }
    string last_token = clean_str.substr(start);
    last_token.erase(0, last_token.find_first_not_of(" \t"));
    last_token.erase(last_token.find_last_not_of(" \t") + 1);
    if (!last_token.empty()) {
      tokens.push_back(last_token);
    }
    
    // Validate count matches number of dimensions
    if ((int)tokens.size() != num_dims) {
      cerr << "Error: " << param_name << " has " << tokens.size() 
           << " values but topology has " << num_dims << " dimensions." << endl;
      cerr << "Expected format: " << param_name << " = {val1,val2,...,val" << num_dims << "}" << endl;
      exit(-1);
    }
    
    // Convert to integers and validate (allow zero for penalties)
    for (const string& token : tokens) {
      int val = atoi(token.c_str());
      if (val < 0) {
        cerr << "Error: All values in " << param_name << " must be non-negative integers. Found: " << token << endl;
        exit(-1);
      }
      values.push_back(val);
    }
    
    return values;
  };

  // Parse and validate bandwidth
  string bandwidth_str = config.GetStr("dim_bandwidth");
  vector<int> bandwidth_values = parseAndValidate(bandwidth_str, "dim_bandwidth");
  if (!bandwidth_values.empty()) {
    _dim_bandwidth = bandwidth_values;
  }

  // Parse and validate latency
  string latency_str = config.GetStr("dim_latency");
  vector<int> latency_values = parseAndValidate(latency_str, "dim_latency");
  if (!latency_values.empty()) {
    _dim_latency = latency_values;
  }

  // Parse and validate penalty (allows zero)
  string penalty_str = config.GetStr("dim_penalty");
  vector<int> penalty_values = parseAndValidatePenalty(penalty_str, "dim_penalty");
  if (!penalty_values.empty()) {
    _dim_penalty = penalty_values;
  }

  // Print configuration
  if (_debug) {
    cout << "UniTorus Direction Configuration:" << endl;
    for (int i = 0; i < num_dims; ++i) {
      cout << "  Dimension " << i << ": size=" << _dim_sizes[i]
           << ", bandwidth=" << _dim_bandwidth[i] 
           << ", latency=" << _dim_latency[i] 
           << ", penalty=" << _dim_penalty[i] << endl;
    }
  }
}

void UniTorus::RegisterRoutingFunctions() {
  gRoutingFunctionMap["dim_order_unitorus_unitorus"] = &dim_order_unitorus;
}

void UniTorus::_BuildNet( const Configuration &config )
{
  ostringstream router_name;

  if (_debug) {
    cout << "Building Unidirectional " << _dim_sizes.size() << "-D Torus" << endl;
    cout << "Dimensions: ";
    for (int i = 0; i < (int)_dim_sizes.size(); ++i) {
      cout << _dim_sizes[i];
      if (i < (int)_dim_sizes.size() - 1) cout << "x";
    }
    cout << " = " << _size << " nodes, " << _channels << " channels" << endl;
  }

  // Create routers
  for ( int node = 0; node < _size; ++node ) {
    if (_debug) cout << "Creating router for node " << node << endl;

    router_name << "router";
    
    // Generate router name based on coordinates
    vector<int> coords = _NodeToCoords(node);
    for (int i = 0; i < (int)_dim_sizes.size(); ++i) {
      router_name << "_" << coords[i];
    }

    if (_debug) {
      cout << "Router name: " << router_name.str() << endl;
      cout << "Node " << node << " inputs=" << (_dim_sizes.size() + 1) << " outputs=" << (_dim_sizes.size() + 1) << endl;
    }

    // Each router has n output ports (one per dimension) + 1 injection + 1 ejection
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
                                        node, _dim_sizes.size() + 1, _dim_sizes.size() + 1 );
    
    if (_routers[node] == nullptr) {
      cerr << "ERROR: Failed to create router for node " << node << endl;
      exit(-1);
    }
    
    _timed_modules.push_back(_routers[node]);

    if (_debug) cout << "Router created successfully" << endl;

    router_name.str("");
  }

  // Connect all the channels after all routers are created
  for ( int node = 0; node < _size; ++node ) {
    // Connect channels for each dimension (unidirectional only)
    for ( int dim = 0; dim < (int)_dim_sizes.size(); ++dim ) {
      int next_node = _NextNode( node, dim );
      int channel = _NextChannel( node, dim );

      if (_debug) {
        cout << "Connecting dim " << dim << ": node " << node << " -> node " << next_node << " via channel " << channel << endl;
      }

      // Validate channel index
      if (channel < 0 || channel >= _channels) {
        cerr << "ERROR: Invalid channel index " << channel << " (max: " << _channels - 1 << ")" << endl;
        exit(-1);
      }

      // Validate that channel objects exist
      if (_chan[channel] == nullptr || _chan_cred[channel] == nullptr) {
        cerr << "ERROR: Channel " << channel << " is null" << endl;
        exit(-1);
      }

      // Validate that routers exist
      if (_routers[node] == nullptr) {
        cerr << "ERROR: Router for node " << node << " is null" << endl;
        exit(-1);
      }
      if (_routers[next_node] == nullptr) {
        cerr << "ERROR: Router for next_node " << next_node << " is null" << endl;
        exit(-1);
      }

      // Add output channel from current node
      if (_debug) cout << "Adding output channel to router " << node << endl;
      _routers[node]->AddOutputChannel( _chan[channel], _chan_cred[channel] );
      
      // Add input channel to next node
      if (_debug) cout << "Adding input channel to router " << next_node << endl;
      _routers[next_node]->AddInputChannel( _chan[channel], _chan_cred[channel] );

      // Set dimension-specific latency
      _chan[channel]->SetLatency( _dim_latency[dim] );
      _chan_cred[channel]->SetLatency( _dim_latency[dim] );
      
      if (_debug) {
        cout << "Channel " << channel << ": node " << node 
             << " -> node " << next_node << " (dim " << dim 
             << ", latency " << _dim_latency[dim] << ")" << endl;
      }
    }
  }

  // Add injection and ejection channels for all routers
  for ( int node = 0; node < _size; ++node ) {
    // Add injection and ejection channels
    _routers[node]->AddInputChannel( _inject[node], _inject_cred[node] );
    _routers[node]->AddOutputChannel( _eject[node], _eject_cred[node] );
    _inject[node]->SetLatency( 1 );
    _inject_cred[node]->SetLatency( 1 );
    _eject[node]->SetLatency( 1 );
    _eject_cred[node]->SetLatency( 1 );
  }
}

int UniTorus::_NextChannel( int node, int dim )
{
  // Channel numbering: node * num_dimensions + dim
  return node * _dim_sizes.size() + dim;
}

int UniTorus::_NextNode( int node, int dim )
{
  vector<int> coords = _NodeToCoords(node);
  
  // Move to next coordinate in this dimension (with wraparound)
  coords[dim] = (coords[dim] + 1) % _dim_sizes[dim];
  
  return _CoordsToNode(coords);
}

vector<int> UniTorus::_NodeToCoords( int node ) const
{
  vector<int> coords(_dim_sizes.size());
  int temp = node;
  
  for (int dim = 0; dim < (int)_dim_sizes.size(); ++dim) {
    coords[dim] = temp % _dim_sizes[dim];
    temp /= _dim_sizes[dim];
  }
  
  return coords;
}

int UniTorus::_CoordsToNode( const vector<int>& coords ) const
{
  int node = 0;
  int multiplier = 1;
  
  for (int dim = 0; dim < (int)_dim_sizes.size(); ++dim) {
    node += coords[dim] * multiplier;
    multiplier *= _dim_sizes[dim];
  }
  
  return node;
}

int UniTorus::GetN( ) const
{
  return _dim_sizes.size();
}

int UniTorus::GetDimSize( int dim ) const
{
  return _dim_sizes[dim];
}

const vector<int>& UniTorus::GetDimSizes( ) const
{
  return _dim_sizes;
}

int UniTorus::GetDimLatency( int dim ) const
{
  return _dim_latency[dim];
}

int UniTorus::GetDimPenalty( int dim ) const
{
  return _dim_penalty[dim];
}

double UniTorus::Capacity( ) const
{
  // Calculate total capacity considering per-dimension bandwidths
  double total_capacity = 0.0;
  for ( int dim = 0; dim < (int)_dim_sizes.size(); ++dim ) {
    total_capacity += (double)(_size * _dim_bandwidth[dim]) / (double)_size;
  }
  return total_capacity;
}

void UniTorus::InsertRandomFaults( const Configuration &config )
{
  // TODO: Implement random fault insertion if needed
}
