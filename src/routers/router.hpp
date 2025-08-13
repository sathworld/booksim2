// $Id$

/*
 Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _ROUTER_HPP_
#define _ROUTER_HPP_

#include <string>
#include <vector>

#include "timed_module.hpp"
#include "flit.hpp"
#include "credit.hpp"
#include "flitchannel.hpp"
#include "channel.hpp"
#include "config_utils.hpp"

typedef Channel<Credit> CreditChannel;

class Router : public TimedModule {

protected:

  static int const STALL_BUFFER_BUSY;
  static int const STALL_BUFFER_CONFLICT;
  static int const STALL_BUFFER_FULL;
  static int const STALL_BUFFER_RESERVED;
  static int const STALL_CROSSBAR_CONFLICT;

  int _id;
  
  int _inputs;
  int _outputs;
  
  int _classes;

  int _input_speedup;
  int _output_speedup;
  
  double _internal_speedup;
  double _partial_internal_cycles;

  int _crossbar_delay;
  int _credit_delay;
  
  vector<FlitChannel *>   _input_channels;
  vector<CreditChannel *> _input_credits;
  vector<FlitChannel *>   _output_channels;
  vector<CreditChannel *> _output_credits;
  vector<bool>            _channel_faults;

  // Optional topology-specific metadata (used by Cake routing, not global)
  int _cake_x_size = 0, _cake_y_size = 0, _cake_layers = 0;
  int _cake_x = -1, _cake_y = -1, _cake_z = -1; // coordinates
  int _cake_elvx = -1, _cake_elvy = -1; // preferred elevator target for this (x,y)
  int _cake_port_xp = -1, _cake_port_yp = -1, _cake_port_zup = -1, _cake_port_zdn = -1;
  int _cake_port_eject = -1;

#ifdef TRACK_FLOWS
  vector<vector<int> > _received_flits;
  vector<vector<int> > _stored_flits;
  vector<vector<int> > _sent_flits;
  vector<vector<int> > _outstanding_credits;
  vector<vector<int> > _active_packets;
#endif

#ifdef TRACK_STALLS
  vector<int> _buffer_busy_stalls;
  vector<int> _buffer_conflict_stalls;
  vector<int> _buffer_full_stalls;
  vector<int> _buffer_reserved_stalls;
  vector<int> _crossbar_conflict_stalls;
#endif

  virtual void _InternalStep() = 0;

public:
  Router( const Configuration& config,
	  Module *parent, const string & name, int id,
	  int inputs, int outputs );

  static Router *NewRouter( const Configuration& config,
			    Module *parent, const string & name, int id,
			    int inputs, int outputs );

  virtual void AddInputChannel( FlitChannel *channel, CreditChannel *backchannel );
  virtual void AddOutputChannel( FlitChannel *channel, CreditChannel *backchannel );
  // Current number of output ports (after dynamic additions)
  int OutputIndexCount() const { return (int)_output_channels.size(); }
 
  inline FlitChannel * GetInputChannel( int input ) const {
    assert((input >= 0) && (input < _inputs));
    return _input_channels[input];
  }
  inline FlitChannel * GetOutputChannel( int output ) const {
    assert((output >= 0) && (output < _outputs));
    return _output_channels[output];
  }

  // Cake metadata setters/getters
  inline void SetCakeSizes(int xs, int ys, int ls){ _cake_x_size=xs; _cake_y_size=ys; _cake_layers=ls; }
  inline void SetCakeXYZ(int x, int y, int z){ _cake_x=x; _cake_y=y; _cake_z=z; }
  inline void SetCakeElevatorTarget(int ex, int ey){ _cake_elvx=ex; _cake_elvy=ey; }
  inline void SetCakePortXPlus(int p){ _cake_port_xp=p; }
  inline void SetCakePortYPlus(int p){ _cake_port_yp=p; }
  inline void SetCakePortZUp(int p){ _cake_port_zup=p; }
  inline void SetCakePortZDown(int p){ _cake_port_zdn=p; }
  inline void SetCakePortEject(int p){ _cake_port_eject=p; }
  inline int CakeXSize() const { return _cake_x_size; }
  inline int CakeYSize() const { return _cake_y_size; }
  inline int CakeLayers() const { return _cake_layers; }
  inline int CakeX() const { return _cake_x; }
  inline int CakeY() const { return _cake_y; }
  inline int CakeZ() const { return _cake_z; }
  inline int CakeElevX() const { return _cake_elvx; }
  inline int CakeElevY() const { return _cake_elvy; }
  inline int CakePortXPlus() const { return _cake_port_xp; }
  inline int CakePortYPlus() const { return _cake_port_yp; }
  inline int CakePortZUp() const { return _cake_port_zup; }
  inline int CakePortZDown() const { return _cake_port_zdn; }
  inline int CakePortEject() const { return _cake_port_eject; }

  virtual void ReadInputs( ) = 0;
  virtual void Evaluate( );
  virtual void WriteOutputs( ) = 0;

  void OutChannelFault( int c, bool fault = true );
  bool IsFaultyOutput( int c ) const;

  inline int GetID( ) const {return _id;}

  // Cake metadata API
  void SetCakeSizes(int xs, int ys, int ls); // defined inline below
  void SetCakeXYZ(int x, int y, int z);
  void SetCakeElevatorTarget(int ex, int ey);
  void SetCakePortXPlus(int p);
  void SetCakePortYPlus(int p);
  void SetCakePortZUp(int p);
  void SetCakePortZDown(int p);
  void SetCakePortEject(int p);
  int CakeXSize() const; int CakeYSize() const; int CakeLayers() const;
  int CakeX() const; int CakeY() const; int CakeZ() const;
  int CakeElevX() const; int CakeElevY() const;
  int CakePortXPlus() const; int CakePortYPlus() const; int CakePortZUp() const; int CakePortZDown() const; int CakePortEject() const;
  int OutputIndexCount() const;


  virtual int GetUsedCredit(int o) const = 0;
  virtual int GetBufferOccupancy(int i) const = 0;

#ifdef TRACK_BUFFERS
  virtual int GetUsedCreditForClass(int output, int cl) const = 0;
  virtual int GetBufferOccupancyForClass(int input, int cl) const = 0;
#endif

#ifdef TRACK_FLOWS
  inline vector<int> const & GetReceivedFlits(int c) const {
    assert((c >= 0) && (c < _classes));
    return _received_flits[c];
  }
  inline vector<int> const & GetStoredFlits(int c) const {
    assert((c >= 0) && (c < _classes));
    return _stored_flits[c];
  }
  inline vector<int> const & GetSentFlits(int c) const {
    assert((c >= 0) && (c < _classes));
    return _sent_flits[c];
  }
  inline vector<int> const & GetOutstandingCredits(int c) const {
    assert((c >= 0) && (c < _classes));
    return _outstanding_credits[c];
  }

  inline vector<int> const & GetActivePackets(int c) const {
    assert((c >= 0) && (c < _classes));
    return _active_packets[c];
  }

  inline void ResetFlowStats(int c) {
    assert((c >= 0) && (c < _classes));
    _received_flits[c].assign(_received_flits[c].size(), 0);
    _sent_flits[c].assign(_sent_flits[c].size(), 0);
  }
#endif

  virtual vector<int> UsedCredits() const = 0;
  virtual vector<int> FreeCredits() const = 0;
  virtual vector<int> MaxCredits() const = 0;

#ifdef TRACK_STALLS
  inline int GetBufferBusyStalls(int c) const {
    assert((c >= 0) && (c < _classes));
    return _buffer_busy_stalls[c];
  }
  inline int GetBufferConflictStalls(int c) const {
    assert((c >= 0) && (c < _classes));
    return _buffer_conflict_stalls[c];
  }
  inline int GetBufferFullStalls(int c) const {
    assert((c >= 0) && (c < _classes));
    return _buffer_full_stalls[c];
  }
  inline int GetBufferReservedStalls(int c) const {
    assert((c >= 0) && (c < _classes));
    return _buffer_reserved_stalls[c];
  }
  inline int GetCrossbarConflictStalls(int c) const {
    assert((c >= 0) && (c < _classes));
    return _crossbar_conflict_stalls[c];
  }

  inline void ResetStallStats(int c) {
    assert((c >= 0) && (c < _classes));
    _buffer_busy_stalls[c] = 0;
    _buffer_conflict_stalls[c] = 0;
    _buffer_full_stalls[c] = 0;
    _buffer_reserved_stalls[c] = 0;
    _crossbar_conflict_stalls[c] = 0;
  }
#endif

  inline int NumInputs() const {return _inputs;}
  inline int NumOutputs() const {return _outputs;}
};

// Inline implementations for Cake metadata to avoid additional .cpp dependencies
inline void Router::SetCakeSizes(int xs, int ys, int ls){ _cake_x_size=xs; _cake_y_size=ys; _cake_layers=ls; }
inline void Router::SetCakeXYZ(int x, int y, int z){ _cake_x=x; _cake_y=y; _cake_z=z; }
inline void Router::SetCakeElevatorTarget(int ex, int ey){ _cake_elvx=ex; _cake_elvy=ey; }
inline void Router::SetCakePortXPlus(int p){ _cake_port_xp=p; }
inline void Router::SetCakePortYPlus(int p){ _cake_port_yp=p; }
inline void Router::SetCakePortZUp(int p){ _cake_port_zup=p; }
inline void Router::SetCakePortZDown(int p){ _cake_port_zdn=p; }
inline void Router::SetCakePortEject(int p){ _cake_port_eject=p; }
inline int Router::CakeXSize() const { return _cake_x_size; }
inline int Router::CakeYSize() const { return _cake_y_size; }
inline int Router::CakeLayers() const { return _cake_layers; }
inline int Router::CakeX() const { return _cake_x; }
inline int Router::CakeY() const { return _cake_y; }
inline int Router::CakeZ() const { return _cake_z; }
inline int Router::CakeElevX() const { return _cake_elvx; }
inline int Router::CakeElevY() const { return _cake_elvy; }
inline int Router::CakePortXPlus() const { return _cake_port_xp; }
inline int Router::CakePortYPlus() const { return _cake_port_yp; }
inline int Router::CakePortZUp() const { return _cake_port_zup; }
inline int Router::CakePortZDown() const { return _cake_port_zdn; }
inline int Router::CakePortEject() const { return _cake_port_eject; }
inline int Router::OutputIndexCount() const { return (int)_output_channels.size(); }

#endif
