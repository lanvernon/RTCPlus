//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_EXTCELL_H_
#define _LTE_EXTCELL_H_

#include <omnetpp.h>
#include "common/LteCommon.h"
#include "corenetwork/binder/LteBinder.h"

typedef std::vector<int> BandStatus;

/* The allocation type defines the interference produced by the ext cell
 * - FULL_ALLOC: the cell allocates all RBs in the frame
 * - RANDOM_ALLOC: the cell allocates X RBs, which are chosen randomly
 * - CONTIGUOUS_ALLOC: the cell allocates X contiguous RBs, starting from a given RB
 */
typedef enum {
    FULL_ALLOC, RANDOM_ALLOC, CONTIGUOUS_ALLOC
} BandAllocationType;

class ExtCell : public cSimpleModule
{
    // playground coordinates
    inet::Coord position_;

    // id among all the external cells
    int id_;

    // tx power
    double txPower_;

    // tx direction
    TxDirectionType txDirection_;

    // tx angle
    double txAngle_;

    // number of logical bands
    int numBands_;

    // reference to the binder
    LteBinder* binder_;

    // Current and previous band occupation status. Used for interference computation
    BandStatus bandStatus_;
    BandStatus prevBandStatus_;

    // TTI self message
    cMessage* ttiTick_;

    /*** ALLOCATION MANAGEMENT ***/

    // Allocation Type for this cell
    BandAllocationType allocationType_;

    // percentage of RBs allocated by the ext cell
    // used for RANDOM_ALLOC and CONTIGUOUS ALLOC allocation types
    double bandUtilization_;

    // index of the first allocated RB for CONTIGUOUS_ALLOC allocation type
    int startingOffset_;

    // update the band status. Called at each TTI (not used for FULL_ALLOC)
    void updateBandStatus();

    // move the current status in the prevBandStatus structure and reset the former
    void resetBandStatus();
    /*****************************/

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

  public:

    const inet::Coord getPosition() { return position_; }

    int getId() { return id_; }

    double getTxPower() { return txPower_; }

    TxDirectionType getTxDirection() { return txDirection_; }

    double getTxAngle() { return txAngle_; }

    void setBlock(int band) { bandStatus_.at(band) = 1; }

    void unsetBlock(int band) { bandStatus_.at(band) = 0; }

    int getBandStatus(int band) { return bandStatus_.at(band); }

    int getPrevBandStatus(int band) { return prevBandStatus_.at(band); }

    // set the band utilization percentage
    void setBandUtilization(double bandUtilization);
};

#endif
