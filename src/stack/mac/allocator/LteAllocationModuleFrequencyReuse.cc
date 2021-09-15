//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "stack/mac/allocator/LteAllocationModuleFrequencyReuse.h"
#include "stack/mac/layer/LteMacEnb.h"
#include "stack/mac/conflict_graph_utilities/meshMaster.h"

LteAllocationModuleFrequencyReuse::LteAllocationModuleFrequencyReuse(LteMacBase* mac,Direction direction)
        : LteAllocationModule(mac,direction)
{
}

void LteAllocationModuleFrequencyReuse::storeAllocation( std::vector<std::vector<AllocatedRbsPerBandMapA> > allocatedRbsPerBand,std::set<Band>* untouchableBands)
{
    Plane plane = MAIN_PLANE;
    const Remote antenna = MACRO;
    std::map<std::pair<MacNodeId,Band>,std::pair<unsigned int,unsigned int> > NodeIdRbsBytesMap;
    NodeIdRbsBytesMap.clear();
    // Create an empty vector
    if(untouchableBands==NULL)
    {
        std::set<Band> tempBand;
        untouchableBands = &tempBand;
        untouchableBands->clear();
    }

    for(unsigned int band=0;band<bands_;band++)
    {
        // Skip allocation if the band is untouchable (this means that the informations are already allocated)
        if( untouchableBands->find(band) == untouchableBands->end() )
        {
            // Copy the ueAllocatedRbsMap
            UeAllocatedBlocksMapA::iterator it_ext = allocatedRbsPerBand[plane][antenna][band].ueAllocatedRbsMap_.begin();
            UeAllocatedBlocksMapA::iterator et_ext = allocatedRbsPerBand[plane][antenna][band].ueAllocatedRbsMap_.end();
            UeAllocatedBytesMapA::iterator it2_ext = allocatedRbsPerBand[plane][antenna][band].ueAllocatedBytesMap_.begin();

            while(it_ext!=et_ext)
            {
                allocatedRbsPerBand_[plane][antenna][band].ueAllocatedRbsMap_[it_ext->first] = it_ext->second;    // Blocks
                allocatedRbsPerBand_[plane][antenna][band].ueAllocatedBytesMap_[it_ext->first] = it2_ext->second; // Bytes

                // Creates a pair (a key) for the Map
                std::pair<MacNodeId,Band> Key_pair (it_ext->first,band);
                // Creates a pair for the blocks and bytes values
                std::pair<unsigned int, unsigned int> Value_pair (it_ext->second,it2_ext->second);
                //Store the nodeId RBs
                NodeIdRbsBytesMap[Key_pair] = Value_pair;
                it_ext++;
                it2_ext++;
            }
            // Copy the allocatedRbsPerBand
            allocatedRbsPerBand_[plane][antenna][band].allocated_ = allocatedRbsPerBand[plane][antenna][band].allocated_;

            if (allocatedRbsPerBand[plane][antenna][band].allocated_ > 0)
                allocatedRbsMatrix_[MAIN_PLANE][MACRO] ++;
        }
    }

    std::map<std::pair<MacNodeId,Band>,std::pair<unsigned int,unsigned int> >::iterator it_rbsB = NodeIdRbsBytesMap.begin();
    while(it_rbsB!=NodeIdRbsBytesMap.end())
    {
        // Skip allocation if the band is untouchable (this means that the informations are already allocated)
        if( untouchableBands->find(it_rbsB->first.second) == untouchableBands->end() )
        {
            allocatedRbsUe_[it_rbsB->first.first].ueAllocatedRbsMap_[antenna][it_rbsB->first.second] = it_rbsB->second.first; //Blocks
            allocatedRbsUe_[it_rbsB->first.first].allocatedBlocks_ += it_rbsB->second.first; //Blocks
            allocatedRbsUe_[it_rbsB->first.first].allocatedBytes_ += it_rbsB->second.second; //Bytes

            // Creates and store the allocation Elem
            AllocationElem elem;
            elem.resourceBlocks_ = it_rbsB->second.first;
            elem.bytes_ = it_rbsB->second.second;

            allocatedRbsUe_[it_rbsB->first.first].allocationMap_[antenna][it_rbsB->first.second].push_back(elem);
        }
        it_rbsB++;
    }
}

std::set<Band>  LteAllocationModuleFrequencyReuse::getAllocatorOccupiedBands()
{
    // TODO add support for logical band different from the number of real bands
    std::set<Band> vectorBand;
    vectorBand.clear();
    for(unsigned int i=0;i<bands_;i++)
    {
        if( allocatedRbsPerBand_[MAIN_PLANE][MACRO][i].allocated_>0 ) vectorBand.insert(i);

    }
    return vectorBand;
}

/**
 * Check if the allocation respects the allocation constraints
 */
void LteAllocationModuleFrequencyReuse::checkAllocation(std::set<Band>* untouchableBands)
{
    Plane plane = MAIN_PLANE;
    const Remote antenna = MACRO;
    LteMacEnb* mac = check_and_cast<LteMacEnb*>(mac_);
    const std::map<MacNodeId,std::set<MacNodeId> >* conflictMap = mac->getMeshMaster()->getConflictMap();
    // Create an empty vector
    if(untouchableBands==NULL)
    {
        std::set<Band> tempBand;
        untouchableBands = &tempBand;
        untouchableBands->clear();
    }
    // Fer every bands in the system
    for(unsigned int band=0;band<bands_;band++)
    {
        // Skip allocation if the band is untouchable (this means that the informations are already allocated)
        if( untouchableBands->find(band) == untouchableBands->end() )
        {
            // Copy the ueAllocatedRbsMap
            UeAllocatedBlocksMapA::iterator ref_it_ext = allocatedRbsPerBand_[plane][antenna][band].ueAllocatedRbsMap_.begin();
            UeAllocatedBlocksMapA::iterator et_ext = allocatedRbsPerBand_[plane][antenna][band].ueAllocatedRbsMap_.end();
            while(ref_it_ext!=et_ext)
            {
                // Set the iterator
                UeAllocatedBlocksMapA::iterator it_ext = allocatedRbsPerBand_[plane][antenna][band].ueAllocatedRbsMap_.begin();

                // If the node is present in the conflict map we have to check if there was an error in the allocation
                if(conflictMap->find(ref_it_ext->first)!=conflictMap->end())
                {
                    // For every nodeId in the band map
                    while(it_ext!=et_ext)
                    {
                        if(conflictMap->at(ref_it_ext->first).find(it_ext->first)!=conflictMap->at(ref_it_ext->first).end())
                            throw cRuntimeError("checkAllocation(): error two conflicting nodes (%d and %d) are sharing the same band: %d",ref_it_ext->first,it_ext->first,band);
                        ++it_ext;
                    }

                }
                ++ref_it_ext;
            }
        }
    }
}

