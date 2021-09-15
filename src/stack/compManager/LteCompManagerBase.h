//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef LTE_LTECOMPMANAGERBASE_H_
#define LTE_LTECOMPMANAGERBASE_H_

#include "common/LteCommon.h"
#include "stack/mac/layer/LteMacEnb.h"
#include "stack/mac/buffer/LteMacBuffer.h"
#include "x2/packet/X2ControlInfo_m.h"
#include "stack/compManager/X2CompMsg.h"
#include "stack/compManager/X2CompRequestIE.h"
#include "stack/compManager/X2CompReplyIE.h"

typedef enum {
    COMP_CLIENT,
    COMP_CLIENT_COORDINATOR,
    COMP_COORDINATOR
} CompNodeType;

//
// LteCompManagerBase
// Base class for CoMP manager modules.
// To add a new CoMP algorithm, extend this class and redefine pure virtual methods
//
class LteCompManagerBase : public cSimpleModule
{

protected:

    // X2 identifier
    X2NodeId nodeId_;

    // reference to the gates
    cGate* x2Manager_[2];

    // reference to the MAC layer
    LteMacEnb* mac_;

    // number of available bands
    int numBands_;

    // period between two coordination instances
    double coordinationPeriod_;

    /// Self messages
    cMessage* compClientTick_;
    cMessage* compCoordinatorTick_;

    // Comp Node Type specification (client, client and coordinator, coordinator only)
    CompNodeType nodeType_;

    // Last received usable bands
    UsableBands usableBands_;


    // ID of the coordinator
    X2NodeId coordinatorId_;

    // IDs of the eNB that are slaves of this master node
    std::vector<X2NodeId> clientList_;

    // statistics
    simsignal_t compReservedBlocks_;

    void runClientOperations();
    void runCoordinatorOperations();
    void handleX2Message(cPacket* pkt);
    void sendClientRequest(X2CompRequestIE* requestIe);
    void sendCoordinatorReply(X2NodeId clientId, X2CompReplyIE* replyIe);

    virtual void provisionalSchedule() = 0;  // run the provisional scheduling algorithm (client side)
    virtual void doCoordination() = 0;       // run the coordination algorithm (coordinator side)

    virtual X2CompRequestIE* buildClientRequest() = 0;
    virtual void handleClientRequest(X2CompMsg* compMsg) = 0;

    virtual X2CompReplyIE* buildCoordinatorReply(X2NodeId clientId) = 0;
    virtual void handleCoordinatorReply(X2CompMsg* compMsg) = 0;

    void setUsableBands(UsableBands& usableBands);

public:
    LteCompManagerBase() {}
    virtual ~LteCompManagerBase() {}

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};

#endif /* LTE_LTECOMPMANAGERBASE_H_ */
