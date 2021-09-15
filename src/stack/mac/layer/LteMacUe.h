//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_LTEMACUE_H_
#define _LTE_LTEMACUE_H_

#include "stack/mac/layer/LteMacBase.h"
#include "stack/mac/buffer/harq/LteHarqBufferTx.h"
#include "stack/phy/feedback/LteFeedback.h"

class LteSchedulingGrant;
class LteSchedulerUeUl;
class LteBinder;

class LteMacUe : public LteMacBase
{
  protected:
    // false if currentHarq_ counter needs to be initialized
    bool firstTx;

    bool isIpBased_;

    LteSchedulerUeUl* lcgScheduler_;

    // configured grant - one each codeword
    LteSchedulingGrant* schedulingGrant_;

    // current H-ARQ process counter
    unsigned char currentHarq_;

    // periodic grant handling
    unsigned int periodCounter_;
    unsigned int expirationCounter_;

    // RAC Handling variables

    bool racRequested_;
    unsigned int racBackoffTimer_;
    unsigned int maxRacTryouts_;
    unsigned int currentRacTry_;
    unsigned int minRacBackoff_;
    unsigned int maxRacBackoff_;

    unsigned int raRespTimer_;
    unsigned int raRespWinStart_;

    // BSR handling
    bool bsrTriggered_;

    // statistics
    simsignal_t cqiDlSpmux0_;
    simsignal_t cqiDlSpmux1_;
    simsignal_t cqiDlSpmux2_;
    simsignal_t cqiDlSpmux3_;
    simsignal_t cqiDlSpmux4_;
    simsignal_t cqiDlTxDiv0_;
    simsignal_t cqiDlTxDiv1_;
    simsignal_t cqiDlTxDiv2_;
    simsignal_t cqiDlTxDiv3_;
    simsignal_t cqiDlTxDiv4_;
    simsignal_t cqiDlMuMimo0_;
    simsignal_t cqiDlMuMimo1_;
    simsignal_t cqiDlMuMimo2_;
    simsignal_t cqiDlMuMimo3_;
    simsignal_t cqiDlMuMimo4_;
    simsignal_t cqiDlSiso0_;
    simsignal_t cqiDlSiso1_;
    simsignal_t cqiDlSiso2_;
    simsignal_t cqiDlSiso3_;
    simsignal_t cqiDlSiso4_;

    /**
     * Reads MAC parameters for ue and performs initialization.
     */
    virtual void initialize(int stage);

    /**
     * macPduMake() creates MAC PDUs (one for each CID)
     * by extracting SDUs from Real Mac Buffers according
     * to the Schedule List.
     * It sends them to H-ARQ (at the moment lower layer)
     *
     * On UE it also adds a BSR control element to the MAC PDU
     * containing the size of its buffer (for that CID)
     */
    virtual void macPduMake(LteMacScheduleList* scheduleList);

    /**
     * macPduUnmake() extracts SDUs from a received MAC
     * PDU and sends them to the upper layer.
     *
     * @param pkt container packet
     */
    virtual void macPduUnmake(cPacket* pkt);

    /**
     * Main loop
     */
    virtual void handleSelfMessage();

    /*
     * Receives and handles scheduling grants
     */
    virtual void macHandleGrant(cPacket* pkt);

    /*
     * Receives and handles RAC responses
     */
    virtual void macHandleRac(cPacket* pkt);

    /*
     * Checks RAC status
     */
    virtual void checkRAC();
    /*
     * Update UserTxParam stored in every lteMacPdu when an rtx change this information
     */
    virtual void updateUserTxParam(cPacket* pkt);

  public:
    LteMacUe();
    virtual ~LteMacUe();

    /*
     * Record CQI-related statistics
     */
    void collectCqiStatistics(MacNodeId id, Direction dir, LteFeedback fb);

    /*
     * Access scheduling grant
     */
    inline const LteSchedulingGrant* getSchedulingGrant() const
    {
        return schedulingGrant_;
    }
    /*
     * Access current H-ARQ pointer
     */
    inline const unsigned char getCurrentHarq() const
    {
        return currentHarq_;
    }
    /*
     * Access BSR trigger flag
     */
    inline const bool bsrTriggered() const
    {
        return bsrTriggered_;
    }

    /* utility functions used by LCP scheduler
     * <cid> and <priority> are returned by reference
     * @return true if at least one backlogged connection exists
     */
    bool getHighestBackloggedFlow(MacCid& cid, unsigned int& priority);
    bool getLowestBackloggedFlow(MacCid& cid, unsigned int& priority);

    /**
     * deleteQueues() must be called on handover
     * to delete queues for a given user
     *
     * @param nodeId Id of the node whose queues are deleted
     */
    virtual void deleteQueues(MacNodeId nodeId);

    // update ID of the serving cell during handover
    virtual void doHandover(MacNodeId targetEnb);
};

#endif
