//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_LTEMACQUEUE_H_
#define _LTE_LTEMACQUEUE_H_

#include <omnetpp.h>
#include "stack/rlc/packet/LteRlcPdu_m.h"

using namespace omnetpp;

/**
 * @class LteMacQueue
 * @brief Queue for MAC SDU packets
 *
 * The Queue registers the following informations:
 * - Packets
 * - Occupation (in bytes)
 * - Maximum size
 * - Number of packets
 * - Head Of Line Timestamp
 *
 * The Queue size can be configured, and packets are
 * dropped if stored packets exceeds the queue size
 * A size equal to 0 means that the size is infinite.
 *
 */
class LteMacQueue : public cPacketQueue
{
  public:

    /**
     * Constructor creates a new cPacketQueue
     * with configurable maximum size
     */
    LteMacQueue(int queueSize);

    virtual ~LteMacQueue()
    {
    }

    /**
     * Copy Constructors
     */

    LteMacQueue(const LteMacQueue& queue);
    LteMacQueue& operator=(const LteMacQueue& queue);
    LteMacQueue* dup() const;

    /**
     * pushBack() inserts a new packet in the back
     * of the queue (standard operation) only
     * if there is enough space left
     *
     * @param pkt packet to insert
     * @return false if queue is full,
     *            true on successful insertion
     */
    bool pushBack(cPacket *pkt);

    /**
     * pushFront() inserts a new packet in the front
     * of the queue only if there is enough space left
     *
     * @param pkt packet to insert
     * @return false if queue is full,
     *            true on successful insertion
     */
    bool pushFront(cPacket *pkt);

    /**
     * popFront() extracts a packet from the
     * front of the queue (standard operation).
     *
     * @return NULL if queue is empty,
     *            pkt on successful operation
     */
    cPacket* popFront();

    /**
     * popFront() extracts a packet from the
     * back of the queue.
     *
     * @return NULL if queue is empty,
     *            pkt on successful operation
     */
    cPacket* popBack();

    /**
     * getQueueOccupancy() returns the occupancy
     * of the queue (in bytes)
     *
     * @return queue occupancy
     */
    int64_t getQueueOccupancy() const;

    /**
     * getQueueSize() returns the maximum
     * size of the queue (in bytes)
     *
     * @return queue size
     */
    int64_t getQueueSize() const;

    /**
     * getQueueLength() returns the number
     * of packets in the queue
     *
     * @return #packets in queue
     */
    int getQueueLength() const;

    /**
     * getHolTimestamp() returns the timestamp
     * of the Head Of Line (front) packet of the queue
     *
     * @return Hol Timestamp (0 if queue empty)
     */
    simtime_t getHolTimestamp() const;

    friend std::ostream &operator << (std::ostream &stream, const LteMacQueue* queue);

  protected:
    /**
     * Check if it makes sense to enqueue this packet.
     *
     * The check is based on the following assumptions:
     * 1) unfragmented PDUs can always be enqueued
     * 2) PDU fragments should be enqueued if
     *    a) no previous fragment of the same PDU was discarded before
     *    b) we have enough space in the queue to hold all remaining fragments of the same packet
     *
     */
    bool isEnqueueablePacket(cPacket* pkt);
    unsigned int lastUnenqueueableMainSno; //<seq. number of
  private:
    /// Size of queue
    int queueSize_;
};

#endif
